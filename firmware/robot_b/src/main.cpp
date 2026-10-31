// Robot B firmware: the ESP32 robot is a "body" only. Mapping and exploration run
// on the laptop (hardware/esp32_robot.py), which sends timed moves over MQTT.
// Motor behaviour mirrors hardware/motor_controller.py on Robot A: same 0-255 speed
// scale, same ramps, pivot turns drive one side only, each move stops itself.
//
// Topics (ROBOT_ID "B"):
//   robot/B/cmd    <- {"id":7,"cmd":"fwd|back|left|right","speed":150,"ms":300}
//                     {"id":8,"cmd":"stop"}  {"id":9,"cmd":"calib"}  {"id":10,"cmd":"yaw","yaw":90}
//   robot/B/done   -> {"id":7,"result":"ok|obstacle|stopped|busy|bad_cmd"}
//   robot/B/state  -> {"yaw":12.3,"front":0.84,"left":-1,"right":1.2,"busy":0}  every 100 ms
//                     distances in metres, -1 = no echo
//   robot/B/status -> "online" / "offline" (retained, last will)
//
// Pin map matches HARDWARE_SETUP.md and firmware/hw_test.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "secrets.h"

#define ROBOT_ID "B"
const char *T_CMD = "robot/" ROBOT_ID "/cmd";
const char *T_DONE = "robot/" ROBOT_ID "/done";
const char *T_STATE = "robot/" ROBOT_ID "/state";
const char *T_STATUS = "robot/" ROBOT_ID "/status";

// Motors: direction pins shared per side, one PWM pin per wheel
// Left IN1/IN2 swapped in software: both left motors ran reversed on the real robot
const int L_IN1 = 17, L_IN2 = 16, R_IN1 = 18, R_IN2 = 19;
const int PWM_PINS[4] = {25, 26, 27, 13};  // FL, RL, FR, RR
const int MAX_DUTY = 178;                  // ~70% of 255 (TT motors rated ~6V, battery up to 8.4V)
const uint32_t MAX_MOVE_MS = 5000;         // longest single move the laptop may request

// Ramps, same as MotorController defaults (percent of full duty per step)
const float FWD_RAMP_PCT = 5.0, TURN_RAMP_PCT = 20.0;
const uint32_t FWD_RAMP_MS = 50, TURN_RAMP_MS = 30;

// Ultrasonics: shared TRIG fires all three, so read one ECHO per cycle
const int TRIG = 23;
const int ECHO_PINS[3] = {34, 35, 32};  // front, left, right
const uint32_t US_SLOT_MS = 60;         // let echoes die out before the next trigger
const float US_MAX_M = 4.0;
const float SAFETY_STOP_M = 0.15;  // last-resort stop; laptop avoidance triggers at 0.30

// IMU
const int SDA_PIN = 21, SCL_PIN = 22;
const uint8_t MPU_ADDR = 0x68;
const float GYRO_LSB_PER_DPS = 131.0;  // +-250 deg/s range
const float GYRO_SPIKE_DPS = 250.0;    // readings past full scale are motor-noise glitches
const int LED = 2;

WiFiClient wifi;
PubSubClient mqtt(wifi);
Preferences prefs;
IPAddress brokerIp;  // laptop running mosquitto; see findBroker()

// ---------- motors ----------
void setSide(bool left, int dir) {  // dir: 1 fwd, -1 back, 0 coast
  int in1 = left ? L_IN1 : R_IN1, in2 = left ? L_IN2 : R_IN2;
  digitalWrite(in1, dir > 0);
  digitalWrite(in2, dir < 0);
}

void setWheel(int i, int duty) { ledcWrite(i, constrain(duty, 0, MAX_DUTY)); }

void stopAll() {
  for (int i = 0; i < 4; i++) setWheel(i, 0);
  setSide(true, 0);
  setSide(false, 0);
}

enum Phase { IDLE, RAMP, HOLD };
struct Motion {
  Phase phase = IDLE;
  long id = 0;
  bool forward = false;  // only forward moves use the safety stop
  int target[4] = {0, 0, 0, 0};
  int steps = 1, step = 0;
  uint32_t stepMs = 0, nextAt = 0, holdMs = 0, holdUntil = 0;
} motion;

void publishDone(long id, const char *result) {
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"id\":%ld,\"result\":\"%s\"}", id, result);
  if (mqtt.connected()) mqtt.publish(T_DONE, buf);
  Serial.printf("done id=%ld %s\n", id, result);
}

void finishMotion(const char *result) {
  stopAll();
  if (motion.phase != IDLE) publishDone(motion.id, result);
  motion.phase = IDLE;
}

void startMotion(long id, const char *cmd, int speed, uint32_t ms) {
  int leftDir, rightDir, leftDuty, rightDuty;
  int duty = min(constrain(speed, 0, 255), MAX_DUTY);
  bool turn = false;
  if (!strcmp(cmd, "fwd")) { leftDir = rightDir = 1; leftDuty = rightDuty = duty; }
  else if (!strcmp(cmd, "back")) { leftDir = rightDir = -1; leftDuty = rightDuty = duty; }
  else if (!strcmp(cmd, "left")) { leftDir = 0; rightDir = 1; leftDuty = 0; rightDuty = duty; turn = true; }
  else if (!strcmp(cmd, "right")) { leftDir = 1; rightDir = 0; leftDuty = duty; rightDuty = 0; turn = true; }
  else { publishDone(id, "bad_cmd"); return; }

  if (motion.phase != IDLE) finishMotion("stopped");  // a new command replaces the old one

  motion.id = id;
  motion.forward = !strcmp(cmd, "fwd");
  motion.target[0] = motion.target[1] = leftDuty;
  motion.target[2] = motion.target[3] = rightDuty;
  float stepDuty = (turn ? TURN_RAMP_PCT : FWD_RAMP_PCT) / 100.0 * 255;
  motion.steps = max(1, (int)(duty / stepDuty));
  motion.step = 0;
  motion.stepMs = turn ? TURN_RAMP_MS : FWD_RAMP_MS;
  motion.holdMs = min(ms, MAX_MOVE_MS);
  motion.nextAt = millis();
  motion.phase = RAMP;
  setSide(true, leftDir);
  setSide(false, rightDir);
}

float frontDist();

void updateMotion() {
  if (motion.phase == IDLE) return;
  uint32_t now = millis();
  if (motion.forward) {
    float f = frontDist();
    if (f > 0 && f < SAFETY_STOP_M) { finishMotion("obstacle"); return; }
  }
  // Ramp timeline matches MotorController._drive: set step k, wait, ..., then hold
  while (motion.phase == RAMP && (int32_t)(now - motion.nextAt) >= 0) {
    motion.step++;
    for (int i = 0; i < 4; i++) setWheel(i, motion.target[i] * motion.step / motion.steps);
    motion.nextAt += motion.stepMs;
    if (motion.step >= motion.steps) {
      motion.holdUntil = motion.nextAt + motion.holdMs;
      motion.phase = HOLD;
    }
  }
  if (motion.phase == HOLD && (int32_t)(now - motion.holdUntil) >= 0) finishMotion("ok");
}

// ---------- ultrasonics ----------
volatile uint32_t echoRise[3], echoWidth[3];
volatile bool echoReady[3];
float distM[3] = {-1, -1, -1};
int usIdx = 0;
uint32_t usNextAt = 0;

void IRAM_ATTR onEcho(int i) {
  uint32_t t = micros();
  if (digitalRead(ECHO_PINS[i])) echoRise[i] = t;
  else { echoWidth[i] = t - echoRise[i]; echoReady[i] = true; }
}
void IRAM_ATTR onEcho0() { onEcho(0); }
void IRAM_ATTR onEcho1() { onEcho(1); }
void IRAM_ATTR onEcho2() { onEcho(2); }

float frontDist() { return distM[0]; }

void updateUltrasonics() {
  uint32_t now = millis();
  if ((int32_t)(now - usNextAt) < 0) return;
  usNextAt = now + US_SLOT_MS;

  // Collect the sensor triggered last slot
  if (echoReady[usIdx]) distM[usIdx] = min(echoWidth[usIdx] * 343e-6f / 2, US_MAX_M);
  else distM[usIdx] = -1;

  usIdx = (usIdx + 1) % 3;
  echoReady[usIdx] = false;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
}

// ---------- IMU ----------
float yawDeg = 0, gyroBias = 0;
uint32_t imuLastUs = 0, imuNextAt = 0;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool readGyroZ(float &dps) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) return false;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  dps = raw / GYRO_LSB_PER_DPS;
  return true;
}

void initImu() {
  mpuWrite(0x6B, 0x00);  // wake up
  mpuWrite(0x1A, 0x03);  // 44 Hz low-pass filter, cuts motor vibration
  mpuWrite(0x1B, 0x00);  // +-250 deg/s
  delay(100);
}

void calibrateGyro() {  // robot must be still; same idea as IMUDriver.calibrate_gyroscope
  float sum = 0, dps;
  int n = 0;
  for (int i = 0; i < 150; i++) {
    if (readGyroZ(dps)) { sum += dps; n++; }
    delay(10);
  }
  gyroBias = n ? sum / n : 0;
  imuLastUs = micros();
  Serial.printf("gyro bias %.3f deg/s (%d samples)\n", gyroBias, n);
}

void updateImu() {
  uint32_t now = millis();
  if ((int32_t)(now - imuNextAt) < 0) return;
  imuNextAt = now + 10;
  float dps;
  uint32_t us = micros();
  float dt = (us - imuLastUs) / 1e6f;
  imuLastUs = us;
  if (!readGyroZ(dps) || fabsf(dps) > GYRO_SPIKE_DPS) return;  // skip noisy/failed sample
  yawDeg = fmodf(yawDeg + (dps - gyroBias) * dt + 360.0f, 360.0f);
}

// ---------- MQTT ----------
void onMessage(char *topic, byte *payload, unsigned int len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) { publishDone(-1, "bad_cmd"); return; }
  long id = doc["id"] | 0L;
  const char *cmd = doc["cmd"] | "";

  if (!strcmp(cmd, "stop")) { finishMotion("stopped"); publishDone(id, "ok"); }
  else if (!strcmp(cmd, "yaw")) { yawDeg = fmodf((doc["yaw"] | 0.0f) + 360.0f, 360.0f); publishDone(id, "ok"); }
  else if (!strcmp(cmd, "calib")) {
    if (motion.phase != IDLE) { publishDone(id, "busy"); return; }
    calibrateGyro();
    publishDone(id, "ok");
  }
  else startMotion(id, cmd, doc["speed"] | 150, doc["ms"] | 0);
}

uint32_t mqttRetryAt = 0, stateNextAt = 0;
int mqttFailures = 0;

// The phone hotspot hands out a new subnet each time it is switched on, so
// the laptop's IP changes. Find the broker by probing the robot's own /24
// for port 1883 and remember it (NVS) for the next boot. Blocks up to ~20 s;
// only called while offline, when motion is already stopped.
bool findBroker() {
  IPAddress me = WiFi.localIP();
  Serial.printf("Scanning %d.%d.%d.0/24 for the MQTT broker ...\n", me[0], me[1], me[2]);
  WiFiClient probe;
  for (int i = 1; i < 255; i++) {
    IPAddress ip(me[0], me[1], me[2], i);
    if (ip == me) continue;
    if (probe.connect(ip, MQTT_PORT, 80)) {
      probe.stop();
      brokerIp = ip;
      prefs.putUInt("broker", (uint32_t)ip);
      mqtt.setServer(brokerIp, MQTT_PORT);
      Serial.printf("Broker found at %s\n", ip.toString().c_str());
      return true;
    }
  }
  Serial.println("No broker found (is mosquitto running on the laptop?)");
  return false;
}

void maintainConnection() {
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (motion.phase != IDLE) finishMotion("stopped");  // never drive blind without the laptop
  digitalWrite(LED, (millis() / 250) % 2);            // blink while offline
  if (WiFi.status() != WL_CONNECTED || (int32_t)(millis() - mqttRetryAt) < 0) return;
  mqttRetryAt = millis() + 2000;

  IPAddress me = WiFi.localIP();
  bool otherSubnet = me[0] != brokerIp[0] || me[1] != brokerIp[1] || me[2] != brokerIp[2];
  if (otherSubnet || mqttFailures >= 2) {
    mqttFailures = 0;
    if (!findBroker()) return;
  }
  Serial.printf("MQTT connecting to %s:%d ... ", brokerIp.toString().c_str(), MQTT_PORT);
  if (mqtt.connect("robot" ROBOT_ID, nullptr, nullptr, T_STATUS, 1, true, "offline")) {
    mqtt.publish(T_STATUS, "online", true);
    mqtt.subscribe(T_CMD);
    digitalWrite(LED, HIGH);
    mqttFailures = 0;
    Serial.println("ok");
  } else {
    mqttFailures++;
    Serial.printf("failed (state %d)\n", mqtt.state());
  }
}

void publishState() {
  uint32_t now = millis();
  if (!mqtt.connected() || (int32_t)(now - stateNextAt) < 0) return;
  stateNextAt = now + 100;
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"yaw\":%.1f,\"front\":%.3f,\"left\":%.3f,\"right\":%.3f,\"busy\":%d}",
           yawDeg, distM[0], distM[1], distM[2], motion.phase != IDLE);
  mqtt.publish(T_STATE, buf);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  for (int p : {L_IN1, L_IN2, R_IN1, R_IN2}) pinMode(p, OUTPUT);
  for (int i = 0; i < 4; i++) { ledcSetup(i, 1000, 8); ledcAttachPin(PWM_PINS[i], i); }
  stopAll();

  pinMode(TRIG, OUTPUT);
  for (int p : ECHO_PINS) pinMode(p, INPUT);
  attachInterrupt(ECHO_PINS[0], onEcho0, CHANGE);
  attachInterrupt(ECHO_PINS[1], onEcho1, CHANGE);
  attachInterrupt(ECHO_PINS[2], onEcho2, CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN);
  initImu();
  Serial.println("Calibrating gyro - keep the robot still");
  calibrateGyro();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // lower latency for move commands
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi connecting to %s\n", WIFI_SSID);

  // Last broker that worked, else secrets.h; findBroker() corrects it if stale
  prefs.begin("robot", false);
  uint32_t saved = prefs.getUInt("broker", 0);
  if (saved) brokerIp = IPAddress(saved);
  else brokerIp.fromString(MQTT_HOST);
  mqtt.setServer(brokerIp, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setKeepAlive(5);  // laptop sees "offline" within ~8 s of a crash
  mqtt.setBufferSize(512);
}

void loop() {
  maintainConnection();
  updateImu();
  updateUltrasonics();
  updateMotion();
  publishState();
}
