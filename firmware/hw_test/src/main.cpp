// Hardware bring-up test. Open the serial monitor (115200) and type a command.
// Pin map matches HARDWARE_SETUP.md. Keep the wheels OFF the ground.
#include <Arduino.h>
#include <Wire.h>

// Motors: direction pins shared per side, one PWM pin per wheel
// Left IN1/IN2 swapped in software: both left motors ran reversed on the real robot
const int L_IN1 = 17, L_IN2 = 16, R_IN1 = 18, R_IN2 = 19;
const int PWM_FL = 25, PWM_RL = 26, PWM_FR = 27, PWM_RR = 13;
const int PWM_PINS[4] = {PWM_FL, PWM_RL, PWM_FR, PWM_RR};
const char *WHEEL_NAMES[4] = {"Front-Left", "Rear-Left", "Front-Right", "Rear-Right"};
const int MAX_DUTY = 178;   // ~70% of 255 (TT motors rated ~6V, battery up to 8.4V)
const int TEST_DUTY = 130;

// Ultrasonics: shared TRIG, one ECHO each
const int TRIG = 23;
const int ECHO_PINS[3] = {34, 35, 32};
const char *US_NAMES[3] = {"Front", "Left", "Right"};

// IMU
const int SDA_PIN = 21, SCL_PIN = 22;
const uint8_t MPU_ADDR = 0x68;
const int LED = 2;

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

void testWheel(int i) {
  bool left = i < 2;
  Serial.printf("%s: FORWARD 1s...\n", WHEEL_NAMES[i]);
  setSide(left, 1);  setWheel(i, TEST_DUTY); delay(1000);
  stopAll(); delay(500);
  Serial.printf("%s: BACKWARD 1s...\n", WHEEL_NAMES[i]);
  setSide(left, -1); setWheel(i, TEST_DUTY); delay(1000);
  stopAll();
  Serial.println("Did ONLY that wheel spin, forward first? If backward first, swap its two motor wires.");
}

void drive(int leftDir, int rightDir, const char *label) {
  Serial.printf("%s for 1.5s\n", label);
  setSide(true, leftDir);
  setSide(false, rightDir);
  for (int i = 0; i < 4; i++) setWheel(i, TEST_DUTY);
  delay(1500);
  stopAll();
}

float readUltrasonicCm(int idx) {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO_PINS[idx], HIGH, 30000);  // ~5 m timeout
  return us == 0 ? -1 : us / 58.0;
}

void testUltrasonics() {
  Serial.println("Ultrasonic readings (10 rounds). Hold a hand in front of each sensor:");
  for (int r = 0; r < 10; r++) {
    for (int s = 0; s < 3; s++) {
      float d = readUltrasonicCm(s);
      if (d < 0) Serial.printf("%s:  ---  ", US_NAMES[s]);
      else Serial.printf("%s: %6.1f cm  ", US_NAMES[s], d);
      delay(60);  // let echoes die out (all sensors fire on the shared TRIG)
    }
    Serial.println();
  }
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

int16_t mpuRead16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  return (Wire.read() << 8) | Wire.read();
}

void i2cScan() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf("  device at 0x%02X\n", a); found++; }
  }
  if (!found) Serial.println("  nothing found - check SDA=21, SCL=22, VCC=3.3V, GND");
}

void testImu() {
  i2cScan();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x75); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  if (!Wire.available()) { Serial.println("MPU6050 not responding at 0x68 (is AD0 tied to GND?)"); return; }
  Serial.printf("WHO_AM_I = 0x%02X (0x68 genuine; 0x70/0x98 = clone, usually fine)\n", Wire.read());
  mpuWrite(0x6B, 0x00);  // wake up
  delay(100);
  Serial.println("Keep still, then rotate the robot by hand. Gyro Z should be ~0 at rest:");
  for (int i = 0; i < 20; i++) {
    float ax = mpuRead16(0x3B) / 16384.0, ay = mpuRead16(0x3D) / 16384.0, az = mpuRead16(0x3F) / 16384.0;
    float gz = mpuRead16(0x47) / 131.0;
    Serial.printf("accel g: %5.2f %5.2f %5.2f   gyroZ: %7.2f deg/s\n", ax, ay, az, gz);
    delay(250);
  }
  Serial.println("Flat on the table, accel Z should read about 1.00 g.");
}

void printMenu() {
  Serial.println(
    "\n=== Hardware test ===\n"
    " 1-4  spin one wheel (1 FL, 2 RL, 3 FR, 4 RR)\n"
    " f/b  all wheels forward / backward\n"
    " l/r  spin in place left / right\n"
    " s    stop\n"
    " u    ultrasonic sensors\n"
    " i    I2C scan + MPU6050\n"
    " ?    this menu");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  for (int p : {L_IN1, L_IN2, R_IN1, R_IN2}) pinMode(p, OUTPUT);
  for (int i = 0; i < 4; i++) { ledcSetup(i, 1000, 8); ledcAttachPin(PWM_PINS[i], i); }
  stopAll();
  pinMode(TRIG, OUTPUT);
  for (int p : ECHO_PINS) pinMode(p, INPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  digitalWrite(LED, HIGH);
  printMenu();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case '1': case '2': case '3': case '4': testWheel(c - '1'); break;
    case 'f': drive(1, 1, "Forward"); break;
    case 'b': drive(-1, -1, "Backward"); break;
    case 'l': drive(-1, 1, "Spin left"); break;
    case 'r': drive(1, -1, "Spin right"); break;
    case 's': stopAll(); Serial.println("Stopped"); break;
    case 'u': testUltrasonics(); break;
    case 'i': testImu(); break;
    case '?': printMenu(); break;
    default: return;  // ignore newlines etc.
  }
  Serial.println("> done");
}
