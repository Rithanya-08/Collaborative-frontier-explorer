// Robot B camera: ESP32-CAM (AI Thinker, OV2640) streaming MJPEG over WiFi.
// No wires to the DevKit; the laptop opens the stream in a browser or OpenCV.
//
//   http://<ip>/          page showing the live stream
//   http://<ip>:81/stream MJPEG stream (cv2.VideoCapture works with this URL)
//   http://<ip>/capture   single JPEG
//   http://<ip>/rotate?on=1|0  turn the image 180 degrees (saved; on by
//                         default because the camera is mounted upside down on Robot B)
//
// The stream URL is also published (retained) on MQTT robot/B/cam, so the
// laptop can find the camera without the serial monitor.
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "secrets.h"

#define ROBOT_ID "B"
const char *T_CAM = "robot/" ROBOT_ID "/cam";

// AI Thinker ESP32-CAM pin map
#define PWDN_GPIO 32
#define RESET_GPIO -1
#define XCLK_GPIO 0
#define SIOD_GPIO 26
#define SIOC_GPIO 27
#define Y9_GPIO 35
#define Y8_GPIO 34
#define Y7_GPIO 39
#define Y6_GPIO 36
#define Y5_GPIO 21
#define Y4_GPIO 19
#define Y3_GPIO 18
#define Y2_GPIO 5
#define VSYNC_GPIO 25
#define HREF_GPIO 23
#define PCLK_GPIO 22
const int LED_RED = 33;  // on-board red LED, active LOW

WiFiClient wifi;
PubSubClient mqtt(wifi);
httpd_handle_t webServer = nullptr, streamServer = nullptr;
Preferences prefs;

// 180 degrees = vertical flip + horizontal mirror, done by the sensor itself
void setRotate180(bool on) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  if (s->set_vflip) s->set_vflip(s, on);
  if (s->set_hmirror) s->set_hmirror(s, on);
}

// Some ESP32-CAM boards ship with a sensor that has no hardware JPEG (this
// one reported "JPEG format is not supported on this sensor" instead of
// being an OV2640). Then frames are captured as RGB565 and JPEG-encoded in
// software - smaller and slower, but the stream works the same.
const int SOFT_JPEG_QUALITY = 60;  // 0-100, for frame2jpg()

bool initCamera(bool hardwareJpeg) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO; c.pin_d1 = Y3_GPIO; c.pin_d2 = Y4_GPIO; c.pin_d3 = Y5_GPIO;
  c.pin_d4 = Y6_GPIO; c.pin_d5 = Y7_GPIO; c.pin_d6 = Y8_GPIO; c.pin_d7 = Y9_GPIO;
  c.pin_xclk = XCLK_GPIO; c.pin_pclk = PCLK_GPIO; c.pin_vsync = VSYNC_GPIO; c.pin_href = HREF_GPIO;
  c.pin_sccb_sda = SIOD_GPIO; c.pin_sccb_scl = SIOC_GPIO;
  c.pin_pwdn = PWDN_GPIO; c.pin_reset = RESET_GPIO;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = hardwareJpeg ? PIXFORMAT_JPEG : PIXFORMAT_RGB565;
  c.grab_mode = CAMERA_GRAB_LATEST;  // always send the newest frame, never a backlog
  if (!hardwareJpeg) {
    c.frame_size = FRAMESIZE_QVGA;  // 320x240: raw frames are 150 KB, software JPEG keeps up
    c.fb_count = psramFound() ? 2 : 1;
    c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  } else if (psramFound()) {
    c.frame_size = FRAMESIZE_VGA;  // 640x480
    c.jpeg_quality = 12;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 15;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
  }
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init (%s) failed: 0x%x\n", hardwareJpeg ? "JPEG" : "RGB565", err);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("Camera sensor PID 0x%04x, %s\n", s ? s->id.PID : 0,
                hardwareJpeg ? "hardware JPEG" : "software JPEG (RGB565 capture)");
  return true;
}

// JPEG bytes of a frame; *owned is true when buf was allocated here and must be freed
bool frameJpeg(camera_fb_t *fb, uint8_t **buf, size_t *len, bool *owned) {
  if (fb->format == PIXFORMAT_JPEG) { *buf = fb->buf; *len = fb->len; *owned = false; return true; }
  *owned = true;
  return frame2jpg(fb, SOFT_JPEG_QUALITY, buf, len);
}

// ---------- HTTP ----------
esp_err_t indexHandler(httpd_req_t *req) {
  char html[400];
  snprintf(html, sizeof(html),
           "<!doctype html><html><head><meta name=viewport content='width=device-width'>"
           "<title>Robot B camera</title></head><body style='margin:0;background:#111'>"
           "<img src='http://%s:81/stream' style='width:100%%;max-width:800px;display:block;margin:auto'>"
           "</body></html>",
           WiFi.localIP().toString().c_str());
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  uint8_t *jpg; size_t len; bool owned;
  bool ok = frameJpeg(fb, &jpg, &len, &owned);
  esp_camera_fb_return(fb);
  if (!ok) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t res = httpd_resp_send(req, (const char *)jpg, len);
  if (owned) free(jpg);
  return res;
}

esp_err_t rotateHandler(httpd_req_t *req) {
  char query[16], val[4];
  bool on = true;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
      httpd_query_key_value(query, "on", val, sizeof(val)) == ESP_OK)
    on = val[0] == '1';
  setRotate180(on);
  prefs.putBool("rot180", on);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, on ? "rotate 180: on\n" : "rotate 180: off\n");
}

#define BOUNDARY "frame"
esp_err_t streamHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char part[80];
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    uint8_t *jpg; size_t len; bool owned;
    bool ok = frameJpeg(fb, &jpg, &len, &owned);
    if (!owned) {  // hardware JPEG: send straight from the frame buffer
      int n = snprintf(part, sizeof(part), "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", len);
      esp_err_t res = httpd_resp_send_chunk(req, part, n);
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpg, len);
      esp_camera_fb_return(fb);
      if (res != ESP_OK) return res;  // viewer closed the stream
      continue;
    }
    esp_camera_fb_return(fb);  // software JPEG is a separate buffer; free the frame early
    if (!ok) return ESP_FAIL;
    int n = snprintf(part, sizeof(part), "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", len);
    esp_err_t res = httpd_resp_send_chunk(req, part, n);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpg, len);
    free(jpg);
    if (res != ESP_OK) return res;  // viewer closed the stream
  }
}

void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  httpd_uri_t index = {"/", HTTP_GET, indexHandler, nullptr};
  httpd_uri_t capture = {"/capture", HTTP_GET, captureHandler, nullptr};
  httpd_uri_t rotate = {"/rotate", HTTP_GET, rotateHandler, nullptr};
  if (httpd_start(&webServer, &cfg) == ESP_OK) {
    httpd_register_uri_handler(webServer, &index);
    httpd_register_uri_handler(webServer, &capture);
    httpd_register_uri_handler(webServer, &rotate);
  }
  // Stream on its own server/port so a running stream doesn't block "/" or "/capture"
  cfg.server_port = 81;
  cfg.ctrl_port += 1;
  httpd_uri_t stream = {"/stream", HTTP_GET, streamHandler, nullptr};
  if (httpd_start(&streamServer, &cfg) == ESP_OK) httpd_register_uri_handler(streamServer, &stream);
}

// ---------- MQTT ----------
// Same broker discovery as robot_b: the hotspot's subnet changes each time
// it is switched on, so probe the camera's own /24 for port 1883.
uint32_t mqttRetryAt = 0;
int mqttFailures = 0;
IPAddress brokerIp;

bool findBroker() {
  IPAddress me = WiFi.localIP();
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
  return false;
}

void maintainMqtt() {
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (WiFi.status() != WL_CONNECTED || (int32_t)(millis() - mqttRetryAt) < 0) return;
  mqttRetryAt = millis() + 5000;
  IPAddress me = WiFi.localIP();
  bool otherSubnet = me[0] != brokerIp[0] || me[1] != brokerIp[1] || me[2] != brokerIp[2];
  if (otherSubnet || mqttFailures >= 2) {
    mqttFailures = 0;
    if (!findBroker()) return;
  }
  if (mqtt.connect("robot" ROBOT_ID "-cam", nullptr, nullptr, T_CAM, 1, true, "offline")) {
    mqttFailures = 0;
    String url = "http://" + WiFi.localIP().toString() + ":81/stream";
    mqtt.publish(T_CAM, url.c_str(), true);
    Serial.printf("MQTT: published %s on %s\n", url.c_str(), T_CAM);
  } else {
    mqttFailures++;
  }
}

void setup() {
  // Power-bank supply through jumper wires sags when the camera and WiFi
  // start, which reset-looped the board on "Brownout detector was triggered"
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial.println("\nboot: camera init");
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);  // off
  bool camOk = initCamera(true);
  if (!camOk) {
    esp_camera_deinit();
    camOk = initCamera(false);
  }
  prefs.begin("cam", false);
  if (camOk) setRotate180(prefs.getBool("rot180", true));
  if (!camOk) {
    while (true) { digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(200); }  // fast blink = camera error
  }

  Serial.println("boot: camera ok, starting WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_11dBm);  // lower transmit current spikes; hotspot is close by
  WiFi.setSleep(false);  // power save adds stream stutter
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nCamera ready: http://%s/  (stream on :81/stream)\n", WiFi.localIP().toString().c_str());
  digitalWrite(LED_RED, LOW);  // on = connected

  startServers();
  uint32_t saved = prefs.getUInt("broker", 0);
  if (saved) brokerIp = IPAddress(saved);
  else brokerIp.fromString(MQTT_HOST);
  mqtt.setServer(brokerIp, MQTT_PORT);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_RED, HIGH);
    WiFi.reconnect();
    delay(1000);
    return;
  }
  digitalWrite(LED_RED, LOW);
  maintainMqtt();
  delay(50);
}
