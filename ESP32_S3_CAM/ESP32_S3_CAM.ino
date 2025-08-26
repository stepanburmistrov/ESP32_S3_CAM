/*********************************************************************
 *  ESP32-S3 WROOM (Freenove) + OV3660 — UNIVERSAL FW v1.5 (S3)
 *  • Неблокирующий MJPEG-стрим (1 кадр/loop).
 *  • Настройки в NVS ("cfg"): Wi-Fi + Res/FPS/Flash(LED GPIO2).
 *  • UART 115200 8-N-1: SETWIFI|..., RESETCFG, SETRES|..., SETFPS|..., FLASH|..., BLINK|...
 *  • HTTP:
 *      /        -> простая страница с <img src="/stream">
 *      /stream  -> multipart/x-mixed-replace MJPEG
 *
 *  Пины камеры (Freenove ESP32-S3 WROOM, OV3660):
 *    XCLK=15, PCLK=13, VSYNC=6, HREF=7, SIOD=4, SIOC=5,
 *    D0..D7: 11, 9, 8, 10, 12, 18, 17, 16
 *********************************************************************/

#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <Preferences.h>
#include "driver/ledc.h"

/* ───── compile-time switches ───── */
#define DEBUG_UART   0       // 1 = печать FPS/heap каждые 5 с
#define BLINK_PERIOD 0       // мс; 0 = heartbeat выкл
#define BLINK_ON_MS  100

/* ───── defaults ───── */
#define LED_PIN        2     // у Freenove бортовой LED на GPIO2
#define DEF_RES        FRAMESIZE_QVGA
#define DEF_FPS        10
#define DEF_FLASH_PCT  0
#define JPEG_QUALITY   15
#define XCLK_HZ        20000000  // 20 МГц (при проблемах можно 10000000)
#define UART_BAUD      115200

/* ───── Freenove ESP32-S3 WROOM + OV3660 pin-map ───── */
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y2_GPIO_NUM    11  // D0
#define Y3_GPIO_NUM    9   // D1
#define Y4_GPIO_NUM    8   // D2
#define Y5_GPIO_NUM    10  // D3
#define Y6_GPIO_NUM    12  // D4
#define Y7_GPIO_NUM    18  // D5
#define Y8_GPIO_NUM    17  // D6
#define Y9_GPIO_NUM    16  // D7
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

/* ───── globals ───── */
Preferences  prefs;
WiFiServer*  server = nullptr;
WiFiClient   camClient;

framesize_t frameSize;
uint8_t     fps;
uint16_t    frameInterval;      // мс между кадрами
uint8_t     flashPct;           // 0–100 %
uint8_t     ledDuty;            // 0–255 (12-бит /16)

enum WStat { ST_EMPTY, ST_CONNECTING, ST_IP, ST_ERR_WIFI, ST_ERR_IP };
WStat wifiState = ST_EMPTY;

String   ssid, pass;
uint32_t tStatus = 0, tBlink = 0, tBlinkHold = 0;
bool     ledTempOn = false;

#if DEBUG_UART
uint32_t frameCnt = 0, tDiag = 0;
#define D(...) Serial.printf(__VA_ARGS__)
#else
#define D(...)
#endif

/* =================================================================
   LED PWM (GPIO2) — IDF LEDC, отдельный таймер от камеры
   =================================================================*/
#define LEDC_LED_MODE   LEDC_LOW_SPEED_MODE
#define LEDC_LED_TIMER  LEDC_TIMER_2        // камера использует TIMER_0
#define LEDC_LED_CH     LEDC_CHANNEL_4      // любой свободный канал
#define LEDC_LED_FREQ   5000
#define LEDC_LED_BITS   LEDC_TIMER_12_BIT   // 0..4095

void ledcInit() {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_LED_MODE;
  t.timer_num       = LEDC_LED_TIMER;
  t.duty_resolution = LEDC_LED_BITS;
  t.freq_hz         = LEDC_LED_FREQ;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t ch = {};
  ch.speed_mode = LEDC_LED_MODE;
  ch.channel    = LEDC_LED_CH;
  ch.gpio_num   = LED_PIN;
  ch.timer_sel  = LEDC_LED_TIMER;
  ch.intr_type  = LEDC_INTR_DISABLE;
  ch.duty       = 0;
  ch.hpoint     = 0;
  ledc_channel_config(&ch);
}
inline void setLedDuty(uint8_t duty) {
  ledDuty = duty;
  uint32_t val = (uint32_t)duty * 4095 / 255; // 8-бит → 12-бит
  ledc_set_duty(LEDC_LED_MODE, LEDC_LED_CH, val);
  ledc_update_duty(LEDC_LED_MODE, LEDC_LED_CH);
}

/* =================================================================
   Camera init / re-init (OV3660)
   =================================================================*/
bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel   = LEDC_CHANNEL_0;     // XCLK канал
  c.ledc_timer     = LEDC_TIMER_0;       // XCLK таймер (не пересекать с LED)
  c.pin_d0         = Y2_GPIO_NUM;
  c.pin_d1         = Y3_GPIO_NUM;
  c.pin_d2         = Y4_GPIO_NUM;
  c.pin_d3         = Y5_GPIO_NUM;
  c.pin_d4         = Y6_GPIO_NUM;
  c.pin_d5         = Y7_GPIO_NUM;
  c.pin_d6         = Y8_GPIO_NUM;
  c.pin_d7         = Y9_GPIO_NUM;
  c.pin_xclk       = XCLK_GPIO_NUM;
  c.pin_pclk       = PCLK_GPIO_NUM;
  c.pin_vsync      = VSYNC_GPIO_NUM;
  c.pin_href       = HREF_GPIO_NUM;
  c.pin_sscb_sda   = SIOD_GPIO_NUM;
  c.pin_sscb_scl   = SIOC_GPIO_NUM;
  c.pin_pwdn       = PWDN_GPIO_NUM;
  c.pin_reset      = RESET_GPIO_NUM;

  c.xclk_freq_hz   = XCLK_HZ;
  c.pixel_format   = PIXFORMAT_JPEG;
  c.frame_size     = frameSize;
  c.jpeg_quality   = JPEG_QUALITY;
  c.fb_count       = psramFound() ? 2 : 1;
  c.grab_mode      = CAMERA_GRAB_LATEST;
  c.fb_location    = CAMERA_FB_IN_PSRAM;

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("STATUS|ERR=CAM");
    return false;
  }

  // Небольшая настройка OV3660 (часто вверх ногами)
  sensor_t* s = esp_camera_sensor_get();
  if (s && s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // поставьте 0, если изображение правильной ориентации
  }
  Serial.printf("STATUS|CAM=OK\n");
  return true;
}
bool reInitCamera() {
  esp_camera_deinit();
  return initCamera();
}

/* =================================================================
   Wi-Fi helpers
   =================================================================*/
void sendStatus() {
  if (wifiState == ST_IP)        Serial.printf("STATUS|IP=%s\r\n", WiFi.localIP().toString().c_str());
  else if (wifiState == ST_ERR_WIFI) Serial.println("STATUS|ERR=NO_WIFI");
  else if (wifiState == ST_ERR_IP)   Serial.println("STATUS|ERR=NO_IP");
  else                                Serial.println("STATUS|ERR=NO_CFG");
}
void connectWifi(const String& s, const String& p) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(s.c_str(), p.c_str());
  wifiState = ST_CONNECTING;

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(100);

  if (WiFi.status() != WL_CONNECTED) { wifiState = ST_ERR_WIFI; return; }
  if (WiFi.localIP() == IPAddress(0,0,0,0)) { wifiState = ST_ERR_IP; return; }
  wifiState = ST_IP;
}

/* =================================================================
   Utils: строка → framesize_t
   =================================================================*/
framesize_t str2size(const String& s) {
  if (s == "QQVGA") return FRAMESIZE_QQVGA;
  if (s == "QVGA")  return FRAMESIZE_QVGA;
  if (s == "HVGA")  return FRAMESIZE_HVGA;
  if (s == "VGA")   return FRAMESIZE_VGA;
  if (s == "SVGA")  return FRAMESIZE_SVGA;
  if (s == "XGA")   return FRAMESIZE_XGA;
  if (s == "SXGA")  return FRAMESIZE_SXGA;
  if (s == "UXGA")  return FRAMESIZE_UXGA;
  return (framesize_t)255;
}

/* =================================================================
   UART parser
   =================================================================*/
void handleUart() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { buf += c; continue; }

    buf.trim();
    if (buf.startsWith("SETWIFI|")) {
      int p = buf.indexOf('|', 8);
      if (p > 8) {
        ssid = buf.substring(8, p);
        pass = buf.substring(p + 1);
        prefs.putString("ssid", ssid);
        prefs.putString("pass", pass);
        connectWifi(ssid, pass);
        Serial.println(wifiState == ST_IP ? "OK" : "ERR");
      }
    } else if (buf == "RESETCFG") {
      prefs.clear(); Serial.println("OK"); delay(50); ESP.restart();
    } else if (buf.startsWith("SETRES|")) {
      framesize_t ns = str2size(buf.substring(7));
      if (ns != 255) { frameSize = ns; prefs.putUChar("res", ns); Serial.println(reInitCamera() ? "OK" : "ERR"); }
      else Serial.println("ERR");
    } else if (buf.startsWith("SETFPS|")) {
      int nf = buf.substring(7).toInt();
      if (nf >= 1 && nf <= 30) { fps = nf; frameInterval = 1000 / fps; prefs.putUChar("fps", fps); Serial.println("OK"); }
      else Serial.println("ERR");
    } else if (buf.startsWith("FLASH|")) {
      int pct = buf.substring(6).toInt();
      if (pct >= 0 && pct <= 100) { flashPct = pct; setLedDuty(pct * 2.55); prefs.putUChar("flash", pct); Serial.println("OK"); }
      else Serial.println("ERR");
    } else if (buf.startsWith("BLINK|")) {
      uint32_t d = buf.substring(6).toInt();
      if (d > 0) { setLedDuty(255); ledTempOn = true; tBlinkHold = millis() + d; Serial.println("OK"); }
      else Serial.println("ERR");
    } else Serial.println("ERR");

    buf = "";
  }
}

/* =================================================================
   LED heartbeat / blink service
   =================================================================*/
void ledService() {
  uint32_t now = millis();
  if (ledTempOn && now >= tBlinkHold) { setLedDuty(flashPct * 2.55); ledTempOn = false; }
  if (BLINK_PERIOD > 0 && !ledTempOn && flashPct == 0) {
    if (now - tBlink >= BLINK_PERIOD) { setLedDuty(255); tBlink = now; }
    else if (now - tBlink >= BLINK_ON_MS) setLedDuty(0);
  }
}

/* =================================================================
   HTTP service: "/" (HTML) и "/stream" (MJPEG)
   =================================================================*/
void serviceHttp() {
  if (!server) return;

  // состояние простой автомата
  enum { IDLE, STREAM } static st = IDLE;

  // если нет клиента или он отвалился — пробуем принять нового
  if (!camClient || !camClient.connected()) {
    st = IDLE;
    camClient = server->available();
    return;
  }

  // только что подключился? прочтём первую строку запроса
  if (st == IDLE) {
    if (!camClient.available()) return;
    String req = camClient.readStringUntil('\n'); // "GET /... HTTP/1.1"
    req.trim();

    if (req.indexOf("GET /stream") >= 0) {
      camClient.print(
        "HTTP/1.1 200 OK\r\n"
        "Cache-Control: no-store, no-cache, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
      st = STREAM;
      return;
    } else {
      camClient.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Connection: close\r\n\r\n"
        "<!doctype html><html><head><meta charset='utf-8'><title>ESP32-S3 Cam</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'/>"
        "<style>body{margin:0;background:#111;color:#eee;font-family:system-ui}header{padding:10px 14px}img{width:100vw;height:calc(100vh - 48px);object-fit:contain}</style>"
        "</head><body><header>ESP32-S3 Cam — <code>/stream</code></header><img src='/stream' alt='stream'></body></html>");
      camClient.stop();
      return;
    }
  }

  // отправка кадров по таймеру FPS
  static uint32_t tLast = 0;
  if (millis() - tLast < frameInterval) return;
  tLast = millis();

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { camClient.stop(); st = IDLE; return; }

  camClient.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
  camClient.write(fb->buf, fb->len);
  camClient.print("\r\n");
  esp_camera_fb_return(fb);

#if DEBUG_UART
  frameCnt++;
#endif
}

/* =================================================================
   SETUP
   =================================================================*/
void setup() {
  Serial.begin(UART_BAUD);
  prefs.begin("cfg", false);

  // загрузка настроек
  frameSize = (framesize_t)prefs.getUChar("res", DEF_RES);
  fps       = prefs.getUChar("fps",  DEF_FPS);
  if (fps < 1 || fps > 30) fps = DEF_FPS;
  frameInterval = 1000 / fps;

  flashPct  = prefs.getUChar("flash", DEF_FLASH_PCT);

  // 1) СНАЧАЛА камера (занимает LEDC TIMER_0)
  if (!initCamera()) {
    Serial.println("STATUS|ERR=CAM");
  }
  Serial.printf("heap=%u psram=%u\n", ESP.getFreeHeap(), ESP.getFreePsram());

  // 2) ПОТОМ LED-PWM на другом таймере
  ledcInit();
  setLedDuty(flashPct * 2.55);

  // 3) Wi-Fi + HTTP
  WiFi.mode(WIFI_STA);
  server = new WiFiServer(80);
  server->begin();

  ssid = prefs.getString("ssid", "");
  pass = prefs.getString("pass", "");
  if (ssid.length()) connectWifi(ssid, pass);
}

/* =================================================================
   LOOP
   =================================================================*/
void loop() {
  handleUart();
  ledService();
  serviceHttp();

  if (millis() - tStatus >= 2000) {
    tStatus = millis();
    sendStatus();
  }

#if DEBUG_UART
  if (millis() - tDiag >= 5000) {
    tDiag = millis();
    D("[diag] fps=%u heap=%u psram=%u\n", frameCnt / 5, ESP.getFreeHeap(), ESP.getFreePsram());
    frameCnt = 0;
  }
#endif
}
