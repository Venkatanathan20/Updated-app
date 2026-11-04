#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <algorithm>

// ================= 1. NETWORK & CLOUD CREDENTIALS =================
const char* DEFAULT_WIFI_SSID = "IB707";
const char* DEFAULT_WIFI_PASS = "IB707IB707";
const char* CAPTIVE_AP_SSID   = "ESP32_UV_WiFi_Setup";

// HiveMQ Cloud Broker (Always syncs Mobile App & Grafana over any internet)
const char* MQTT_BROKER       = "broker.hivemq.com";
const int   MQTT_PORT         = 1883;
const char* MQTT_TOPIC        = "venkat_iot/sensors/uv/node_01";

// InfluxDB Cloud Config (Optional: Can also be set dynamically from the phone app!)
String influxUrl   = ""; 
String influxToken = ""; 

// ================= 2. HARDWARE PINOUT =============================
#define UV_SENSOR_PIN 2   // GUVA-S12SD SIG pin (ADC1_CH2)
#define I2C_SDA_PIN   8   // SSD1306 OLED SDA
#define I2C_SCL_PIN   9   // SSD1306 OLED SCL
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WebServer server(80);
DNSServer dnsServer;
Preferences prefs;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ================= 3. TASK 1: SUPPLIED GUVA-S12SD LOOKUP ==========
// Exact Vout (mV) thresholds from Supplied Reference Table (UVI 0 to 11+):
const float UVI_MV_TABLE[12] = {
  50.0f, 227.0f, 318.0f, 408.0f, 503.0f, 606.0f,
  696.0f, 795.0f, 881.0f, 976.0f, 1079.0f, 1170.0f
};

int calculateLookupIntegerUVI(float mv) {
  if (mv < 50.0f) return 0;
  for (int uvi = 11; uvi >= 1; uvi--) {
    if (mv >= UVI_MV_TABLE[uvi]) return uvi;
  }
  return 0;
}

float calculateCalibratedUVI(float mv) {
  if (mv < 50.0f) return 0.0f; // Clamped <50mV dark offset floor
  if (mv >= 1170.0f) return 11.0f + (mv - 1170.0f) / 91.0f;
  for (int i = 0; i < 11; i++) {
    float vLow  = UVI_MV_TABLE[i];
    float vHigh = UVI_MV_TABLE[i + 1];
    if (mv >= vLow && mv <= vHigh) {
      return (float)i + (mv - vLow) / (vHigh - vLow);
    }
  }
  return 0.0f;
}

const char* getRiskCategory(float uvi) {
  if (uvi < 3.0f)  return "Low";
  if (uvi < 6.0f)  return "Moderate";
  if (uvi < 8.0f)  return "High";
  if (uvi < 11.0f) return "Very high";
  return "Extreme";
}

// ================= 4. 6-STAGE DIGITAL NOISE FILTER ENGINE =========
#define FILTER_WIN 9
float winBuffer[FILTER_WIN] = {0};
int   winIdx = 0;
bool  winFilled = false;

float emaVal = 0.0f;
const float EMA_ALPHA = 0.25f;

// Standalone 1D Kalman State
float kalX = 0.0f, kalP = 1.0f;
// Hybrid (Median -> Kalman) State
float hybX = 0.0f, hybP = 1.0f;
const float KAL_Q = 0.05f, KAL_R = 2.50f;

struct FilterPack {
  float raw;
  float ma;
  float ema;
  float median;
  float kalman;
  float hybrid;
};

FilterPack fOut = {0, 0, 0, 0, 0, 0};
float uviCalibrated = 0.0f;
float uviLinearOld  = 0.0f;
int   uviLookupInt  = 0;
String riskCategory = "Low";
int   currentRssi   = 0;
uint32_t packetSeq  = 0;
unsigned long lastSampleMs = 0;
const unsigned long SAMPLE_INTERVAL_MS = 1000;

void sampleAndCompute6Filters() {
  // 1. Raw Unfiltered Single Sample
  float raw = (float)analogReadMilliVolts(UV_SENSOR_PIN);

  winBuffer[winIdx] = raw;
  winIdx = (winIdx + 1) % FILTER_WIN;
  if (winIdx == 0) winFilled = true;
  int count = winFilled ? FILTER_WIN : winIdx;

  // 2. Moving Average (MA)
  float sum = 0.0f;
  for (int i = 0; i < count; i++) sum += winBuffer[i];
  float ma = sum / (float)count;

  // 3. Exponential Moving Average (EMA)
  if (packetSeq <= 1) emaVal = raw;
  else emaVal = (EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * emaVal);

  // 4. Median Filter (Window = 9)
  float sorted[FILTER_WIN];
  for (int i = 0; i < count; i++) sorted[i] = winBuffer[i];
  std::sort(sorted, sorted + count);
  float med = sorted[count / 2];

  // 5. 1D Kalman Filter (on Raw)
  if (packetSeq <= 1) kalX = raw;
  kalP += KAL_Q;
  float kGain = kalP / (kalP + KAL_R);
  kalX += kGain * (raw - kalX);
  kalP = (1.0f - kGain) * kalP;

  // 6. Hybrid Filter (Median Spike Rejection -> 1D Kalman Smoothing)
  if (packetSeq <= 1) hybX = med;
  hybP += KAL_Q;
  float hGain = hybP / (hybP + (KAL_R * 0.6f));
  hybX += hGain * (med - hybX);
  hybP = (1.0f - hGain) * hybP;

  fOut = {raw, ma, emaVal, med, kalX, hybX};

  // Compute Task 1 Calibrated UVI using Hybrid stabilized voltage
  uviLookupInt  = calculateLookupIntegerUVI(hybX);
  uviCalibrated = calculateCalibratedUVI(hybX);
  uviLinearOld  = hybX / 100.0f;
  riskCategory  = getRiskCategory(uviCalibrated);
}

// ================= 5. HTTP API & PHONE-STYLE WIFI MANAGER =========
void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
}

void handleJsonData() {
  addCorsHeaders();
  JsonDocument doc;
  doc["seq"]                 = packetSeq;
  doc["ip"]                  = WiFi.localIP().toString();
  doc["ssid"]                = WiFi.SSID();
  doc["rssi_dbm"]            = currentRssi;
  doc["mqtt_connected"]      = mqttClient.connected();

  // All 6 Filter Outputs (in mV)
  doc["mv_raw"]              = serialized(String(fOut.raw, 2));
  doc["mv_ma"]               = serialized(String(fOut.ma, 2));
  doc["mv_ema"]              = serialized(String(fOut.ema, 2));
  doc["mv_median"]           = serialized(String(fOut.median, 2));
  doc["mv_kalman"]           = serialized(String(fOut.kalman, 2));
  doc["mv_hybrid"]           = serialized(String(fOut.hybrid, 2));
  doc["sensor_mv"]           = serialized(String(fOut.hybrid, 2));

  // Calibrated Unitless UVI Outputs (Task 1)
  doc["uv_index_calibrated"] = serialized(String(uviCalibrated, 2));
  doc["uv_index_estimated"]  = serialized(String(uviCalibrated, 2));
  doc["uv_index_lookup_int"] = uviLookupInt;
  doc["uv_index_linear_old"] = serialized(String(uviLinearOld, 2));
  doc["risk_category"]       = riskCategory;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleWiFiScan() {
  addCorsHeaders();
  int n = WiFi.scanNetworks();
  JsonDocument doc;
  JsonArray arr = doc["networks"].to<JsonArray>();
  for (int i = 0; i < n; i++) {
    JsonObject net = arr.add<JsonObject>();
    net["ssid"] = WiFi.SSID(i);
    net["rssi"] = WiFi.RSSI(i);
    net["open"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
  }
  String res;
  serializeJson(doc, res);
  server.send(200, "application/json", res);
}

void handleSetWiFi() {
  addCorsHeaders();
  String newSsid = server.arg("ssid");
  String newPass = server.arg("pass");
  String inUrl   = server.arg("influx_url");
  String inTok   = server.arg("influx_token");

  prefs.begin("uv_cfg", false);
  if (newSsid.length() > 0) {
    prefs.putString("ssid", newSsid);
    prefs.putString("pass", newPass);
  }
  if (inUrl.length() > 5) {
    prefs.putString("in_url", inUrl);
    prefs.putString("in_tok", inTok);
  }
  prefs.end();

  server.send(200, "application/json", "{\"status\":\"saved\",\"restarting\":true}");
  delay(800);
  ESP.restart();
}

void handleCaptivePortal() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<style>body{background:#0B0F19;color:#fff;font-family:sans-serif;padding:16px;}";
  html += ".card{background:#151F32;padding:20px;border-radius:14px;max-width:420px;margin:auto;}";
  html += "input,button{width:100%;padding:12px;margin:8px 0;border-radius:8px;border:none;box-sizing:border-box;}";
  html += "button{background:#00F5D4;color:#000;font-weight:bold;cursor:pointer;}</style></head><body>";
  html += "<div class='card'><h2>ESP32 UV Sensor Wi-Fi & Cloud Setup</h2>";
  html += "<form action='/wifi' method='POST'>";
  html += "<label>Wi-Fi Network Name (SSID):</label><input type='text' name='ssid' placeholder='Enter Wi-Fi or Hotspot Name' required>";
  html += "<label>Wi-Fi Password:</label><input type='password' name='pass' placeholder='Enter Password'>";
  html += "<label>InfluxDB Write URL (Optional for Grafana):</label><input type='text' name='influx_url' value='" + influxUrl + "'>";
  html += "<label>InfluxDB Token (Optional):</label><input type='password' name='influx_token' value='" + influxToken + "'>";
  html += "<button type='submit'>Save & Connect</button></form></div></body></html>";
  server.send(200, "text/html", html);
}

void connectWiFiMultiFallback() {
  prefs.begin("uv_cfg", true);
  String savedSsid = prefs.getString("ssid", DEFAULT_WIFI_SSID);
  String savedPass = prefs.getString("pass", DEFAULT_WIFI_PASS);
  influxUrl        = prefs.getString("in_url", influxUrl);
  influxToken      = prefs.getString("in_tok", influxToken);
  prefs.end();

  WiFi.mode(WIFI_AP_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(savedSsid.c_str(), savedPass.c_str());

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connecting Wi-Fi:");
  display.println(savedSsid);
  display.display();

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 10000) {
    delay(400);
  }

  if (WiFi.status() != WL_CONNECTED && savedSsid != DEFAULT_WIFI_SSID) {
    WiFi.begin(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASS);
    startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < 8000) {
      delay(400);
    }
  }

  // Broadcast Setup AP so you can always connect and change Wi-Fi from your phone
  WiFi.softAP(CAPTIVE_AP_SSID);
  if (WiFi.status() != WL_CONNECTED) {
    dnsServer.start(53, "*", WiFi.softAPIP());
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  connectWiFiMultiFallback();

  server.on("/", HTTP_GET, handleCaptivePortal);
  server.on("/data", HTTP_GET, handleJsonData);
  server.on("/scan", HTTP_GET, handleWiFiScan);
  server.on("/wifi", HTTP_POST, handleSetWiFi);
  server.onNotFound(handleCaptivePortal);
  server.begin();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      String clientId = "ESP32C3_UV_" + String(random(0xffff), HEX);
      mqttClient.connect(clientId.c_str());
    }
    mqttClient.loop();
  }

  unsigned long now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    packetSeq++;
    currentRssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;

    sampleAndCompute6Filters();

    // 1. Publish to HiveMQ MQTT Broker
    if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
      JsonDocument doc;
      doc["seq"]                 = packetSeq;
      doc["ip"]                  = WiFi.localIP().toString();
      doc["ssid"]                = WiFi.SSID();
      doc["rssi_dbm"]            = currentRssi;
      doc["mv_raw"]              = serialized(String(fOut.raw, 2));
      doc["mv_ma"]               = serialized(String(fOut.ma, 2));
      doc["mv_ema"]              = serialized(String(fOut.ema, 2));
      doc["mv_median"]           = serialized(String(fOut.median, 2));
      doc["mv_kalman"]           = serialized(String(fOut.kalman, 2));
      doc["mv_hybrid"]           = serialized(String(fOut.hybrid, 2));
      doc["uv_index_calibrated"] = serialized(String(uviCalibrated, 2));
      doc["uv_index_lookup_int"] = uviLookupInt;
      doc["uv_index_linear_old"] = serialized(String(uviLinearOld, 2));
      doc["risk_category"]       = riskCategory;

      char buffer[450];
      serializeJson(doc, buffer);
      mqttClient.publish(MQTT_TOPIC, buffer);
    }

    // 2. Push to InfluxDB Cloud every 5 seconds (for simultaneous Grafana Dashboard)
    if (WiFi.status() == WL_CONNECTED && influxUrl.length() > 10 && (packetSeq % 5 == 0)) {
      WiFiClientSecure client;
      client.setInsecure();
      HTTPClient https;
      if (https.begin(client, influxUrl)) {
        https.addHeader("Authorization", "Token " + influxToken);
        https.addHeader("Content-Type", "text/plain; charset=utf-8");
        String line = "uv_sensor,device=esp32c3 "
                      "mv_raw=" + String(fOut.raw, 2) + ","
                      "mv_ma=" + String(fOut.ma, 2) + ","
                      "mv_ema=" + String(fOut.ema, 2) + ","
                      "mv_median=" + String(fOut.median, 2) + ","
                      "mv_kalman=" + String(fOut.kalman, 2) + ","
                      "mv_hybrid=" + String(fOut.hybrid, 2) + ","
                      "uv_index_calibrated=" + String(uviCalibrated, 2) + ","
                      "uv_index_lookup_int=" + String(uviLookupInt) + "i,"
                      "uv_index_linear_old=" + String(uviLinearOld, 2) + ","
                      "rssi_dbm=" + String(currentRssi) + "i";
        https.POST(line);
        https.end();
      }
    }

    // 3. Update OLED (Unitless UVI + Risk Category)
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    if (WiFi.status() == WL_CONNECTED) {
      display.printf("IP:%s\n", WiFi.localIP().toString().c_str());
    } else {
      display.printf("Setup:192.168.4.1\n");
    }
    display.println("--------------------");
    display.printf("Vout: %.1fmV (Raw:%.0f)\n", fOut.hybrid, fOut.raw);
    display.printf("UVI : %.2f (Tbl:%d)\n", uviCalibrated, uviLookupInt);
    display.printf("Risk: %s\n", riskCategory.c_str());
    display.printf("MQTT:%s | %ddBm\n", mqttClient.connected() ? "ON" : "--", currentRssi);
    display.display();
  }
}