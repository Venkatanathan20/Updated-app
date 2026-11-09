#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- HARDWARE PIN DEFINITIONS ----------------
#define UV_ADC_PIN       0   // GPIO0 (ADC1_CH0) for GUVA-S12SD
#define OLED_SDA_PIN     8   // ESP32-C3 Default I2C SDA
#define OLED_SCL_PIN     9   // ESP32-C3 Default I2C SCL
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledAvailable = false;

// ---------------- NETWORK & CLOUD CONFIG ----------------
const char* AP_SSID       = "ESP32_UV_WiFi_Setup";
const char* MQTT_BROKER   = "broker.hivemq.com";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_TOPIC    = "venkat_iot/sensors/uv/node_01";

Preferences prefs;
WebServer server(80);
DNSServer dnsServer;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

String savedSsid       = "";
String savedPass       = "";
String savedInfluxUrl  = "";
String savedInfluxTok  = "";

// ---------------- 3-FILTER DSP STATE (MEDIAN, KALMAN, HYBRID) ----------------
#define MEDIAN_WIN_SIZE 9
float winBuf[MEDIAN_WIN_SIZE];
int winCount = 0;
int winIdx   = 0;

bool filterInitialized = false;
float kalX = 0.0f, kalP = 1.0f;
float hybX = 0.0f, hybP = 1.0f;

// Latest Telemetry Values
float latestMvRaw      = 0.0f;
float latestMvMedian   = 0.0f;
float latestMvKalman   = 0.0f;
float latestMvHybrid   = 0.0f;
float latestUviCal     = 0.0f;
float latestUviLin     = 0.0f;
int   latestUviInt     = 0;
String latestRisk      = "Low";

unsigned long lastSampleMs = 0;
const unsigned long SAMPLE_INTERVAL_MS = 2000;

// ---------------- GUVA-S12SD PIECEWISE CALIBRATION ----------------
const float MV_THRESHOLDS[12] = {
  50.0f, 227.0f, 318.0f, 408.0f, 503.0f, 606.0f,
  696.0f, 795.0f, 881.0f, 976.0f, 1079.0f, 1170.0f
};

int lookupDiscreteUVI(float mv) {
  if (mv < 50.0f) return 0;
  for (int uvi = 11; uvi >= 1; uvi--) {
    if (mv >= MV_THRESHOLDS[uvi]) return uvi;
  }
  return 0;
}

float calculateCalibratedUVI(float mv) {
  if (mv < 50.0f) return 0.0f;
  if (mv >= 1170.0f) return 11.0f + (mv - 1170.0f) / 91.0f;
  for (int i = 0; i < 11; i++) {
    float vLow = MV_THRESHOLDS[i];
    float vHigh = MV_THRESHOLDS[i + 1];
    if (mv >= vLow && mv <= vHigh) {
      return i + (mv - vLow) / (vHigh - vLow);
    }
  }
  return 0.0f;
}

String getRiskCategory(float uvi) {
  if (uvi < 3.0f)  return "Low";
  if (uvi < 6.0f)  return "Moderate";
  if (uvi < 8.0f)  return "High";
  if (uvi < 11.0f) return "Very high";
  return "Extreme";
}

// ---------------- ANALOG READ & 3-FILTER ENGINE ----------------
float readSensorMillivolts() {
  uint32_t sumMv = 0;
  const int oversample = 16;
  for (int i = 0; i < oversample; i++) {
    sumMv += analogReadMilliVolts(UV_ADC_PIN);
    delayMicroseconds(250);
  }
  return (float)sumMv / (float)oversample;
}

void update3Filters() {
  float raw = readSensorMillivolts();
  latestMvRaw = raw;

  // 1. Sliding Window for Median Filter
  winBuf[winIdx] = raw;
  winIdx = (winIdx + 1) % MEDIAN_WIN_SIZE;
  if (winCount < MEDIAN_WIN_SIZE) winCount++;

  float sorted[MEDIAN_WIN_SIZE];
  for (int i = 0; i < winCount; i++) sorted[i] = winBuf[i];
  for (int i = 0; i < winCount - 1; i++) {
    for (int j = i + 1; j < winCount; j++) {
      if (sorted[j] < sorted[i]) {
        float tmp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = tmp;
      }
    }
  }
  latestMvMedian = sorted[winCount / 2];

  // 2. 1D Kalman Filter & 3. Hybrid (Median + Kalman) Filter
  if (!filterInitialized) {
    kalX = raw;
    hybX = latestMvMedian;
    filterInitialized = true;
  } else {
    // 1D Kalman on Raw
    kalP += 0.05f;
    float kG = kalP / (kalP + 2.5f);
    kalX += kG * (raw - kalX);
    kalP = (1.0f - kG) * kalP;

    // Hybrid: 1D Kalman on Median-filtered output
    hybP += 0.05f;
    float hG = hybP / (hybP + 1.5f);
    hybX += hG * (latestMvMedian - hybX);
    hybP = (1.0f - hG) * hybP;
  }

  latestMvKalman = kalX;
  latestMvHybrid = hybX;

  // Calibrate UV Index using Hybrid filter output
  latestUviCal = calculateCalibratedUVI(latestMvHybrid);
  latestUviLin = latestMvHybrid / 100.0f;
  latestUviInt = lookupDiscreteUVI(latestMvHybrid);
  latestRisk   = getRiskCategory(latestUviCal);
}

// ---------------- PAYLOAD BUILDER ----------------
String buildTelemetryJson() {
  JsonDocument doc;
  doc["mv_raw"]       = serialized(String(latestMvRaw, 2));
  doc["mv_median"]    = serialized(String(latestMvMedian, 2));
  doc["mv_kalman"]    = serialized(String(latestMvKalman, 2));
  doc["mv_hybrid"]    = serialized(String(latestMvHybrid, 2));
  doc["uvi_cal"]      = serialized(String(latestUviCal, 3));
  doc["uvi_lin"]      = serialized(String(latestUviLin, 3));
  doc["uvi_int"]      = latestUviInt;
  doc["risk"]         = latestRisk;
  doc["rssi_dbm"]     = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  doc["ip"]           = (WiFi.status() == WL_CONNECTED)
                          ? WiFi.localIP().toString()
                          : WiFi.softAPIP().toString();

  String output;
  serializeJson(doc, output);
  return output;
}

// ---------------- CLOUD STREAMING (MQTT + INFLUXDB) ----------------
void publishToHiveMq(const String& jsonPayload) {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!mqttClient.connected()) {
    String clientId = "esp32c3_uv_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    mqttClient.connect(clientId.c_str());
  }
  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_TOPIC, jsonPayload.c_str());
  }
}

void pushToInfluxDb() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (savedInfluxUrl.length() < 8 || savedInfluxTok.length() < 4) return;

  NetworkClientSecure client;
  client.setInsecure();
  HTTPClient https;

  if (https.begin(client, savedInfluxUrl)) {
    https.addHeader("Authorization", "Token " + savedInfluxTok);
    https.addHeader("Content-Type", "text/plain; charset=utf-8");

    String line = "uv_telemetry,node=esp32c3_01 ";
    line += "mv_raw=" + String(latestMvRaw, 2) + ",";
    line += "mv_median=" + String(latestMvMedian, 2) + ",";
    line += "mv_kalman=" + String(latestMvKalman, 2) + ",";
    line += "mv_hybrid=" + String(latestMvHybrid, 2) + ",";
    line += "uvi_cal=" + String(latestUviCal, 3) + ",";
    line += "uvi_int=" + String(latestUviInt) + "i";

    https.POST(line);
    https.end();
  }
}

// ---------------- OLED DISPLAY UPDATE ----------------
void updateOledDisplay() {
  if (!oledAvailable) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    display.print("IP:");
    display.println(WiFi.localIP());
  } else {
    display.println("AP: 192.168.4.1");
  }

  display.setCursor(0, 12);
  display.printf("Raw: %.1f | Med: %.1f\n", latestMvRaw, latestMvMedian);
  display.printf("Kal: %.1f | Hyb: %.1f\n", latestMvKalman, latestMvHybrid);

  display.setTextSize(2);
  display.setCursor(0, 34);
  display.printf("UVI: %.2f\n", latestUviCal);

  display.setTextSize(1);
  display.setCursor(0, 54);
  display.printf("Risk: %s (Step %d)", latestRisk.c_str(), latestUviInt);
  display.display();
}

// ---------------- HTTP ENDPOINTS & CORS ----------------
void sendCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
}

void setupWebServer() {
  server.on("/data", HTTP_GET, []() {
    sendCorsHeaders();
    server.send(200, "application/json", buildTelemetryJson());
  });

  server.on("/scan", HTTP_GET, []() {
    sendCorsHeaders();
    int n = WiFi.scanNetworks();
    JsonDocument doc;
    JsonArray arr = doc["networks"].to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject net = arr.add<JsonObject>();
      net["ssid"] = WiFi.SSID(i);
      net["rssi"] = WiFi.RSSI(i);
    }
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.on("/wifi", HTTP_POST, []() {
    sendCorsHeaders();
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    String iUrl = server.arg("influx_url");
    String iTok = server.arg("influx_token");

    if (ssid.length() > 0) {
      prefs.begin("uv_cfg", false);
      prefs.putString("ssid", ssid);
      prefs.putString("pass", pass);
      if (iUrl.length() > 0) prefs.putString("iurl", iUrl);
      if (iTok.length() > 0) prefs.putString("itok", iTok);
      prefs.end();

      savedSsid = ssid;
      savedPass = pass;
      if (iUrl.length() > 0) savedInfluxUrl = iUrl;
      if (iTok.length() > 0) savedInfluxTok = iTok;

      server.send(200, "application/json", "{\"status\":\"reconnecting\"}");
      delay(400);
      WiFi.disconnect();
      WiFi.begin(savedSsid.c_str(), savedPass.c_str());
    } else {
      server.send(400, "application/json", "{\"error\":\"missing_ssid\"}");
    }
  });

  server.onNotFound([]() {
    sendCorsHeaders();
    server.send(200, "application/json", buildTelemetryJson());
  });

  server.begin();
}

// ---------------- SETUP & MAIN LOOP ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(UV_ADC_PIN, ADC_11db);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledAvailable = true;
    display.clearDisplay();
    display.display();
  }

  prefs.begin("uv_cfg", true);
  savedSsid      = prefs.getString("ssid", "");
  savedPass      = prefs.getString("pass", "");
  savedInfluxUrl = prefs.getString("iurl", "");
  savedInfluxTok = prefs.getString("itok", "");
  prefs.end();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID);
  dnsServer.start(53, "*", WiFi.softAPIP());

  if (savedSsid.length() > 0) {
    WiFi.begin(savedSsid.c_str(), savedPass.c_str());
  }

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setBufferSize(512);
  setupWebServer();
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  if (WiFi.status() == WL_CONNECTED) {
    mqttClient.loop();
  }

  unsigned long now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    update3Filters();
    updateOledDisplay();

    String payload = buildTelemetryJson();
    Serial.println(payload);
    publishToHiveMq(payload);
    pushToInfluxDb();
  }
}
