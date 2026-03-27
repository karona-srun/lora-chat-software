// -------------------------------------------------------------------------------
// LoRa Node with Dynamic Configuration - E22-900T33S / ESP32
// Configure Node ID, Network ID, and Repeater settings via WebUI
// -------------------------------------------------------------------------------

#include <Arduino.h>
#include <LoRa_E22.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>

// ----------------------------- Dynamic Configuration -------------------------
Preferences preferences;

uint8_t MY_ADDH = 0x00;
uint8_t MY_ADDL = 0x03;     // Default Node ID
uint8_t TARGET_ADDH = 0x00;
uint8_t TARGET_ADDL = 0x00; // Default Target
uint8_t REPEATER_ADDH = 0xFF;
uint8_t REPEATER_ADDL = 0xFF;
uint8_t NETWORK_ID = 0x01;  // Default Network ID
uint8_t CHANNEL = 0x41;     // Default Channel 915Mhz
uint16_t CRYPT = 0x8002;    // KEY 32770
bool USE_REPEATER = false;  // Repeater mode

// ----------------------------- WiFi credentials -------------------------------
String ssid_AP = "LM-" + String(MY_ADDL);
String password_AP = "12345678";
String callSign = ssid_AP;

String defaultApSsid() {
    return "LM-" + String(MY_ADDL);
}

#define BAT_READER    36  // ADC pin for battery voltage
#define BAT_CHARGING  34  // HIGH when charging (input only)

// Battery ADC: 0–4095 @ 3.3V; adjust divider so 4.2V battery → ~3.3V at pin
#define BAT_ADC_MAX   4095.0
// Battery range: 3.3V (low) .. 4.2V (full)
#define BAT_V_MIN     3.60   // battery "low" voltage
#define BAT_V_MAX     4.20   // battery "full" voltage
#define BAT_DIVIDER   (4.20 / 3.60)  // scale ADC to battery voltage if needed

#define BTN_NEXT_PIN  32  // next screen button, pressed = LOW
#define BTN_PREV_PIN  35  // previous screen button, pressed = LOW (input-only pin)

// ────────────────────────────────────────────────
// GPS on Serial1
// ────────────────────────────────────────────────
#define GPS_TX_PIN  25
#define GPS_RX_PIN  26
#define GPS_BAUD    9600

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ----------------------------- OLED ------------------------------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS 0x3C
#define OLED_SDA        27
#define OLED_SCL        33

unsigned long startTime = 0;
int onlineNodes = 0;
float batteryVoltage   = 3.79f;
int batteryPercent     = 100;
bool batteryCharging   = false;
bool lastBatteryCharging = false;

int currentScreen = 0;
unsigned long lastNextBtnPress = 0;
unsigned long lastPrevBtnPress = 0;
bool nextBtnHeld = false;
bool nextBtnLongSent = false;
unsigned long nextBtnDownMs = 0;
bool chargeScreenSuppressed = false;

enum class PopupType : uint8_t { None, Sent, Received };
PopupType popupType = PopupType::None;
String popupMsg = "";
String popupStatus = "";
unsigned long popupUntilMs = 0;

const bool AUTO_GPS_ENABLED = false;
const unsigned long AUTO_GPS_INTERVAL_MS = 5000UL;
const unsigned long BTN_NEXT_LONG_PRESS_MS = 3000UL;
const unsigned long CHARGE_PIN_DEBOUNCE_MS = 400UL;
const unsigned long UI_REFRESH_NORMAL_MS = 1000UL;
const unsigned long UI_REFRESH_CHARGING_MS = 200UL;

// ----------------------------- E22 pins --------------------------------------
#define PIN_E22_TX   4
#define PIN_E22_RX  22
#define PIN_AUX     18
#define PIN_M0     21
#define PIN_M1     19

LoRa_E22 e22(PIN_E22_TX, PIN_E22_RX, &Serial2, PIN_AUX, PIN_M0, PIN_M1, UART_BPS_RATE_9600, SERIAL_8N1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

bool nodeReady = false;
String statusMsg = "Booting";
uint32_t sentCount = 0;
uint32_t receivedCount = 0;
String lastSent = "";
String lastReceived = "";
int16_t lastRssiSignal = -255;
uint16_t txMessageCounter = 0;
String awaitingAckId = "";
bool awaitingAckReceived = false;

const unsigned long ACK_WAIT_MS = 1200UL;
const uint8_t ACK_RETRY_MAX = 2;

// Nearby nodes registry (learned from HELLO beacons)
#define MAX_NEARBY_NODES 20
struct NearbyNode {
    uint16_t addr;
    uint8_t netId;
    uint8_t channel;
    int16_t rssi;
    unsigned long lastSeen;
    bool isRepeater;
    String callSign;
};
NearbyNode nearbyNodes[MAX_NEARBY_NODES];
uint8_t nearbyCount = 0;
unsigned long lastBeaconAt = 0;

// Same rule as /api/nodes "online" field: seen a HELLO within this window (beacons ~5s).
#define ONLINE_NODE_TTL_MS 10000UL

static bool isNearbyNodeOnlineAt(uint8_t i, unsigned long now) {
    if (i >= nearbyCount) return false;
    return (now - nearbyNodes[i].lastSeen) <= ONLINE_NODE_TTL_MS;
}

static uint8_t countOnlineNearbyNodes(unsigned long now) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < nearbyCount; i++) {
        if (isNearbyNodeOnlineAt(i, now)) n++;
    }
    return n;
}

// Keep OLED "online" count in sync with /api/nodes filtering logic
void syncOnlineNodesFromRegistry() {
    onlineNodes = (int)countOnlineNearbyNodes(millis());
}

// Message log
#define MAX_LOG_ENTRIES 20
struct LogEntry {
    unsigned long timestamp;
    String direction;
    int8_t rssi;
    String address;
    String message;
};
LogEntry messageLog[MAX_LOG_ENTRIES];
int logIndex = 0;

void addLogEntry(String direction, int8_t rssi, String addr, String msg) {
    messageLog[logIndex].timestamp = millis();
    messageLog[logIndex].direction = direction;
    messageLog[logIndex].rssi = rssi;
    messageLog[logIndex].address = addr;
    messageLog[logIndex].message = msg;
    logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
}

// =============================================================================
// Preferences - Load/Save Configuration
// =============================================================================

void loadConfig() {
    preferences.begin("lora-config", false);
    
    MY_ADDL = preferences.getUChar("my_addl", MY_ADDL);
    MY_ADDH = preferences.getUChar("my_addh", MY_ADDH);
    TARGET_ADDL = preferences.getUChar("target_addl", TARGET_ADDL);
    TARGET_ADDH = preferences.getUChar("target_addh", TARGET_ADDH);
    REPEATER_ADDL = preferences.getUChar("rep_addl", REPEATER_ADDL);
    REPEATER_ADDH = preferences.getUChar("rep_addh", REPEATER_ADDH);
    NETWORK_ID = preferences.getUChar("netid", NETWORK_ID);
    CHANNEL = preferences.getUChar("channel", CHANNEL);
    USE_REPEATER = preferences.getBool("use_repeater", false);
    if (preferences.isKey("crypt")) {
        CRYPT = preferences.getUShort("crypt", CRYPT);
    } else if (preferences.isKey("crypt_h") || preferences.isKey("crypt_l")) {
        uint8_t h = (uint8_t)preferences.getUShort("crypt_h", highByte(CRYPT));
        uint8_t l = (uint8_t)preferences.getUShort("crypt_l", lowByte(CRYPT));
        CRYPT = ((uint16_t)h << 8) | l;
    }
    ssid_AP = preferences.getString("ssid_ap", defaultApSsid());
    if (ssid_AP.length() == 0) ssid_AP = defaultApSsid();
    
    preferences.end();
    
    Serial.println("\n=== Loaded Configuration ===");
    Serial.printf("My Address: 0x%02X%02X\n", MY_ADDH, MY_ADDL);
    Serial.printf("Target: 0x%02X%02X\n", TARGET_ADDH, TARGET_ADDL);
    Serial.printf("Repeater: 0x%02X%02X\n", REPEATER_ADDH, REPEATER_ADDL);
    Serial.printf("Network ID: 0x%02X\n", NETWORK_ID);
    Serial.printf("Channel: 0x%02X\n", CHANNEL);
    Serial.printf("Use Repeater: %s\n", USE_REPEATER ? "Yes" : "No");
    Serial.printf("CRYPT: 0x%04X (must match other nodes; 0x0000 = off)\n", CRYPT);
    Serial.println("============================\n");
}

void saveConfig() {
    preferences.begin("lora-config", false);
    
    preferences.putUChar("my_addl", MY_ADDL);
    preferences.putUChar("my_addh", MY_ADDH);
    preferences.putUChar("target_addl", TARGET_ADDL);
    preferences.putUChar("target_addh", TARGET_ADDH);
    preferences.putUChar("rep_addl", REPEATER_ADDL);
    preferences.putUChar("rep_addh", REPEATER_ADDH);
    preferences.putUChar("netid", NETWORK_ID);
    preferences.putUChar("channel", CHANNEL);
    preferences.putBool("use_repeater", USE_REPEATER);
    preferences.putUShort("crypt", CRYPT);
    
    preferences.end();
    
    Serial.println("Configuration saved!");
}

void saveApConfig() {
    preferences.begin("lora-config", false);
    preferences.putString("ssid_ap", ssid_AP);
    preferences.end();
    Serial.printf("AP SSID saved to flash: %s\n", ssid_AP.c_str());
}

// =============================================================================
// Helpers
// =============================================================================

String toHex2(uint8_t v) {
    char b[3]; 
    snprintf(b, sizeof(b), "%02X", v); 
    return String(b);
}

String toHex4(uint16_t addr) {
    char b[5];
    snprintf(b, sizeof(b), "%04X", addr);
    return String(b);
}

int16_t parseHex16(const String &hex4) {
    if (hex4.length() != 4) return -1;
    char *end = nullptr;
    long v = strtol(hex4.c_str(), &end, 16);
    if (end == hex4.c_str() || *end != '\0') return -1;
    if (v < 0 || v > 0xFFFF) return -1;
    return (int16_t)v;
}

int16_t parseHex8(const String &hex2) {
    if (hex2.length() != 2) return -1;
    char *end = nullptr;
    long v = strtol(hex2.c_str(), &end, 16);
    if (end == hex2.c_str() || *end != '\0') return -1;
    if (v < 0 || v > 0xFF) return -1;
    return (int16_t)v;
}

String getUptime() {
    unsigned long seconds = (millis() - startTime) / 1000;
    unsigned long hours = seconds / 3600;
    unsigned long minutes = (seconds % 3600) / 60;
    char buf[16];
    snprintf(buf, sizeof(buf), "%luh %lum", hours, minutes);
    return String(buf);
}

String jsonEscape(const String &s) {
    String out = "";
    out.reserve(s.length() + 8);
    for (size_t i = 0; i < s.length(); i++) {
        char c = s.charAt(i);
        if (c == '\\' || c == '"') {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c == '\t') {
            out += "\\t";
        } else {
            out += c;
        }
    }
    return out;
}

String htmlEscape(const String &s) {
    String out;
    out.reserve(s.length() + 16);
    for (size_t i = 0; i < s.length(); i++) {
        char c = s.charAt(i);
        if (c == '&') {
            out += F("&amp;");
        } else if (c == '<') {
            out += F("&lt;");
        } else if (c == '>') {
            out += F("&gt;");
        } else if (c == '"') {
            out += F("&quot;");
        } else {
            out += c;
        }
    }
    return out;
}

static void appendStatusRow(String &html, const char *label, const String &value) {
    html += F("<tr><th>");
    html += label;
    html += F("</th><td>");
    html += htmlEscape(value);
    html += F("</td></tr>");
}

static String defaultCallSignForAddr(uint16_t addr) {
    return String("LM-") + String((uint8_t)(addr & 0xFF));
}

void upsertNearbyNode(uint16_t addr, uint8_t netId, uint8_t channel, int16_t rssi, bool isRepeater,
                      const String &callSignIn) {
    if (addr == 0x0000) return;
    unsigned long now = millis();
    for (uint8_t i = 0; i < nearbyCount; i++) {
        if (nearbyNodes[i].addr == addr) {
            nearbyNodes[i].netId = netId;
            nearbyNodes[i].channel = channel;
            nearbyNodes[i].rssi = rssi;
            nearbyNodes[i].lastSeen = now;
            nearbyNodes[i].isRepeater = isRepeater;
            if (callSignIn.length() > 0) nearbyNodes[i].callSign = callSignIn;
            return;
        }
    }
    if (nearbyCount < MAX_NEARBY_NODES) {
        nearbyNodes[nearbyCount].addr = addr;
        nearbyNodes[nearbyCount].netId = netId;
        nearbyNodes[nearbyCount].channel = channel;
        nearbyNodes[nearbyCount].rssi = rssi;
        nearbyNodes[nearbyCount].lastSeen = now;
        nearbyNodes[nearbyCount].isRepeater = isRepeater;
        nearbyNodes[nearbyCount].callSign = callSignIn.length() > 0 ? callSignIn : defaultCallSignForAddr(addr);
        nearbyCount++;
        return;
    }

    // Replace the oldest entry when full
    uint8_t oldestIdx = 0;
    for (uint8_t i = 1; i < nearbyCount; i++) {
        if (nearbyNodes[i].lastSeen < nearbyNodes[oldestIdx].lastSeen) oldestIdx = i;
    }
    nearbyNodes[oldestIdx].addr = addr;
    nearbyNodes[oldestIdx].netId = netId;
    nearbyNodes[oldestIdx].channel = channel;
    nearbyNodes[oldestIdx].rssi = rssi;
    nearbyNodes[oldestIdx].lastSeen = now;
    nearbyNodes[oldestIdx].isRepeater = isRepeater;
    nearbyNodes[oldestIdx].callSign =
        callSignIn.length() > 0 ? callSignIn : defaultCallSignForAddr(addr);
}

// =============================================================================
// OLED Display
// =============================================================================

void drawDisplay() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    
    // Node info
    display.setCursor(0, 0);  
    display.print("Node:");
    display.print(toHex2(MY_ADDH));
    display.print(toHex2(MY_ADDL));
    display.print(" N:");
    display.print(toHex2(NETWORK_ID));
    
    // IP address (last octet only)
    display.setCursor(0, 12); 
    display.print("IP: ");
    display.print(WiFi.softAPIP().toString());
    
    // Stats
    display.setCursor(0, 22);
    display.print("RX:"); 
    display.print(receivedCount);
    display.print(" TX:"); 
    display.print(sentCount);
    
    // Last received
    display.setCursor(0, 32);
    if (lastReceived.length() > 0) {
        display.print("RX:");
        display.print(lastReceived.substring(0, 13));
    }
    
    // Last sent
    display.setCursor(0, 42);
    if (lastSent.length() > 0) {
        display.print("TX:");
        display.print(lastSent.substring(0, 13));
    }
    
    // Status
    display.setCursor(0, 52);
    display.print("Up:");
    display.print(getUptime());
    
    display.display();
}

// =============================================================================
// Wait for AUX
// =============================================================================

bool waitAuxHigh(uint32_t timeoutMs = 5000) {
    uint32_t t = millis();
    while (digitalRead(PIN_AUX) == LOW) {
        if (millis() - t > timeoutMs) return false;
        delay(10);
    }
    return true;
}

// =============================================================================
// Apply Configuration to E22
// =============================================================================

bool applyNodeConfig() {
    if (!waitAuxHigh(8000)) return false;
    delay(100);

    ResponseStructContainer c = e22.getConfiguration();
    if (c.status.code != 1) { 
        c.close(); 
        return false; 
    }

    Configuration config = *(Configuration*)c.data;
    c.close();

    // Set addresses and network
    config.ADDH  = MY_ADDH;
    config.ADDL  = MY_ADDL;
    config.NETID = NETWORK_ID;
    config.CHAN  = CHANNEL;

    // E22 hardware AES key (high byte first per module register map)
    config.CRYPT.CRYPT_H = highByte(CRYPT);
    config.CRYPT.CRYPT_L = lowByte(CRYPT);

    // Common settings
    config.SPED.uartBaudRate = UART_BPS_9600;
    config.SPED.airDataRate  = AIR_DATA_RATE_010_24;
    config.SPED.uartParity   = MODE_00_8N1;

    config.OPTION.subPacketSetting  = SPS_240_00;
    config.OPTION.RSSIAmbientNoise  = RSSI_AMBIENT_NOISE_DISABLED;
    config.OPTION.transmissionPower = POWER_22; // MAX POWER

    config.TRANSMISSION_MODE.enableRSSI            = RSSI_ENABLED;
    config.TRANSMISSION_MODE.fixedTransmission     = FT_FIXED_TRANSMISSION;
    config.TRANSMISSION_MODE.enableRepeater        = USE_REPEATER ? REPEATER_ENABLED : REPEATER_DISABLED;
    config.TRANSMISSION_MODE.enableLBT             = LBT_DISABLED;
    config.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
    config.TRANSMISSION_MODE.WORPeriod             = WOR_2000_011;

    ResponseStatus rs = e22.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
    if (rs.code != 1) return false;

    delay(300);
    if (!waitAuxHigh(5000)) return false;
    delay(100);

    Serial.println("[CONFIG] Applied to E22 module");
    Serial.printf("[KEY] CRYPT=0x%04X (H:%s L:%s)\n", CRYPT,
                  toHex2(highByte(CRYPT)).c_str(), toHex2(lowByte(CRYPT)).c_str());
    return true;
}

// =============================================================================
// Send Message
// =============================================================================

static bool waitForAck(const String& msgId, unsigned long timeoutMs) {
    awaitingAckId = msgId;
    awaitingAckReceived = false;

    unsigned long started = millis();
    while ((millis() - started) < timeoutMs) {
        checkIncoming();
        if (awaitingAckReceived) {
            awaitingAckId = "";
            return true;
        }
        delay(10);
    }

    awaitingAckId = "";
    return false;
}

bool sendMessage(String msg, bool viaRepeater = false, uint8_t destAddh = 0xFF, uint8_t destAddl = 0xFF, bool overrideDest = false) {
    uint8_t finalAddh = overrideDest ? destAddh : TARGET_ADDH;
    uint8_t finalAddl = overrideDest ? destAddl : TARGET_ADDL;
    String myAddrHex = toHex4((MY_ADDH << 8) | MY_ADDL);
    String msgId = toHex4(++txMessageCounter);
    String payload = "MSG|" + msgId + "|" + myAddrHex + "|" + msg;

    bool acked = false;
    for (uint8_t attempt = 0; attempt <= ACK_RETRY_MAX; attempt++) {
        if (viaRepeater) {
            // Send via repeater with relay envelope.
            String destHex = toHex4((finalAddh << 8) | finalAddl);
            String relayMsg = "RELAY|" + destHex + "|" + payload;
            ResponseStatus rs = e22.sendFixedMessage(REPEATER_ADDH, REPEATER_ADDL, CHANNEL, relayMsg);
            Serial.printf("[TX] Via repeater to 0x%02X%02X (id=%s try=%u): %s\n",
                finalAddh, finalAddl, msgId.c_str(), (unsigned)(attempt + 1), rs.getResponseDescription().c_str());
            if (rs.code == 1 && waitForAck(msgId, ACK_WAIT_MS)) {
                acked = true;
                break;
            }
        } else {
            ResponseStatus rs = e22.sendFixedMessage(finalAddh, finalAddl, CHANNEL, payload);
            Serial.printf("[TX] Direct to 0x%02X%02X (id=%s try=%u): %s\n",
                finalAddh, finalAddl, msgId.c_str(), (unsigned)(attempt + 1), rs.getResponseDescription().c_str());
            if (rs.code == 1 && waitForAck(msgId, ACK_WAIT_MS)) {
                acked = true;
                break;
            }
        }
    }

    sentCount++;
    lastSent = msg;
    statusMsg = acked ? "SENT ACK" : "SENT NO ACK";
    if (viaRepeater) {
        addLogEntry(acked ? "TX-RELAY-ACK" : "TX-RELAY-NOACK", 0, toHex4((REPEATER_ADDH << 8) | REPEATER_ADDL), msg);
    } else {
        addLogEntry(acked ? "TX-ACK" : "TX-NOACK", 0, toHex4((finalAddh << 8) | finalAddl), msg);
    }
    return acked;
}

// =============================================================================
// Receive Messages
// =============================================================================

void checkIncoming() {
    if (e22.available() <= 0) return;

    ResponseContainer rc = e22.receiveMessageRSSI();
    if (rc.status.code != 1) return;

    String payload = rc.data;

    // ACK frame format: ACK|<msgId>
    if (payload.startsWith("ACK|")) {
        String ackId = payload.substring(4);
        int sep = ackId.indexOf('|');
        if (sep > 0) ackId = ackId.substring(0, sep);
        ackId.trim();

        if (awaitingAckId.length() > 0 && ackId == awaitingAckId) {
            awaitingAckReceived = true;
            statusMsg = "ACK RX";
            Serial.printf("[ACK] Received for id=%s\n", ackId.c_str());
        }
        return;
    }

    // MSG frame format: MSG|<msgId>|<srcAddrHex>|<body>
    if (payload.startsWith("MSG|")) {
        int p1 = payload.indexOf('|');          // after MSG
        int p2 = payload.indexOf('|', p1 + 1);  // after id
        int p3 = payload.indexOf('|', p2 + 1);  // after src
        if (p1 >= 0 && p2 > p1 && p3 > p2) {
            String msgId = payload.substring(p1 + 1, p2);
            String srcHex = payload.substring(p2 + 1, p3);
            String body = payload.substring(p3 + 1);

            int16_t srcAddr = parseHex16(srcHex);
            if (srcAddr >= 0) {
                uint8_t srcAddh = (uint8_t)((srcAddr >> 8) & 0xFF);
                uint8_t srcAddl = (uint8_t)(srcAddr & 0xFF);
                String ackPayload = "ACK|" + msgId;
                e22.sendFixedMessage(srcAddh, srcAddl, CHANNEL, ackPayload);
            }

            payload = body;
        }
    }

    receivedCount++;
    lastReceived = payload;
    lastRssiSignal = (int8_t)rc.rssi;
    statusMsg = "RX OK";

    // Learn nearby nodes via HELLO beacons:
    // Payload format: HELLO|AABB|NN|CC|R/N  or  HELLO|AABB|NN|CC|R/N|callSign
    if (payload.startsWith("HELLO|")) {
        int p1 = payload.indexOf('|');                 // after HELLO
        int p2 = payload.indexOf('|', p1 + 1);         // after addr
        int p3 = payload.indexOf('|', p2 + 1);         // after net
        int p4 = payload.indexOf('|', p3 + 1);         // after chan

        if (p1 >= 0 && p2 > p1 && p3 > p2 && p4 > p3) {
            String addrHex = payload.substring(p1 + 1, p2);
            String netHex  = payload.substring(p2 + 1, p3);
            String chHex   = payload.substring(p3 + 1, p4);
            int p5 = payload.indexOf('|', p4 + 1);
            String role, csIn;
            if (p5 < 0) {
                role = payload.substring(p4 + 1);
                role.trim();
            } else {
                role = payload.substring(p4 + 1, p5);
                role.trim();
                csIn = payload.substring(p5 + 1);
                csIn.trim();
            }

            int16_t addrVal = parseHex16(addrHex);
            int16_t netVal  = parseHex8(netHex);
            int16_t chVal   = parseHex8(chHex);
            bool isRep = (role.length() > 0 && role.charAt(0) == 'R');
            if (addrVal >= 0 && netVal >= 0 && chVal >= 0) {
                upsertNearbyNode((uint16_t)addrVal, (uint8_t)netVal, (uint8_t)chVal, (int8_t)rc.rssi, isRep,
                                 csIn);
            }
        }
    }
    
    // addLogEntry("RX", (int8_t)rc.rssi, "????", rc.data);
    
    // Serial.printf("[RX] RSSI=%d msg=%s\n", (int8_t)rc.rssi, rc.data.c_str());
    // drawDisplay();
}

// =============================================================================
// Web Handlers
// =============================================================================

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LoRa Node - Dynamic Config</title>
  <style>
    body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee;}
    h1{color:#0af;}
    h2{color:#0f3;margin-top:30px;}
    .container{max-width:1200px;margin:0 auto;}
    .card{background:#16213e;padding:20px;border-radius:8px;margin-bottom:20px;box-shadow:0 4px 8px rgba(0,0,0,0.3);}
    .stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:15px;margin-bottom:20px;}
    .stat-box{background:#0f3460;padding:15px;border-radius:5px;text-align:center;}
    .stat-box h3{margin:0 0 10px 0;font-size:14px;color:#aaa;}
    .stat-box .value{font-size:24px;font-weight:bold;color:#0af;}
    .config-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;}
    .input-group{margin-bottom:15px;}
    .input-group label{display:block;margin-bottom:5px;color:#aaa;font-size:14px;}
    input[type=text],select{width:100%;padding:10px;background:#0f3460;color:#fff;border:1px solid #0af;border-radius:5px;font-size:14px;}
    input[type=checkbox]{width:20px;height:20px;margin-right:10px;}
    button{background:#0af;color:#000;border:none;padding:12px 25px;border-radius:5px;cursor:pointer;font-size:16px;font-weight:bold;margin:5px;}
    button:hover{background:#08d;}
    button.save{background:#0f3;}
    button.save:hover{background:#0c2;}
    button.relay{background:#f90;}
    button.relay:hover{background:#d70;}
    button.restart{background:#f00;}
    button.restart:hover{background:#c00;}
    table{width:100%;border-collapse:collapse;margin-top:15px;}
    th,td{padding:10px;text-align:left;border-bottom:1px solid #333;}
    th{background:#0f3460;color:#0af;}
    tr:hover{background:#1f2f4f;}
    .tx{color:#0f3;}
    .rx{color:#0af;}
    .warning{background:#f90;color:#000;padding:10px;border-radius:5px;margin:10px 0;}
  </style>
  <script>
    function sendMsg(useRelay) {
      let msg = document.getElementById('msgInput').value;
      if (!msg) { alert('Enter a message!'); return; }
      
      let to = document.getElementById('nodeSelect') ? document.getElementById('nodeSelect').value : '';
      let base = useRelay ? '/send?relay=1' : '/send?';
      let url = base + (base.endsWith('?') ? '' : '&') + 'msg=' + encodeURIComponent(msg);
      if (to && to !== 'default') url += '&to=' + encodeURIComponent(to);

      fetch(url)
        .then(r => r.text())
        .then(() => {
          document.getElementById('msgInput').value = '';
          refreshData();
        });
    }
    
    function saveConfig() {
      let data = {
        my_addh: document.getElementById('my_addh').value,
        my_addl: document.getElementById('my_addl').value,
        target_addh: document.getElementById('target_addh').value,
        target_addl: document.getElementById('target_addl').value,
        rep_addh: document.getElementById('rep_addh').value,
        rep_addl: document.getElementById('rep_addl').value,
        netid: document.getElementById('netid').value,
        channel: document.getElementById('channel').value,
        crypt: document.getElementById('crypt').value,
        use_repeater: document.getElementById('use_repeater').checked ? '1' : '0'
      };
      
      let params = new URLSearchParams(data).toString();
      
      if(confirm('Save configuration and restart node?')) {
        fetch('/config?' + params)
          .then(r => r.text())
          .then(msg => {
            alert(msg);
            setTimeout(() => location.reload(), 3000);
          });
      }
    }
    
    function refreshData() {
      fetch('/api/status')
        .then(r => r.json())
        .then(data => {
          // Support both old flat JSON and new nested JSON.
          const node = data.node || data;
          const traffic = data.traffic || data;
          const fmtHex = (value, fallback = '--') => value ? ('0x' + value) : fallback;

          document.getElementById('uptime').textContent = node.uptime || '--';
          document.getElementById('sentCount').textContent = (traffic.sent ?? 0);
          document.getElementById('rxCount').textContent = (traffic.received ?? 0);
          document.getElementById('lastTx').textContent = traffic.lastSent || '(none)';
          document.getElementById('lastRx').textContent = traffic.lastReceived || '(none)';
          document.getElementById('myAddr').textContent = fmtHex(node.myAddr);
          document.getElementById('targetAddr').textContent = fmtHex(node.targetAddr);
          document.getElementById('netId').textContent = fmtHex(node.netId);
        });
      
      fetch('/api/log')
        .then(r => r.json())
        .then(data => {
          let tbody = document.getElementById('logBody');
          tbody.innerHTML = '';
          data.forEach(entry => {
            let tr = document.createElement('tr');
            tr.innerHTML = `
              <td>${entry.time}</td>
              <td class="${entry.dir.toLowerCase()}">${entry.dir}</td>
              <td>${entry.rssi} dBm</td>
              <td>0x${entry.addr}</td>
              <td>${entry.msg}</td>
            `;
            tbody.appendChild(tr);
          });
        });

      fetch('/api/nodes')
        .then(r => r.json())
        .then(payload => {
          const sel = document.getElementById('nodeSelect');
          if (!sel) return;

          const nodes = Array.isArray(payload) ? payload : (payload.nodes || []);

          const keep = sel.value;
          sel.innerHTML = '';

          const optDefault = document.createElement('option');
          optDefault.value = 'default';
          optDefault.textContent = 'Use configured target';
          sel.appendChild(optDefault);

          nodes
            .sort((a, b) => (a.lastSeenMs ?? 0) - (b.lastSeenMs ?? 0))
            .forEach(n => {
              const opt = document.createElement('option');
              opt.value = n.addr; // AABB
              const age = (n.lastSeenMs ?? 0);
              const sec = Math.round(age / 1000);
              const on = (n.online !== false);
              opt.textContent = `0x${n.addr}  RSSI:${n.rssi}  ${on ? '' : '[off] '}seen:${sec}s${n.repeater ? '  (repeater)' : ''}`;
              sel.appendChild(opt);
            });

          if (keep) sel.value = keep;
        });
    }
    
    setInterval(refreshData, 2000);
    window.onload = refreshData;
  </script>
</head>
<body>
  <div class="container">
    <h1>📡 LoRa Node - Dynamic Configuration</h1>
    <p style="margin:-8px 0 16px 0"><a href="/setup" style="color:#0af">WiFi setup</a> · <a href="/status" style="color:#0af">Device &amp; E22 status</a> · <a href="/api/status" style="color:#888">JSON</a></p>
    
    <div class="card">
      <h2>Current Status</h2>
      <div class="stats">
        <div class="stat-box">
          <h3>My Address</h3>
          <div class="value" id="myAddr">)rawliteral"+ toHex2(MY_ADDH) + toHex2(MY_ADDL)
            + R"rawliteral("</div>
        </div>
        <div class="stat-box">
          <h3>Target</h3>
          <div class="value" id="targetAddr">)rawliteral"+ toHex2(TARGET_ADDH) + toHex2(TARGET_ADDL)
            + R"rawliteral("</div>
        </div>
        <div class="stat-box">
          <h3>Network ID</h3>
          <div class="value" id="netId">)rawliteral"+ toHex2(NETWORK_ID)
            + R"rawliteral("</div>
        </div>
        <div class="stat-box">
          <h3>Uptime</h3>
          <div class="value" id="uptime">--</div>
        </div>
        <div class="stat-box">
          <h3>Received</h3>
          <div class="value" id="rxCount">0</div>
        </div>
        <div class="stat-box">
          <h3>Sent</h3>
          <div class="value" id="sentCount">0</div>
        </div>
      </div>
    </div>
    
    <div class="card">
      <h2>⚙️ Node Configuration</h2>
      <div class="warning">⚠️ Changing configuration will restart the node!</div>
      
      <div class="config-grid">
        <div class="input-group">
          <label>My Address High (ADDH)</label>
          <input type="text" id="my_addh" value=")rawliteral"+ toHex2(MY_ADDH)
            + R"rawliteral(" maxlength="4" placeholder="00">
        </div>
        <div class="input-group">
          <label>My Address Low (ADDL)</label>
          <input type="text" id="my_addl" value=")rawliteral"+ toHex2(MY_ADDL)
          +R"rawliteral(" maxlength="4" placeholder="02">
        </div>
        <div class="input-group">
          <label>Target Address High</label>
          <input type="text" id="target_addh" value=")rawliteral"+ toHex2(TARGET_ADDH)
          +R"rawliteral(" maxlength="2" placeholder="00">
        </div>
        <div class="input-group">
          <label>Target Address Low</label>
          <input type="text" id="target_addl" value=")rawliteral"+ toHex2(TARGET_ADDL)
          +R"rawliteral(" maxlength="2" placeholder="01">
        </div>
        <div class="input-group">
          <label>Repeater Address High</label>
          <input type="text" id="rep_addh" value=")rawliteral"+ toHex2(REPEATER_ADDH)
          +R"rawliteral(" maxlength="2" placeholder="FF">
        </div>
        <div class="input-group">
          <label>Repeater Address Low</label>
          <input type="text" id="rep_addl" value=")rawliteral"+ toHex2(REPEATER_ADDL)
          +R"rawliteral(" maxlength="2" placeholder="FF">
        </div>
        <div class="input-group">
          <label>Network ID (NETID)</label>
          <input type="text" id="netid" value=")rawliteral"+ toHex2(NETWORK_ID)
          +R"rawliteral(" maxlength="2" placeholder="00">
        </div>
        <div class="input-group">
          <label>Channel (00-FF)</label>
          <input type="text" id="channel" value=")rawliteral"+ toHex2(CHANNEL)
          +R"rawliteral(" maxlength="2" placeholder="41">
        </div>
        <div class="input-group">
          <label>CRYPT key (4 hex digits, same on all nodes; 0000 = off)</label>
          <input type="text" id="crypt" value=")rawliteral" + toHex4(CRYPT) + R"rawliteral(" maxlength="4" placeholder="1234">
        </div>
      </div>
      
      <div class="input-group">
        <label>
          <input type="checkbox" id="use_repeater">
          Enable as Repeater (forward all messages)
        </label>
      </div>
      
      <button class="save" onclick="saveConfig()">💾 Save & Apply Configuration</button>
    </div>
    
    <div class="card">
      <h2>Send Message</h2>
      <p><strong>Last RX:</strong> <span id="lastRx">(none)</span></p>
      <p><strong>Last TX:</strong> <span id="lastTx">(none)</span></p>
      <div class="input-group">
        <label>Send To (nearby nodes)</label>
        <select id="nodeSelect">
          <option value="default">Use configured target</option>
        </select>
      </div>
      <input type="text" id="msgInput" placeholder="Type your message..." maxlength="200">
      <div>
        <button onclick="sendMsg(false)">📤 Send Direct</button>
        <button class="relay" onclick="sendMsg(true)">🔄 Send via Repeater</button>
      </div>
    </div>
    
    <div class="card">
      <h2>Message Log</h2>
      <table>
        <thead>
          <tr>
            <th>Time</th>
            <th>Direction</th>
            <th>RSSI</th>
            <th>Address</th>
            <th>Message</th>
          </tr>
        </thead>
        <tbody id="logBody">
          <tr><td colspan="5" style="text-align:center;">Loading...</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

void handleSetupPage() {
    String html;
    html.reserve(2500);
    html += F("<!DOCTYPE html><html><head><meta charset=\"utf-8\">");
    html += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
    html += F("<title>WiFi Setup</title>");
    html += F("<style>");
    html += F("body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee;}");
    html += F(".card{background:#16213e;padding:20px;border-radius:8px;max-width:680px;margin:0 auto;box-shadow:0 4px 8px rgba(0,0,0,0.3);}");
    html += F("h1{color:#0af;margin-top:0;}label{display:block;margin:10px 0 6px;color:#aaa;}");
    html += F("input[type=text]{width:100%;padding:10px;background:#0f3460;color:#fff;border:1px solid #0af;border-radius:5px;font-size:15px;box-sizing:border-box;}");
    html += F("button{background:#0f3;color:#000;border:none;padding:12px 20px;border-radius:5px;cursor:pointer;font-size:15px;font-weight:bold;margin-top:14px;}");
    html += F("button:hover{background:#0c2;}a{color:#0af;}");
    html += F(".muted{color:#888;font-size:13px;}.warn{margin-top:12px;padding:10px;background:#f90;color:#000;border-radius:5px;}");
    html += F("</style></head><body><div class=\"card\">");
    html += F("<p><a href=\"/\">&larr; Back to control panel</a></p>");
    html += F("<h1>WiFi AP setup</h1>");
    html += F("<form method=\"POST\" action=\"/setup/save\">");
    html += F("<label for=\"ssid_ap\">AP SSID</label>");
    html += F("<input id=\"ssid_ap\" name=\"ssid_ap\" type=\"text\" maxlength=\"32\" required value=\"");
    html += htmlEscape(ssid_AP);
    html += F("\">");
    html += F("<p class=\"muted\">Current AP IP: ");
    html += htmlEscape(WiFi.softAPIP().toString());
    html += F("</p>");
    html += F("<div class=\"warn\">Saving will restart the device and AP will reconnect with the new SSID.</div>");
    html += F("<button type=\"submit\">Save to flash & restart</button>");
    html += F("</form></div></body></html>");
    server.send(200, "text/html", html);
}

void handleSetupSave() {
    if (!server.hasArg("ssid_ap")) {
        server.send(400, "text/plain", "Missing ssid_ap");
        return;
    }

    String newSsid = server.arg("ssid_ap");
    newSsid.trim();

    if (newSsid.length() == 0 || newSsid.length() > 32) {
        server.send(400, "text/plain", "Invalid SSID length (1..32)");
        return;
    }

    ssid_AP = newSsid;
    saveApConfig();

    server.send(200, "text/plain", "SSID saved to flash. Restarting...");
    delay(1000);
    ESP.restart();
}

void handleSend() {
    if (server.hasArg("msg")) {
        String msg = server.arg("msg");
        bool useRelay = server.hasArg("relay");
        bool overrideDest = false;
        uint8_t destAddh = 0;
        uint8_t destAddl = 0;

        // Optional override target: /send?to=AABB
        if (server.hasArg("to")) {
            int16_t addr = parseHex16(server.arg("to"));
            if (addr >= 0) {
                destAddh = (uint8_t)((addr >> 8) & 0xFF);
                destAddl = (uint8_t)(addr & 0xFF);
                overrideDest = true;
            }
        }

        // Optional override target: /send?addh=AA&addl=BB (takes priority)
        if (server.hasArg("addh") && server.hasArg("addl")) {
            int16_t ah = parseHex8(server.arg("addh"));
            int16_t al = parseHex8(server.arg("addl"));
            if (ah >= 0 && al >= 0) {
                destAddh = (uint8_t)ah;
                destAddl = (uint8_t)al;
                overrideDest = true;
            }
        }

        bool acked = sendMessage(msg, useRelay, destAddh, destAddl, overrideDest);
        if (acked) {
            server.send(200, "text/plain", "OK (ACK)");
        } else {
            server.send(504, "text/plain", "Timeout waiting ACK");
        }
    } else {
        server.send(400, "text/plain", "Missing msg parameter");
    }
}

void handleStatusPage() {
    String html;
    html.reserve(4500);

    html += F("<!DOCTYPE html><html><head><meta charset=\"utf-8\">");
    html += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
    html += F("<title>ESP32 & E22 Status</title>");
    html += F("<style>");
    html += F("body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee;}");
    html += F("h1{color:#0af;}h2{color:#0f3;margin-top:28px;}a{color:#0af}");
    html += F(".card{background:#16213e;padding:20px;border-radius:8px;margin:0 auto;max-width:920px;box-shadow:0 4px 8px rgba(0,0,0,0.3);}");
    html += F("table{width:100%;border-collapse:collapse;margin-top:8px;}");
    html += F("th,td{padding:10px 8px;text-align:left;border-bottom:1px solid #333;vertical-align:top;}");
    html += F("th{width:42%;color:#aaa;font-weight:normal;}");
    html += F("td{color:#eee;font-family:monospace;font-size:14px;}");
    html += F(".muted{color:#888;font-size:13px;margin-top:12px;}");
    html += F("</style></head><body><div class=\"card\">");
    html += F("<p><a href=\"/\">← Control panel</a></p>");
    html += F("<h1>Device &amp; E22 status</h1>");

    html += F("<h2>ESP32</h2><table>");
    appendStatusRow(html, "Chip", String(ESP.getChipModel()));
    appendStatusRow(html, "SDK", String(ESP.getSdkVersion()));
    html += F("<tr><th>CPU MHz</th><td>");
    html += String(ESP.getCpuFreqMHz());
    html += F(" MHz</td></tr>");
    html += F("<tr><th>Flash</th><td>");
    html += String(ESP.getFlashChipSize() / 1024);
    html += F(" KB</td></tr>");
    html += F("<tr><th>Free heap</th><td>");
    html += String(ESP.getFreeHeap());
    html += F(" bytes</td></tr>");
    appendStatusRow(html, "Uptime", getUptime());
    appendStatusRow(html, "WiFi mode", String(WiFi.getMode() == WIFI_AP ? "AP" : "Other"));
    appendStatusRow(html, "callSign", ssid_AP);
    appendStatusRow(html, "AP SSID", ssid_AP);
    appendStatusRow(html, "AP IP", WiFi.softAPIP().toString());
    uint8_t mac[6];
    WiFi.softAPmacAddress(mac);
    char macBuf[18];
    snprintf(macBuf, sizeof(macBuf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    appendStatusRow(html, "AP MAC", String(macBuf));
    appendStatusRow(html, "Node ready", String(nodeReady ? "yes" : "no"));
    appendStatusRow(html, "Status", statusMsg);
    appendStatusRow(html, "My address", "0x" + toHex2(MY_ADDH) + toHex2(MY_ADDL));
    appendStatusRow(html, "Target address", "0x" + toHex2(TARGET_ADDH) + toHex2(TARGET_ADDL));
    appendStatusRow(html, "Repeater address", "0x" + toHex2(REPEATER_ADDH) + toHex2(REPEATER_ADDL));
    appendStatusRow(html, "Network ID", "0x" + toHex2(NETWORK_ID));
    appendStatusRow(html, "Channel", "0x" + toHex2(CHANNEL));
    appendStatusRow(html, "CRYPT (hex)", "0x" + toHex4(CRYPT));
    appendStatusRow(html, "Use repeater (app)", USE_REPEATER ? "yes" : "no");
    appendStatusRow(html, "Battery", String(batteryPercent));
    appendStatusRow(html, "Charging", String(batteryCharging));
    html += F("</table>");

    html += F("<h2>E22 module (read from radio)</h2><table>");
    ResponseStructContainer c = e22.getConfiguration();
    if (c.status.code == 1) {
        Configuration cfg = *(Configuration*)c.data;
        appendStatusRow(html, "ADDH / ADDL", "0x" + toHex2(cfg.ADDH) + " / 0x" + toHex2(cfg.ADDL));
        appendStatusRow(html, "NETID", "0x" + toHex2(cfg.NETID));
        appendStatusRow(html, "CHAN", "0x" + toHex2(cfg.CHAN));
        html += F("<tr><th>UART baud (enum)</th><td>");
        html += String((uint8_t)cfg.SPED.uartBaudRate);
        html += F("</td></tr>");
        html += F("<tr><th>Air data rate (enum)</th><td>");
        html += String((uint8_t)cfg.SPED.airDataRate);
        html += F("</td></tr>");
        html += F("<tr><th>UART parity (enum)</th><td>");
        html += String((uint8_t)cfg.SPED.uartParity);
        html += F("</td></tr>");
        html += F("<tr><th>Sub-packet (enum)</th><td>");
        html += String((uint8_t)cfg.OPTION.subPacketSetting);
        html += F("</td></tr>");
        html += F("<tr><th>RSSI ambient noise (enum)</th><td>");
        html += String((uint8_t)cfg.OPTION.RSSIAmbientNoise);
        html += F("</td></tr>");
        html += F("<tr><th>TX power (enum)</th><td>");
        html += String((uint8_t)cfg.OPTION.transmissionPower);
        html += F("</td></tr>");
        html += F("<tr><th>RSSI on wire (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.enableRSSI);
        html += F("</td></tr>");
        html += F("<tr><th>Last RX RSSI (dBm)</th><td>");
        html += String(lastRssiSignal);
        html += F("</td></tr>");
        html += F("<tr><th>Fixed transmission (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.fixedTransmission);
        html += F("</td></tr>");
        html += F("<tr><th>Repeater (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.enableRepeater);
        html += F("</td></tr>");
        html += F("<tr><th>LBT (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.enableLBT);
        html += F("</td></tr>");
        html += F("<tr><th>WOR transceiver (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.WORTransceiverControl);
        html += F("</td></tr>");
        html += F("<tr><th>WOR period (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.WORPeriod);
        html += F("</td></tr>");
    } else {
        html += F("<tr><td colspan=\"2\">getConfiguration failed (code ");
        html += String(c.status.code);
        html += F(")</td></tr>");
    }
    c.close();
    html += F("</table>");

    html += F("<p class=\"muted\">Machine-readable: <a href=\"/api/status\">/api/status</a></p>");
    html += F("</div></body></html>");

    server.send(200, "text/html", html);
}

void handleConfig() {
    if (server.hasArg("my_addh") && server.hasArg("my_addl")) {
        // Parse all hex values
        MY_ADDH = (uint8_t)strtol(server.arg("my_addh").c_str(), NULL, 16);
        MY_ADDL = (uint8_t)strtol(server.arg("my_addl").c_str(), NULL, 16);
        TARGET_ADDH = (uint8_t)strtol(server.arg("target_addh").c_str(), NULL, 16);
        TARGET_ADDL = (uint8_t)strtol(server.arg("target_addl").c_str(), NULL, 16);
        REPEATER_ADDH = (uint8_t)strtol(server.arg("rep_addh").c_str(), NULL, 16);
        REPEATER_ADDL = (uint8_t)strtol(server.arg("rep_addl").c_str(), NULL, 16);
        NETWORK_ID = (uint8_t)strtol(server.arg("netid").c_str(), NULL, 16);
        CHANNEL = (uint8_t)strtol(server.arg("channel").c_str(), NULL, 16);
        USE_REPEATER = server.arg("use_repeater") == "1";
        if (server.hasArg("crypt")) {
            String ch = server.arg("crypt");
            ch.trim();
            ch.toUpperCase();
            if (ch.length() == 4) {
                int16_t v = parseHex16(ch);
                if (v >= 0) CRYPT = (uint16_t)v;
            }
        }
        
        saveConfig();
        
        server.send(200, "text/plain", "Configuration saved! Restarting...");
        
        delay(1000);
        ESP.restart();
    } else {
        server.send(400, "text/plain", "Missing parameters");
    }
}

void handleAPIStatus() {
    readBattery();
    String json = "{";

    // High-level runtime and node addressing info.
    json += "\"node\":{";
    json += "\"ready\":" + String(nodeReady ? "true" : "false") + ",";
    json += "\"status\":\"" + jsonEscape(statusMsg) + "\",";
    json += "\"callSign\":\"" + String(ssid_AP) + "\",";
    json += "\"uptime\":\"" + jsonEscape(getUptime()) + "\",";
    json += "\"battery\":" + String(batteryPercent) + ",";
    json += "\"charging\":\"" + jsonEscape(getCharging()) + "\",";
    json += "\"ip\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\",";
    json += "\"myAddr\":\"" + toHex2(MY_ADDH) + toHex2(MY_ADDL) + "\",";
    json += "\"targetAddr\":\"" + toHex2(TARGET_ADDH) + toHex2(TARGET_ADDL) + "\",";
    json += "\"repeaterAddr\":\"" + toHex2(REPEATER_ADDH) + toHex2(REPEATER_ADDL) + "\",";
    json += "\"netId\":\"" + toHex2(NETWORK_ID) + "\",";
    json += "\"channel\":\"" + toHex2(CHANNEL) + "\",";
    json += "\"crypt\":\"" + toHex4(CRYPT) + "\",";
    json += "\"useRepeater\":" + String(USE_REPEATER ? "true" : "false");
    json += "},";

    // Traffic counters and last payloads.
    json += "\"traffic\":{";
    json += "\"sent\":" + String(sentCount) + ",";
    json += "\"received\":" + String(receivedCount) + ",";
    json += "\"lastSent\":\"" + jsonEscape(lastSent) + "\",";
    json += "\"lastReceived\":\"" + jsonEscape(lastReceived) + "\"";
    json += "},";

    // Read back active E22 register-level config from the module.
    json += "\"e22\":{";
    ResponseStructContainer c = e22.getConfiguration();
    if (c.status.code == 1) {
        Configuration cfg = *(Configuration*)c.data;
        json += "\"ok\":true,";
        json += "\"addh\":\"" + toHex2(cfg.ADDH) + "\",";
        json += "\"addl\":\"" + toHex2(cfg.ADDL) + "\",";
        json += "\"netid\":\"" + toHex2(cfg.NETID) + "\",";
        json += "\"chan\":\"" + toHex2(cfg.CHAN) + "\",";
        json += "\"uartBaudRate\":" + String((uint8_t)cfg.SPED.uartBaudRate) + ",";
        json += "\"airDataRate\":" + String((uint8_t)cfg.SPED.airDataRate) + ",";
        json += "\"uartParity\":" + String((uint8_t)cfg.SPED.uartParity) + ",";
        json += "\"subPacketSetting\":" + String((uint8_t)cfg.OPTION.subPacketSetting) + ",";
        json += "\"rssiAmbientNoise\":" + String((uint8_t)cfg.OPTION.RSSIAmbientNoise) + ",";
        json += "\"transmissionPower\":" + String((uint8_t)cfg.OPTION.transmissionPower) + ",";
        json += "\"enableRSSI\":" + String((uint8_t)cfg.TRANSMISSION_MODE.enableRSSI) + ",";
        json += "\"lastSignalRssi\":" + String(lastRssiSignal) + ",";
        json += "\"fixedTransmission\":" + String((uint8_t)cfg.TRANSMISSION_MODE.fixedTransmission) + ",";
        json += "\"enableRepeater\":" + String((uint8_t)cfg.TRANSMISSION_MODE.enableRepeater) + ",";
        json += "\"enableLBT\":" + String((uint8_t)cfg.TRANSMISSION_MODE.enableLBT) + ",";
        json += "\"worTransceiverControl\":" + String((uint8_t)cfg.TRANSMISSION_MODE.WORTransceiverControl) + ",";
        json += "\"worPeriod\":" + String((uint8_t)cfg.TRANSMISSION_MODE.WORPeriod) + ",";
        json += "\"cryptH\":\"" + toHex2(cfg.CRYPT.CRYPT_H) + "\",";
        json += "\"cryptL\":\"" + toHex2(cfg.CRYPT.CRYPT_L) + "\"";
    } else {
        json += "\"ok\":false,";
        json += "\"error\":\"getConfiguration failed (" + String(c.status.code) + ")\"";
    }
    c.close();
    json += "}";

    json += "}";
    server.send(200, "application/json", json);
}

void handleAPILog() {
    String json = "[";
    for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
        int idx = (logIndex - MAX_LOG_ENTRIES + i + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
        if (messageLog[idx].timestamp == 0) continue;
        
        if (json.length() > 1) json += ",";
        json += "{";
        json += "\"time\":\"" + String(messageLog[idx].timestamp / 1000) + "s\",";
        json += "\"dir\":\"" + jsonEscape(messageLog[idx].direction) + "\",";
        json += "\"rssi\":\"" + String(messageLog[idx].rssi) + "\",";
        json += "\"addr\":\"" + jsonEscape(messageLog[idx].address) + "\",";
        json += "\"msg\":\"" + jsonEscape(messageLog[idx].message) + "\"";
        json += "}";
    }
    json += "]";
    server.send(200, "application/json", json);
}

void handleAPINodes() {
    String json = "[";
    unsigned long now = millis();
    for (uint8_t i = 0; i < nearbyCount; i++) {
        if (json.length() > 1) json += ",";
        bool online = isNearbyNodeOnlineAt(i, now);
        json += "{";
        json += "\"addr\":\"" + toHex4(nearbyNodes[i].addr) + "\",";
        json += "\"netId\":\"" + toHex2(nearbyNodes[i].netId) + "\",";
        json += "\"channel\":\"" + toHex2(nearbyNodes[i].channel) + "\",";
        json += "\"rssi\":" + String(nearbyNodes[i].rssi) + ",";
        json += "\"lastSeenMs\":" + String(now - nearbyNodes[i].lastSeen) + ",";
        json += "\"repeater\":" + String(nearbyNodes[i].isRepeater ? "true" : "false") + ",";
        json += "\"online\":" + String(online ? "true" : "false") + ",";
        json += "\"callSign\":\"" + jsonEscape(nearbyNodes[i].callSign) + "\",";
        if (CRYPT != 0x0000) {
            json += "\"crypt\":\"enabled\"";
        } else {
            json += "\"crypt\":\"disabled\"";
        }
        json += "}";
    }
    json += "]";
    json = "{\"onlineCount\":" + String(countOnlineNearbyNodes(now)) + ",\"nodes\":" + json + "}";
    server.send(200, "application/json", json);
}

void sendHelloBeacon() {
    // Beacon format: HELLO|AABB|NN|CC|R/N|callSign  (pipe not allowed in callSign)
    String payload = "HELLO|";
    payload += toHex4((MY_ADDH << 8) | MY_ADDL);
    payload += "|";
    payload += toHex2(NETWORK_ID);
    payload += "|";
    payload += toHex2(CHANNEL);
    payload += "|";
    payload += (USE_REPEATER ? "R" : "N");
    {
        String cs = ssid_AP;
        cs.replace("|", "");
        payload += "|";
        payload += cs;
    }

    // Best-effort broadcast (0xFFFF). Doesn't affect counters/log.
    e22.sendFixedMessage(0xFF, 0xFF, CHANNEL, payload.c_str());
}

// Clear configuration (erase stored NVS prefs for this namespace)
void ClearConfig() {
    preferences.begin("lora-config", false);
    preferences.clear();
    preferences.end();
    Serial.println("Configuration cleared!");
}

// ────────────────────────────────────────────────
// Read battery from BAT_READER (ADC), charging from BAT_CHARGING
// ────────────────────────────────────────────────
float getBattV() {
  // Low-pass filter (exponential moving average) to smooth voltage
  static bool   battInit = false;
  static float  battFilt = 0.0f;

  int raw = analogRead(BAT_READER);
  float adcVoltage = (float)raw * 3.30f / (float)BAT_ADC_MAX; // ADC pin voltage
  float vInstant   = adcVoltage * 2.3f;                       // battery voltage (divider factor)

  const float alpha = 0.20f;  // 0..1, higher = faster response, lower = smoother
  if (!battInit) {
    battFilt = vInstant;
    battInit = true;
  } else {
    battFilt = battFilt + alpha * (vInstant - battFilt);
  }

  // Debug
  // Serial.print("BAT_READER raw: ");
  // Serial.println(raw);
  // Serial.print("BattV filtered: ");
  // Serial.println(battFilt, 2);

  return battFilt;
}

int getBattPct() {
  float v = getBattV();
  float pct = (v - (float)BAT_V_MIN) * 100.0f / ((float)BAT_V_MAX - (float)BAT_V_MIN);
  return constrain((int)(pct + 0.5f), 0, 100);
}

void readBattery() {
  batteryVoltage = getBattV();
  batteryPercent = getBattPct();

  static bool rawChargeState = false;
  static bool debouncedChargeState = false;
  static unsigned long lastRawChangeMs = 0;

  bool rawNow = (digitalRead(BAT_CHARGING) == HIGH);
  unsigned long now = millis();

  if (rawNow != rawChargeState) {
    rawChargeState = rawNow;
    lastRawChangeMs = now;
  }

  if ((now - lastRawChangeMs) >= CHARGE_PIN_DEBOUNCE_MS) {
    debouncedChargeState = rawChargeState;
  }

  lastBatteryCharging = batteryCharging;
  batteryCharging = debouncedChargeState;

  // Re-enable charging screen automatically on a new charging event.
  if (batteryCharging && !lastBatteryCharging) {
    chargeScreenSuppressed = false;
  }
}

String getBattery() {
  char buf[40];
  snprintf(buf, sizeof(buf), "%.2fV %d%%", batteryVoltage, batteryPercent);
  return String(buf);
}

String getCharging() {
  return batteryCharging ? "true" : "false";
}

String getCurrentTime() {
  unsigned long totalSeconds = (millis() - startTime) / 1000;
  char buf[8];
  snprintf(buf, sizeof(buf), "%02d:%02d", (int)((totalSeconds / 3600) % 24), (int)((totalSeconds / 60) % 60));
  return String(buf);
}

String getGPSMessage() {
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    char buf[80];
    snprintf(buf, sizeof(buf), "T:%lus Lat:%.5f Lng:%.5f S:%02lu",
             (unsigned long)(millis() / 1000), gps.location.lat(), gps.location.lng(),
             (unsigned long)gps.satellites.value());
    return String(buf);
  }
  char buf[64];
  snprintf(buf, sizeof(buf), "No fix  Sats:%02lu  T:%lus",
           (unsigned long)gps.satellites.value(), (unsigned long)(millis() / 1000));
  return String(buf);
}

// ────────────────────────────────────────────────
// Draw header with rounded corners background (radius ~2px)
// ────────────────────────────────────────────────
void drawHeader() {
  // Draw filled rectangle with rounded corners (radius 1px)
  // Clear full top band to avoid old pixels after long uptime
  display.fillRect(0, 0, SCREEN_WIDTH, 13, SSD1306_BLACK);
  // Then filled rounded area across full width
  display.fillRoundRect(0, 0, SCREEN_WIDTH, 13, 3, SSD1306_WHITE);
  // Optional: white outline on top (makes it look sharper)
  display.drawRoundRect(0, 0, SCREEN_WIDTH, 13, 3, SSD1306_WHITE);
  
  // Draw text in inverse (black on white)
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);

  int batX = 6;   // left position of the battery body
  int batY = 2;   // top position of the whole battery (including nub)
  // Nub
  display.fillRect(batX + 1, batY - 1, 3, 1, SSD1306_BLACK);

  // Main body outline
  display.drawRect(batX - 1, batY, 7, 9, SSD1306_BLACK);

  // Fill (4 pixels usable height inside) – refreshed from BAT_READER
  int bars = map(batteryPercent, 0, 100, 0, 4);
  if (bars > 0) {
    display.fillRect(batX + 1, batY + 7 - bars, 3, bars, SSD1306_WHITE);
  }

  // Charging: lightning bolt inside battery when BAT_CHARGING is HIGH
  if (batteryCharging) {
    display.drawPixel(batX + 3, batY + 2, SSD1306_BLACK);
    display.drawPixel(batX + 2, batY + 3, SSD1306_BLACK);
    display.drawPixel(batX + 2, batY + 5, SSD1306_BLACK);
    display.drawPixel(batX + 1, batY + 4, SSD1306_BLACK);
    display.drawPixel(batX + 3, batY + 4, SSD1306_BLACK);
    display.drawPixel(batX + 1, batY + 6, SSD1306_BLACK);
  }

  // Percentage next to icon (refreshed from BAT_READER)
  display.setCursor(batX + 7, 3);
  display.print(batteryPercent);
  display.print("%");
  // if (batteryCharging) {
  //   display.print("+");  // charging indicator
  // }

  // Time on the right
  display.setCursor(92, 3);
  display.print(getCurrentTime());
  
  // Reset text color for rest of screen
  display.setTextColor(SSD1306_WHITE);
}

// ────────────────────────────────────────────────
// OLED drawing - Meshtastic-style layout
// ────────────────────────────────────────────────
// Screen 1: Status screen
// ────────────────────────────────────────────────
void drawStatusScreen() {
  display.clearDisplay();
  
  // Draw header with rounded corners
  drawHeader();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Same source as /api/nodes: registry + ONLINE_NODE_TTL_MS
  syncOnlineNodesFromRegistry();
  
  // Line 2: Online nodes and Uptime
  display.setCursor(2, 16);
  display.print((char)0x09);
  display.print(onlineNodes);
  display.print(" online");
  
  display.setCursor(74, 16);
  display.print("Up:");
  display.print(getUptime());
  
  // Line 3: Satellites and Voltage
  display.setCursor(2, 26);
  display.print("#");
  display.print(gps.satellites.value());
  display.print(" sats");
  
  display.setCursor(82, 26);
  display.print(batteryVoltage, 2);
  display.print("V");
  
  // Line 4: Channel Utilization
  display.setCursor(2, 37);
  display.print("ChUtil.");
  
  int chUtil = 0;
  int barX = 50;
  int barY = 37;
  int barWidth = 48;
  display.drawRect(barX, barY, barWidth, 6, SSD1306_WHITE);
  if (chUtil > 0) {
    int fillWidth = (chUtil * (barWidth - 2)) / 100;
    display.fillRect(barX + 1, barY + 1, fillWidth, 4, SSD1306_WHITE);
  }
  
  display.setCursor(100, 37);
  display.print(chUtil);
  display.print("%");
  
  // Line 5: Callsign
  display.setCursor(20, 49);
  display.setTextSize(1);
  display.print(ssid_AP);
  display.setTextSize(1);
  display.print(" (DBR)");
  
  // Bottom progress bar (width = SCREEN_WIDTH/2, centered)
  int progress = 0;
  int progBarWidth = SCREEN_WIDTH / 2;  // 64
  int progBarX = (SCREEN_WIDTH - progBarWidth) / 2;  // centered
  int progBarY = SCREEN_HEIGHT - 4;  // near bottom
  display.drawRect(progBarX, progBarY, progBarWidth, 2, SSD1306_WHITE);
  if (progress > 0) {
    int fillWidth = (progress * (progBarWidth - 2)) / 100;
    display.fillRect(progBarX + 1, progBarY + 1, fillWidth, 1, SSD1306_WHITE);
  }
  
  display.display();
}

// ────────────────────────────────────────────────
// Screen 2: WiFi info
// ────────────────────────────────────────────────
void drawWifiScreen() {
  display.clearDisplay();
  drawHeader();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(4, 18);
  display.print("SSID:");
  display.print(ssid_AP);
  
  display.setCursor(4, 30);
  display.print("PASS:");
  display.print(password_AP);
  
  display.setCursor(4, 42);
  display.print("IP: ");
  display.print(WiFi.softAPIP().toString());
  
  display.display();
}

// ────────────────────────────────────────────────
// Screen 3: GPS info
// ────────────────────────────────────────────────
void drawGpsScreen() {
  getGPSMessage();
  display.clearDisplay();
  drawHeader();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(4, 16);
  display.print("Sats: ");
  display.print(gps.satellites.value());
  display.setCursor(60, 16);
  display.print("Up: ");
  display.print(getUptime());
  
  if (gps.location.isValid()) {
    display.setCursor(4, 26);
    display.print("Lat: ");
    display.print(gps.location.lat(), 5);
    display.setCursor(4, 36);
    display.print("Lng: ");
    display.print(gps.location.lng(), 5);
    if (gps.altitude.isValid()) {
      display.setCursor(4, 46);
      display.print("Alt: ");
      display.print((int)gps.altitude.meters());
      display.print("m");
    }
  } else {
    display.setCursor(4, 28);
    display.print("No GPS fix");
    display.setCursor(4, 40);
    display.print("Sats: ");
    display.print(gps.satellites.value());
  }
  Serial.println("----------------------------");
  Serial.print("GPS Satellites -> ");
  Serial.println(gps.satellites.value());
  Serial.print("Lat: ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Lng: ");
  Serial.println(gps.location.lng(), 5);
  Serial.println("----------------------------");
  display.display();
}

// ────────────────────────────────────────────────
// Screen 4: Charging screen
// ────────────────────────────────────────────────
void drawChargingScreen() {
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(18, 12);
  display.print("CHARGING");

  // Big battery icon
  const int x = 44;
  const int y = 32;
  const int w = 36;
  const int h = 18;
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.fillRect(x + w, y + 5, 3, 8, SSD1306_WHITE); // battery nub

  display.fillRect(x + 2, y + 2, w-4, h - 4, SSD1306_WHITE);

  // Lightning bolt blinks and moves slightly while charging.
  bool blinkOn = ((millis() / 400UL) % 2UL) == 0UL;
  if (blinkOn) {
    int px = x + 11;
    int py = y + 4;

    // Cable
    display.drawFastHLine(px - 4, py + 4, 4, SSD1306_BLACK);
    // Plug body
    display.fillRoundRect(px, py + 1, 7, 7, 1, SSD1306_BLACK);
    // Prongs
    display.fillRect(px + 7, py + 2, 2, 1, SSD1306_BLACK);
    display.fillRect(px + 7, py + 6, 2, 1, SSD1306_BLACK);
  }

  display.setTextSize(1);
  display.setCursor(2, 56);
  display.print("Hold NEXT 3s: Normal");
  display.display();
}

// ────────────────────────────────────────────────
// Draw current screen (router)
// ────────────────────────────────────────────────
void drawCurrentScreen() {
  if (batteryCharging && !chargeScreenSuppressed) {
    drawChargingScreen();
  } else if (currentScreen == 0) {
    drawStatusScreen();
  } else if (currentScreen == 1) {
    drawWifiScreen();
  } else {
    drawGpsScreen();
  }
}

void showSent(const String &message, const String &status) {
  display.clearDisplay();
  
  // Draw header with rounded corners
  drawHeader();
  
  // Draw line
  // display.drawFastHLine(0, 13, 128, SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(35, 19);
  display.println("SENT");

  display.setTextSize(1);
  display.setCursor(0, 39);
  display.print("-> ");
  
  // Two-line preview for long text
  String line1 = message.substring(0, 18);
  String line2 = "";
  if (message.length() > 18) {
    line2 = message.substring(18, min((int)message.length(), 36));
    if (message.length() > 36) {
      line2 += "...";
    }
  }

  display.println(line1);
  if (line2.length() > 0) {
    display.setCursor(0, 49);
    display.print("   "); // indent under arrow
    display.println(line2);
  }

  display.display();
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== LoRa Node with Dynamic Config ===");

    startTime = millis();
    // Clear cache
    // ClearConfig();

    // Load saved configuration
    loadConfig();

    pinMode(PIN_AUX, INPUT);
    pinMode(PIN_M0,  OUTPUT);
    pinMode(PIN_M1,  OUTPUT);

    // Initialize WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid_AP, password_AP,1,0,1);
    Serial.print("AP: ");
    Serial.println(ssid_AP);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());

    // Setup web server
    server.on("/", handleRoot);
    server.on("/setup", HTTP_GET, handleSetupPage);
    server.on("/setup/save", HTTP_POST, handleSetupSave);
    server.on("/send", handleSend);
    server.on("/config", handleConfig);
    server.on("/status", handleStatusPage);
    server.on("/api/status", handleAPIStatus);
    server.on("/api/log", handleAPILog);
    server.on("/api/nodes", handleAPINodes);
    server.begin();
    Serial.println("Web server started");

    // Initialize GPS
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
    // Initialize OLED
    Wire.begin(OLED_SDA, OLED_SCL);
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    display.setRotation(2);

    // Initialize E22
    if (!e22.begin()) {
        Serial.println("[E22] FAILED");
        statusMsg = "E22 FAIL";
        while (true) delay(1000);
    }

    // Apply configuration
    if (applyNodeConfig()) {
        nodeReady = true;
        statusMsg = USE_REPEATER ? "REPEATER" : "READY";
        Serial.printf("[READY] Node 0x%02X%02X active (Net: 0x%02X)\n", 
            MY_ADDH, MY_ADDL, NETWORK_ID);
    } else {
        statusMsg = "CFG FAIL";
    }

    pinMode(BTN_NEXT_PIN, INPUT_PULLUP);
    // GPIO35 has no internal pull-up/down on ESP32, so use external pull-up resistor.
    pinMode(BTN_PREV_PIN, INPUT);
    pinMode(BAT_READER, INPUT);
    pinMode(BAT_CHARGING, INPUT);
    readBattery();
    // lastSyncBroadcastMs = millis();
    drawCurrentScreen();
    // drawDisplay();
}

// =============================================================================
// Loop
// =============================================================================

void loop() {
    static unsigned long lastUiRefresh = 0;

    server.handleClient();

    if (nodeReady) {
        checkIncoming();
    }

    // Periodic discovery beacon to help UIs list nearby nodes
    if (nodeReady && (millis() - lastBeaconAt >= 5000)) {
        lastBeaconAt = millis();
        sendHelloBeacon();
    }

    // Buttons
  if (digitalRead(BTN_PREV_PIN) == LOW && (millis() - lastPrevBtnPress) > 300) {
    lastPrevBtnPress = millis();
    currentScreen = (currentScreen + 2) % 3;
    popupType = PopupType::None;
    drawCurrentScreen();
  }

  bool nextPressed = (digitalRead(BTN_NEXT_PIN) == LOW);
  if (nextPressed) {
    if (!nextBtnHeld) { nextBtnHeld = true; nextBtnLongSent = false; nextBtnDownMs = millis(); }
    else if (!nextBtnLongSent && (millis() - nextBtnDownMs >= BTN_NEXT_LONG_PRESS_MS)) {
      nextBtnLongSent = true;
      chargeScreenSuppressed = true;
      currentScreen = 0;
      popupType = PopupType::None;
      drawCurrentScreen();
    }
  } else if (nextBtnHeld) {
    unsigned long heldMs = millis() - nextBtnDownMs;
    if (!nextBtnLongSent && heldMs > 20 && (millis() - lastNextBtnPress) > 300) {
      lastNextBtnPress = millis();
      currentScreen = (currentScreen + 1) % 3;
      popupType = PopupType::None;
      drawCurrentScreen();
    }
    nextBtnHeld = false;
  }

  // Update GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (popupType != PopupType::None && (long)(millis() - popupUntilMs) >= 0) {
    popupType = PopupType::None;
    drawCurrentScreen();
  }

  static unsigned long lastBatteryRead = 0;
  if (millis() - lastBatteryRead >= 2000UL) {
    lastBatteryRead = millis();
    readBattery();
    if (popupType == PopupType::None) drawCurrentScreen();
  }

  bool chargeScreenVisible = (batteryCharging && !chargeScreenSuppressed);
  unsigned long uiRefreshPeriod = chargeScreenVisible ? UI_REFRESH_CHARGING_MS : UI_REFRESH_NORMAL_MS;
  if (popupType == PopupType::None && millis() - lastUiRefresh >= uiRefreshPeriod) {
    lastUiRefresh = millis();
    drawCurrentScreen();
  }
}