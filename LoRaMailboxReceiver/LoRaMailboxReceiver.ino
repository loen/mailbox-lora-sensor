#include "heltec.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include "arduino_secrets.h"   // Wi-Fi and MQTT – see arduino_secrets.h.example

// ================= State shared between cores =================
// The struct must be defined before the first function (Arduino IDE inserts prototypes there)
struct MailState {
  bool mail = false;
  unsigned long count = 0;
  int rssi = 0;
  float snr = 0;
  bool hasPacket = false;
  int batteryMv = 0;            // sender battery, 0 = not reported yet
};

// ---------- Wi-Fi (2.4 GHz only) ----------
const char* WIFI_SSID    = SECRET_WIFI_SSID;
const char* WIFI_PASS    = SECRET_WIFI_PASS;

// ---------- MQTT (Mosquitto in Home Assistant) ----------
const char* MQTT_HOST    = SECRET_MQTT_HOST;
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER    = SECRET_MQTT_USER;
const char* MQTT_PASS    = SECRET_MQTT_PASS;

// ===== Shared settings – MUST match the sender =====
#define BAND     869.525E6
#define LORA_SF  12
#define LORA_BW  125E3
#define LORA_CR  8
// ==================================================================

#define TX_POWER   20      // ACK transmit power
#define BUTTON_PIN 0       // PRG button – wakes the display / clears the "MAIL!" notification
#define SCREEN_TIMEOUT_MS 60000   // OLED turns off after this much inactivity (prevents burn-in)

// ---------- MQTT topics (must match configuration.yaml) ----------
#define MQTT_CLIENT_ID "mailbox_lora"
#define T_AVAIL  "custom/mailbox/availability"   // online / offline (LWT)
#define T_MAIL   "custom/mailbox/mail"           // ON / OFF
#define T_COUNT  "custom/mailbox/count"
#define T_RSSI   "custom/mailbox/rssi"
#define T_SNR    "custom/mailbox/snr"
#define T_CMD    "custom/mailbox/cmd"            // RESET from Home Assistant
#define T_BATT   "custom/mailbox/battery"          // sender battery, 0–100 %
#define T_BATT_V "custom/mailbox/battery_voltage"  // sender battery, volts

// ---------- SX1276 registers and pins ----------
#define LORA_NSS            18
#define LORA_DIO0           26                   // DIO0 = "packet received" in receive mode
#define REG_IRQ_FLAGS       0x12
#define REG_MODEM_CONFIG_3  0x26
#define IRQ_CRC_ERROR       0x20

MailState state;                                  // guarded by stateMux
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool statePending   = false;   // LoRa -> network: "publish new state"
volatile bool resetRequested = false;   // network -> LoRa: "RESET from Home Assistant"
volatile bool netWifiOk      = false;   // network status for the display
volatile bool netMqttOk      = false;

WiFiClient espClient;
PubSubClient mqtt(espClient);

long lastId = -1;
unsigned long mailTime = 0;

bool screenOn = true;                   // OLED state (core 1 only)
unsigned long screenOnSince = 0;        // last activity that kept the display on

// ================= Direct SX1276 register access =================
uint8_t sxRead(uint8_t reg) {
  SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_NSS, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(LORA_NSS, HIGH);
  SPI.endTransaction();
  return v;
}

void sxWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_NSS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(LORA_NSS, HIGH);
  SPI.endTransaction();
}

// Returns the received packet length or 0. The radio stays in continuous receive mode.
int pollPacket() {
  if (digitalRead(LORA_DIO0) == LOW) return 0;          // nothing received
  if (sxRead(REG_IRQ_FLAGS) & IRQ_CRC_ERROR) {
    Serial.println("!! Packet dropped: CRC error");
  }
  int size = LoRa.parsePacket();                         // read packet and clear flags
  if (size == 0) LoRa.receive();                         // corrupted – go back to receive
  return size;
}

MailState getState() {
  taskENTER_CRITICAL(&stateMux);
  MailState copy = state;
  taskEXIT_CRITICAL(&stateMux);
  return copy;
}

// Resting voltage of a single Li-ion / LiPo cell -> state of charge.
// 0 % is set at 3.3 V: below that the board's 3.3 V regulator drops out anyway.
int batteryPercent(int mv) {
  static const int curve[][2] = {
    {4200, 100}, {4100, 90}, {4000, 80}, {3900, 65}, {3800, 45},
    {3750, 30},  {3700, 20}, {3650, 12}, {3600, 7},  {3500, 3}, {3300, 0}
  };
  const int n = sizeof(curve) / sizeof(curve[0]);
  if (mv >= curve[0][0]) return 100;
  for (int i = 1; i < n; i++) {
    if (mv >= curve[i][0]) {                    // linear interpolation between two points
      return curve[i][1] + (long) (mv - curve[i][0]) * (curve[i - 1][1] - curve[i][1]) /
                           (curve[i - 1][0] - curve[i][0]);
    }
  }
  return 0;
}

// ================= Core 0: Wi-Fi + MQTT =================
void publishState() {
  MailState s = getState();
  mqtt.publish(T_MAIL, s.mail ? "ON" : "OFF", true);
  mqtt.publish(T_COUNT, String(s.count).c_str(), true);
  if (s.hasPacket) {
    mqtt.publish(T_RSSI, String(s.rssi).c_str(), true);
    mqtt.publish(T_SNR, String(s.snr, 1).c_str(), true);
  }
  if (s.batteryMv > 0) {
    mqtt.publish(T_BATT, String(batteryPercent(s.batteryMv)).c_str(), true);
    mqtt.publish(T_BATT_V, String(s.batteryMv / 1000.0, 2).c_str(), true);
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char) payload[i];
  if (String(topic) == T_CMD && msg == "RESET") {
    Serial.println("MQTT: RESET from Home Assistant");
    resetRequested = true;                        // handled by the LoRa loop
  }
}

void netTask(void *param) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setSocketTimeout(2);

  unsigned long lastTry = 0;

  for (;;) {
    netWifiOk = (WiFi.status() == WL_CONNECTED);

    if (netWifiOk && !mqtt.connected() && millis() - lastTry >= 5000) {
      lastTry = millis();
      Serial.print("MQTT: connecting... ");
      // may block for a few seconds – fine here, LoRa runs on the other core
      if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, T_AVAIL, 0, true, "offline")) {
        Serial.println("OK");
        mqtt.publish(T_AVAIL, "online", true);
        mqtt.subscribe(T_CMD);
        statePending = true;                      // publish current state after connecting
      } else {
        Serial.printf("failed, rc=%d\n", mqtt.state());
      }
    }

    netMqttOk = mqtt.connected();
    if (netMqttOk) {
      mqtt.loop();
      if (statePending) {
        statePending = false;
        publishState();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ================= Core 1: LoRa, display, button =================
void configureLoRa() {
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.enableCrc();
  LoRa.setTxPower(TX_POWER, RF_PACONFIG_PASELECT_PABOOST);

  // Low Data Rate Optimize – required for SF11/SF12 at 125 kHz (symbol > 16 ms)
  uint8_t mc3 = sxRead(REG_MODEM_CONFIG_3);
  sxWrite(REG_MODEM_CONFIG_3, mc3 | 0x08);
  Serial.printf("RegModemConfig3: 0x%02X -> 0x%02X (LDRO enabled)\n",
                mc3, sxRead(REG_MODEM_CONFIG_3));

  pinMode(LORA_DIO0, INPUT);
}

String agoText(unsigned long ms) {
  unsigned long s = ms / 1000;
  if (s < 60)   return String(s) + " s";
  if (s < 3600) return String(s / 60) + " min";
  return String(s / 3600) + " h " + String((s % 3600) / 60) + " min";
}

void drawScreen() {
  if (!screenOn) return;                      // display off – nothing to draw
  MailState s = getState();

  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);

  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, s.mail ? "MAIL!" : "No mail");

  Heltec.display->setFont(ArialMT_Plain_10);
  if (s.mail) {
    Heltec.display->drawString(0, 20, "Opened " + agoText(millis() - mailTime) + " ago");
  }
  String net = String("WiFi:") + (netWifiOk ? "OK" : "--") +
               " MQTT:" + (netMqttOk ? "OK" : "--");
  Heltec.display->drawString(0, 36, "Opens: " + String(s.count) + "  " + net);
  if (s.hasPacket) {
    Heltec.display->drawString(0, 50, "RSSI " + String(s.rssi) + "  SNR " + String(s.snr, 1));
  }
  if (s.batteryMv > 0) {
    Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
    Heltec.display->drawString(128, 3, String(batteryPercent(s.batteryMv)) + "%");
  }
  Heltec.display->display();
}

// Turns the display on (if needed) and restarts the inactivity timeout
void wakeScreen() {
  screenOnSince = millis();
  if (!screenOn) {
    screenOn = true;
    Heltec.display->displayOn();
  }
  drawScreen();
}

void sleepScreen() {
  screenOn = false;
  Heltec.display->displayOff();               // the LED still signals new mail
}

void clearMail() {
  taskENTER_CRITICAL(&stateMux);
  state.mail = false;
  taskEXIT_CRITICAL(&stateMux);
  digitalWrite(LED, LOW);
  statePending = true;
  drawScreen();
}

void sendAck(const String &id) {
  delay(100);                                 // give the sender time to switch to receive
  LoRa.beginPacket();
  LoRa.print("ACK;" + id);
  LoRa.endPacket();
  LoRa.receive();                             // back to continuous receive
  Serial.println("Sent ACK;" + id);
}

void handlePacket() {
  String p = "";
  while (LoRa.available()) p += (char) LoRa.read();
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  Serial.printf("Received: %s | RSSI %d dBm | SNR %.1f dB\n", p.c_str(), rssi, snr);

  // Format: "<type>;<id>[;<battery mV>]", type = MAILBOX_OPEN or BATTERY (heartbeat)
  bool isMail = p.startsWith("MAILBOX_OPEN;");
  if (!isMail && !p.startsWith("BATTERY;")) {
    LoRa.receive();                           // foreign packet – go straight back to listening
    return;
  }

  int idStart = p.indexOf(';') + 1;
  int idEnd = p.indexOf(';', idStart);
  String id = (idEnd < 0) ? p.substring(idStart) : p.substring(idStart, idEnd);
  int batteryMv = (idEnd < 0) ? 0 : p.substring(idEnd + 1).toInt();
  sendAck(id);                                // ACK immediately (radio returns to receive afterwards)

  long n = id.toInt();
  bool isNew = isMail && (n != lastId);
  if (!isMail) {
    Serial.printf("Battery report: %d mV (%d %%)\n", batteryMv, batteryPercent(batteryMv));
  } else if (isNew) {
    lastId = n;
    mailTime = millis();
    digitalWrite(LED, HIGH);
  } else {
    Serial.println("Duplicate - already counted");
  }

  taskENTER_CRITICAL(&stateMux);
  state.rssi = rssi;
  state.snr = snr;
  state.hasPacket = true;
  if (batteryMv > 0) state.batteryMv = batteryMv;
  if (isNew) {
    state.mail = true;
    state.count++;
  }
  taskEXIT_CRITICAL(&stateMux);

  statePending = true;                        // core 0 will publish this to MQTT
  if (isNew) wakeScreen();                    // new mail – show it
  else drawScreen();
}

// ================= Setup / loop =================
void setup() {
  Heltec.begin(true /*Display*/, true /*LoRa*/, true /*Serial*/,
               true /*PABOOST*/, BAND);
  configureLoRa();
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  wakeScreen();                               // display on, timeout starts now

  // Wi-Fi and MQTT in a separate task on core 0 (loop() runs on core 1)
  xTaskCreatePinnedToCore(netTask, "net", 8192, NULL, 1, NULL, 0);

  Serial.println("Receiver ready, waiting for the mailbox to open...");
  LoRa.receive();                             // continuous receive
}

void loop() {
  if (pollPacket()) {
    handlePacket();
  }

  if (resetRequested) {                       // RESET from Home Assistant
    resetRequested = false;
    clearMail();
  }

  static bool lastBtn = HIGH;
  bool btn = digitalRead(BUTTON_PIN);
  if (btn == LOW && lastBtn == HIGH) {        // PRG pressed (falling edge)
    if (!screenOn) {
      wakeScreen();                           // first press only wakes the display
    } else {
      screenOnSince = millis();               // keep the display on
      if (getState().mail) clearMail();       // PRG = "mail collected"
    }
    delay(50);                                // debounce
  }
  lastBtn = btn;

  if (screenOn && millis() - screenOnSince >= SCREEN_TIMEOUT_MS) {
    sleepScreen();
  }

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 1000) {          // refreshes "x min ago" and network status
    lastDraw = millis();
    drawScreen();
  }
  delay(5);
}
