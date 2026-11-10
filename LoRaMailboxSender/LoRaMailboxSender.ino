#include "heltec.h"
#include "driver/rtc_io.h"
#include <SPI.h>
#include <sys/time.h>

// ===== Shared settings – MUST match the receiver =====
#define BAND     869.525E6   // 869.4–869.65 MHz sub-band: 20 dBm allowed, 10% duty cycle
#define LORA_SF  12          // spreading factor: 12 = maximum range
#define LORA_BW  125E3       // bandwidth
#define LORA_CR  8           // coding rate 4/8 = strongest error correction
// ==================================================================

#define TX_POWER          20            // dBm, maximum for SX1276 (PA_BOOST)
#define REED_PIN          GPIO_NUM_13   // reed switch GPIO13 <-> GND, closed = flap open
#define VEXT_PIN          21            // OLED power (HIGH = off)
#define MAX_ATTEMPTS      3             // how many times to send if no ACK
#define ACK_TIMEOUT_MS    4000          // ACK wait time (an ACK at SF12 takes ~1.2 s)
#define STUCK_CHECK_S     30            // how often to check whether an open flap has closed
#define SERVICE_WINDOW_MS 10000         // after reset: time to upload firmware / open Serial Monitor

// ---------- Battery ----------
// WiFi LoRa 32 V2.1 (hardware >= 2.3): VBAT -> 220k/100k divider -> GPIO37, the divider is
// switched on together with Vext (GPIO21 LOW). Older boards used GPIO13 instead.
#define BATTERY_PIN       37
#define BATTERY_DIVIDER   3.2f          // (220k + 100k) / 100k – tune against a multimeter if needed
#define BATTERY_SAMPLES   16
#define BATTERY_REPORT_S  (24UL * 3600) // heartbeat: report the battery at least this often
#define BATTERY_RETRY_S   (1UL * 3600)  // retry sooner when a report was not acknowledged

// ---------- SX1276 registers and pins ----------
#define LORA_NSS            18
#define LORA_DIO0           26          // DIO0 = "packet received" in receive mode
#define REG_IRQ_FLAGS       0x12
#define REG_MODEM_CONFIG_3  0x26
#define IRQ_CRC_ERROR       0x20

RTC_DATA_ATTR uint32_t msgId = 0;       // message number – survives deep sleep
RTC_DATA_ATTR uint32_t nextBatteryReportS = 0;   // RTC time (s) of the next battery heartbeat

uint16_t batteryMv = 0;                 // measured once per wake-up, before any transmission

// Seconds on the RTC clock – keeps counting through deep sleep, restarts at 0 on power-on
uint32_t nowS() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint32_t) tv.tv_sec;
}

// ================= Battery =================
// Must run while the radio is idle: a 20 dBm transmission pulls the voltage down noticeably.
uint16_t readBatteryMv() {
  digitalWrite(VEXT_PIN, LOW);                // enable the divider (also powers the OLED rail)
  analogSetPinAttenuation(BATTERY_PIN, ADC_6db);   // ~0.15–1.75 V; 4.2 V / 3.2 = 1.31 V
  delay(20);                                  // let the divider settle
  analogReadMilliVolts(BATTERY_PIN);          // discard the first sample
  uint32_t sum = 0;
  for (int i = 0; i < BATTERY_SAMPLES; i++) sum += analogReadMilliVolts(BATTERY_PIN);
  digitalWrite(VEXT_PIN, HIGH);
  return (uint16_t) (sum / BATTERY_SAMPLES * BATTERY_DIVIDER);
}

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

// ================= LoRa =================
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

bool waitForAck(uint32_t id) {
  String expected = "ACK;" + String(id);
  LoRa.receive();                                        // continuous receive – no gaps
  unsigned long t0 = millis();
  while (millis() - t0 < ACK_TIMEOUT_MS) {
    if (pollPacket()) {
      String rx = "";
      while (LoRa.available()) rx += (char) LoRa.read();
      Serial.printf("Received: %s (RSSI %d, SNR %.1f)\n",
                    rx.c_str(), LoRa.packetRssi(), LoRa.packetSnr());
      if (rx == expected) return true;
      LoRa.receive();                                    // foreign packet – keep listening
    }
    delay(2);
  }
  return false;
}

// Sends "<type>;<id>;<battery mV>" and waits for "ACK;<id>". Every message carries the
// battery voltage, so a flap event doubles as a battery report.
bool sendMessage(const char *type) {
  String msg = String(type) + ";" + String(msgId) + ";" + String(batteryMv);
  bool acked = false;
  for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    digitalWrite(LED, HIGH);
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();                         // blocks until transmission ends (~1.3 s)
    digitalWrite(LED, LOW);
    Serial.printf("Attempt %d/%d: sent %s\n", attempt, MAX_ATTEMPTS, msg.c_str());

    if (waitForAck(msgId)) {
      Serial.println("ACK received");
      acked = true;
      break;
    }
    Serial.println("No ACK");
    if (attempt < MAX_ATTEMPTS) delay(random(500, 2000));   // random back-off before retry
  }
  if (!acked) Serial.println("Message was not acknowledged");
  msgId++;
  // the receiver has a fresh reading -> push the heartbeat a full interval away
  if (acked) nextBatteryReportS = nowS() + BATTERY_REPORT_S;
  return acked;
}

// ================= Sleep =================
void goToSleep() {
  bool stillOpen = (digitalRead(REED_PIN) == LOW);

  LoRa.sleep();                               // put radio to sleep
  digitalWrite(LED, LOW);
  digitalWrite(VEXT_PIN, HIGH);               // OLED off

  uint32_t now = nowS();
  uint32_t untilReport = (nextBatteryReportS > now) ? nextBatteryReportS - now : 1;

  if (stillOpen) {
    // flap still open – don't wake on the pin (it would wake repeatedly), check with a timer
    uint32_t s = min((uint32_t) STUCK_CHECK_S, untilReport);
    Serial.printf("Flap still open, checking again in %u s\n", s);
    esp_sleep_enable_timer_wakeup(s * 1000000ULL);
  } else {
    Serial.printf("Sleeping until the flap opens (battery report in %u s)\n", untilReport);
    esp_sleep_enable_timer_wakeup(untilReport * 1000000ULL);
    esp_sleep_enable_ext0_wakeup(REED_PIN, 0);   // wake when GPIO13 = LOW (closed)
    rtc_gpio_pullup_en(REED_PIN);                // internal pull-up active during deep sleep
    rtc_gpio_pulldown_dis(REED_PIN);
  }
  Serial.flush();
  esp_deep_sleep_start();
}

// ================= Setup / loop =================
void setup() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  rtc_gpio_deinit(REED_PIN);                  // after wake-up, restore the pin to a regular GPIO
  pinMode(REED_PIN, INPUT_PULLUP);

  Heltec.begin(false /*Display*/, true /*LoRa*/, true /*Serial*/,
               true /*PABOOST*/, BAND);
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, HIGH);               // display not needed in the mailbox
  pinMode(LED, OUTPUT);
  batteryMv = readBatteryMv();                // before any transmission
  Serial.printf("Battery: %u mV\n", batteryMv);
  configureLoRa();
  randomSeed(esp_random());

  switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT0:
      // reed switch wake-up = flap was opened, even if it has closed already
      Serial.println("Wake-up: flap opened");
      sendMessage("MAILBOX_OPEN");
      break;

    case ESP_SLEEP_WAKEUP_TIMER:
      // either the battery heartbeat is due or we are polling a flap that was left open
      if (nowS() >= nextBatteryReportS) {
        Serial.println("Wake-up: battery report");
        if (!sendMessage("BATTERY")) nextBatteryReportS = nowS() + BATTERY_RETRY_S;
      } else {
        Serial.println("Wake-up: checking whether the flap has closed");
      }
      break;

    default:
      Serial.println("Power-on / reset - service window");
      delay(SERVICE_WINDOW_MS);
      if (!sendMessage("BATTERY")) nextBatteryReportS = nowS() + BATTERY_RETRY_S;
      break;
  }

  goToSleep();
}

void loop() {
  // unused – after deep sleep the program restarts from setup()
}
