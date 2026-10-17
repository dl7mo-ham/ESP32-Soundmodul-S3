#ifndef PINS_H
#define PINS_H

/*
 * pins.h  -  Soundmodul Mini (Heltec WiFi LoRa 32 V3 / ESP32-S3FN8)
 *
 * Alle GPIOs an EINER Stelle. Vollstaendiges Pinout fuer das Heltec WiFi LoRa 32 V3
 * Board (ESP32-S3FN8, 8 MB Flash, kein PSRAM).
 *
 * Wichtige Onboard-Belegungen des Heltec V3 (NICHT extern verwenden!):
 *   - LoRa SX1262 (intern): GPIO 8 (NSS), 9 (SCK), 10 (MOSI), 11 (MISO),
 *                           12 (RST), 13 (BUSY), 14 (DIO1)
 *   - OLED SSD1306 (intern): GPIO 17 (SDA), 18 (SCL), 21 (RST)
 *   - Vext Power Control:    GPIO 36 (LOW = 3.3V-Stromversorgung fuer OLED/Vext EIN)
 *   - Onboard Status-LED:    GPIO 35 (LED_BUILTIN)
 *   - PRG / User-Taster:     GPIO 0 (LOW wenn gedrueckt -> Boot-Trigger fuer WLAN-AP)
 *   - Batterie-Spannung ADC: GPIO 1 (Messung), GPIO 37 (ADC Enable Control)
 *   - USB CDC (nativ):       GPIO 19 (D-), 20 (D+)
 *   - Strapping Pins:        GPIO 45 (VDD_SPI), GPIO 46 (Input-only auf ESP32-S3)
 *
 * Externe Pin-Verteilung auf den 2x18 Header-Leisten:
 *   Left Header:
 *     - Power: GND, 3V3, 5V (fuer MAX98357A & Empfaenger)
 *     - GPIO 2: I2S BCLK
 *     - GPIO 3: I2S LRC (WS)
 *     - GPIO 4: I2S DOUT (DIN)
 *     - GPIO 5: Port A CRSF RX (Serial2)
 *     - GPIO 6: Port A CRSF TX (Serial2)
 *     - GPIO 7: Frei / Aux
 *   Right Header:
 *     - GPIO 0:  WLAN-Bootpin (PRG-Taster auf dem Board, oder externer Schalter)
 *     - GPIO 35: Onboard Status-LED
 *     - GPIO 36: Vext Power Control (OLED / externe Vext-Schiene)
 *     - GPIO 41: Port B RX (Serial1)
 *     - GPIO 42: Port B TX (Serial1)
 *     - GPIO 47: Port C GPS RX (SoftwareSerial / Meshtastic Standard)
 *     - GPIO 48: Port C GPS TX (SoftwareSerial / Meshtastic Standard)
 *     - GPIO 38: SD CS (optional)
 *     - GPIO 39: SD MOSI (optional)
 *     - GPIO 40: SD CLK (optional)
 *     - GPIO 46: SD MISO (optional, GPIO 46 ist Input-only auf S3 -> ideal fuer MISO)
 */

// ── Onboard-Hardware ──────────────────────────────────────────────────
#define PIN_WIFI        0    // Onboard PRG-Taster (LOW beim Einschalten = AP-Modus)
#define PIN_VEXT        36   // Vext Power Switch (LOW = aktiv, versorgt OLED & Vext)
#define PIN_LED         35   // Onboard Status-LED
#define PIN_OLED_SDA    17   // Onboard OLED SDA (I2C)
#define PIN_OLED_SCL    18   // Onboard OLED SCL (I2C)
#define PIN_OLED_RST    21   // Onboard OLED Reset (Active LOW)

// ── Port A: CRSF (Hardware-UART, Serial2) ────────────────────────────
#define PIN_CRSF_RX     5
#define PIN_CRSF_TX     6

// ── I2S-Verstaerker (MAX98357A) ──────────────────────────────────────
#define PIN_I2S_BCLK    2
#define PIN_I2S_LRC     3
#define PIN_I2S_DOUT    4

// ── Port B: zweiter Hardware-UART (Serial1) ──────────────────────────
// S.Port-Rolle: TX -[1N4148 Anode->Kathode]-> S.Port-Signal <- RX
#define PIN_PORTB_RX    41
#define PIN_PORTB_TX    42

// ── Port C: GPS (SoftwareSerial / Meshtastic Standard) ────────────────
#define PIN_GPS_RX      47   // GPS-TX -> ESP32
#define PIN_GPS_TX      48   // ESP32 -> GPS-RX

// ── SD-Karte (optional, dedizierter SPI-Bus) ──────────────────────────
#define PIN_SD_CS       38
#define PIN_SD_MOSI     39
#define PIN_SD_CLK      40
#define PIN_SD_MISO     46   // S3 Input-only Pin

#endif // PINS_H
