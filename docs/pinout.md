# Pinout & Hardware-Dokumentation: Heltec WiFi LoRa 32 V3

Dieses Dokument beschreibt die Hardware-Belegung und den Anschluss externer Komponenten für das **Heltec WiFi LoRa 32 V3** (ESP32-S3FN8, 8 MB Flash, kein PSRAM).

---

## 1. Übersicht & Board-Spezifikationen

- **MCU:** ESP32-S3 (Dual-Core 32-bit Xtensa LX7 bis 240 MHz)
- **Flash:** 8 MB Quad-SPI NOR Flash
- **RAM:** 512 KB internes SRAM (kein externes PSRAM)
- **Konsole:** Nativer USB-C CDC Anschluss (`-DARDUINO_USB_CDC_ON_BOOT=1`)
- **Display:** Onboard 0,96" SSD1306 I2C OLED (128×64)
- **LoRa:** Semtech SX1262 (intern über SPI angebunden)
- **Power Switch (Vext):** GPIO 36 schaltet 3,3V für OLED und externe Sensoren (Active LOW)

---

## 2. Onboard-Hardware (Interne Pins – NICHT extern belegen!)

| Peripherie | Signal | GPIO | Hinweis |
| :--- | :--- | :--- | :--- |
| **Vext Power Switch** | Vext_Ctrl | **GPIO 36** | In `setup()` auf LOW gesetzt, um 3,3V-Schiene & OLED zu versorgen |
| **Status-LED** | LED | **GPIO 35** | Onboard weiße LED |
| **PRG-Taster** | BOOT / PRG | **GPIO 0** | Beim Einschalten gedrückt halten = **WLAN-AP-Modus** |
| **OLED Display** | SDA | **GPIO 17** | I2C Datenleitung |
| **OLED Display** | SCL | **GPIO 18** | I2C Taktleitung |
| **OLED Display** | RST | **GPIO 21** | Reset für Display |
| **LoRa SX1262** | NSS / CS | **GPIO 8** | Interner SPI Chip-Select |
| **LoRa SX1262** | SCK | **GPIO 9** | Interner SPI Takt |
| **LoRa SX1262** | MOSI | **GPIO 10** | Interner SPI MOSI |
| **LoRa SX1262** | MISO | **GPIO 11** | Interner SPI MISO |
| **LoRa SX1262** | RST | **GPIO 12** | LoRa Reset |
| **LoRa SX1262** | BUSY | **GPIO 13** | LoRa Busy Signal |
| **LoRa SX1262** | DIO1 | **GPIO 14** | LoRa Interrupt |
| **Batterie-Messung** | VBAT Read | **GPIO 1** | ADC-Kanal über internen Spannungsteiler |
| **Batterie ADC Ctrl**| ADC Ctrl | **GPIO 37** | Schaltet Teiler zur Messung zu |
| **Nativer USB** | D- / D+ | **GPIO 19 / 20** | Konsole & Flashen |

---

## 3. Externe Pinbelegung (Header-Leisten)

Das Soundmodul nutzt die freien Pins der beiden 18-poligen Stiftleisten:

### Linke Header-Leiste (neben GND / 3V3 / 5V):

| Pin-Nr. | Pin-Name / GPIO | Funktion im Soundmodul | Anschluss |
| :--- | :--- | :--- | :--- |
| 1 | **GND** | Masse | GND für Verstärker, Empfänger |
| 2 | **5V** | 5V Versorgung | 5V Eingang oder Ausgang vom USB |
| 3 | **3V3** | 3,3V geregelt | 3,3V Versorgung |
| 4 | GPIO 1 | *Reserviert (VBAT)* | Onboard Batterie-Messung |
| 5 | **GPIO 2** | **I2S BCLK** | Bit-Clock an MAX98357A |
| 6 | **GPIO 3** | **I2S LRC (WS)** | Word-Select an MAX98357A |
| 7 | **GPIO 4** | **I2S DOUT (DIN)**| Data-In an MAX98357A |
| 8 | **GPIO 5** | **Port A: CRSF RX** | Empfängt Telemetrie/CRSF vom RC-Empfänger |
| 9 | **GPIO 6** | **Port A: CRSF TX** | Sendet CRSF-Telemetrie zum RC-Empfänger |
| 10 | **GPIO 7** | *Frei / Aux* | Für Erweiterungen verfügbar |
| 11..17 | GPIO 8..14 | *Tabu (LoRa SPI)* | Intern verdrahtet mit SX1262 |
| 18 | GPIO 21 | *Tabu (OLED RST)* | Intern verdrahtet mit Display |

### Rechte Header-Leiste:

| Pin-Nr. | Pin-Name / GPIO | Funktion im Soundmodul | Anschluss |
| :--- | :--- | :--- | :--- |
| 1 | **GND** | Masse | Zusätzlicher Masseanschluss |
| 2 | **5V** | 5V | 5V Schiene |
| 3 | **Vext** | Geschaltete 3,3V | Über GPIO 36 gesteuerte 3,3V-Versorgung |
| 4 | GPIO 44 | UART0 RX | USB Serial / Frei |
| 5 | GPIO 43 | UART0 TX | USB Serial / Frei |
| 6 | **GPIO 0** | **WLAN-Bootpin** | Onboard PRG-Taster (oder externer Taster gegen GND) |
| 7 | GPIO 36 | *Vext Ctrl* | Intern (wird automatisch auf LOW gesetzt) |
| 8 | **GPIO 35** | **Status-LED** | Onboard LED |
| 9 | **GPIO 38** | **SD CS** | Chip-Select für optionale MicroSD-Karte |
| 10 | **GPIO 39** | **SD MOSI** | SPI MOSI für MicroSD-Karte |
| 11 | **GPIO 40** | **SD CLK** | SPI Takt für MicroSD-Karte |
| 12 | **GPIO 46** | **SD MISO** | SPI MISO (S3 Input-only Pin – perfekt für MISO) |
| 13 | **GPIO 41** | **Port B: RX** | Zweiter Hardware-UART (Serial1) RX (z. B. S.Port) |
| 14 | **GPIO 42** | **Port B: TX** | Zweiter Hardware-UART (Serial1) TX |
| 15 | **GPIO 47** | **Port C: GPS RX** | GPS-TX an ESP32 (Meshtastic Standard) |
| 16 | **GPIO 48** | **Port C: GPS TX** | ESP32 an GPS-RX (Meshtastic Standard) |
| 17 | GPIO 45 | *Frei / Aux* | Strapping Pin / Frei verfügbar |
| 18 | GPIO 37 | *ADC Ctrl* | Intern Batterie-Spannungsteiler-Steuerung |

---

## 4. Verdrahtung externer Module

### A. I2S Verstärker (MAX98357A)
```
MAX98357A          Heltec V3
───────────────────────────────
VIN        ───►    5V
GND        ───►    GND
BCLK       ───►    GPIO 2
LRC (WS)   ───►    GPIO 3
DIN        ───►    GPIO 4
GAIN       ───►    GND (12 dB) oder offen (9 dB)
SD_MODE    ───►    offen (Mono-Mix)
```

### B. RC-Empfänger (Port A, CRSF)
```
CRSF-Empfänger     Heltec V3
───────────────────────────────
5V         ───►    5V
GND        ───►    GND
CH1 (TX)   ───►    GPIO 5 (CRSF RX)
CH2 (RX)   ───►    GPIO 6 (CRSF TX)
```

### C. GPS-Modul (Port C, NMEA)
```
GPS-Modul          Heltec V3
───────────────────────────────
VCC        ───►    3V3 oder 5V
GND        ───►    GND
TX         ───►    GPIO 47 (GPS RX)
RX         ───►    GPIO 48 (GPS TX)
```

### D. Port B (z. B. FrSky S.Port LiPo-Telemetrie)
```
S.Port             Heltec V3
───────────────────────────────
GND        ───►    GND
Signal     ───►    GPIO 41 (RX)
           └───◄── [1N4148 Kathode | Anode] ─── GPIO 42 (TX)
```

### E. MicroSD-Karten-Adapter (optional)
```
MicroSD Modul      Heltec V3
───────────────────────────────
VCC        ───►    3V3 (oder 5V je nach Adapter)
GND        ───►    GND
CS         ───►    GPIO 38
MOSI       ───►    GPIO 39
CLK / SCK  ───►    GPIO 40
MISO       ───►    GPIO 46
```

---

## 5. Flash-Aufteilung (8 MB Flash)

Mit dem 8 MB Flash des Heltec V3 (`partitions_8mb.csv`) steht deutlich mehr Platz zur Verfügung:
- **OTA Partition 0:** 2,5 MB
- **OTA Partition 1:** 2,5 MB
- **LittleFS (Sounds):** 2,875 MB (entspricht ca. 65 Sekunden Audio bei 22,05 kHz 16-Bit Mono)
