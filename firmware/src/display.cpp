/*
 * display.cpp  -  Soundmodul Mini (Heltec WiFi LoRa 32 V3)
 *
 * Ansteuerung des Onboard-OLED-Displays (SSD1306 128x64 über I2C).
 */

#include "display.h"
#include "pins.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C

static Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
static bool s_hasDisplay = false;
static unsigned long s_bootScreenUntil = 0;

bool displayInit() {
#ifdef PIN_VEXT
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, LOW); // Active LOW: Stromversorgung für OLED einschalten
    delay(30);
#endif

#ifdef PIN_OLED_RST
    pinMode(PIN_OLED_RST, OUTPUT);
    digitalWrite(PIN_OLED_RST, LOW);
    delay(20);
    digitalWrite(PIN_OLED_RST, HIGH);
    delay(50);
#endif

#if defined(PIN_OLED_SDA) && defined(PIN_OLED_SCL)
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    Wire.setClock(400000);
#else
    Wire.begin();
#endif

    // periphBegin = false verhindert, dass begin() Wire mit den falschen Default-Pins reinitialisiert
    if (!oledDisplay.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false)) {
        Serial.println("[Display] Kein SSD1306 OLED gefunden (I2C 0x3C).");
        s_hasDisplay = false;
        return false;
    }

    s_hasDisplay = true;
    oledDisplay.clearDisplay();
    oledDisplay.setTextColor(SSD1306_WHITE);
    oledDisplay.setTextSize(1);
    oledDisplay.display();

    Serial.println("[Display] Onboard OLED (SSD1306 128x64) initialisiert.");
    return true;
}

void displayShowBoot(const char* version,
                     const char* storage,
                     uint8_t crsfAddr,
                     const char* portBName,
                     bool wifiActive,
                     const char* ipStr) {
    if (!s_hasDisplay) return;

    (void)portBName; // Für spätere Erweiterungen reserviert

    oledDisplay.clearDisplay();

    // Invertierte Kopfzeile
    oledDisplay.fillRect(0, 0, SCREEN_WIDTH, 12, SSD1306_WHITE);
    oledDisplay.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    oledDisplay.setTextSize(1);
    oledDisplay.setCursor(19, 2);
    oledDisplay.print("SOUNDMODUL MINI");

    // Informationszeilen
    oledDisplay.setTextColor(SSD1306_WHITE);

    oledDisplay.setCursor(0, 15);
    oledDisplay.print("Board:  Heltec V3");

    oledDisplay.setCursor(0, 25);
    oledDisplay.printf("FW:     v%s", version ? version : "?.??");

    oledDisplay.setCursor(0, 35);
    oledDisplay.printf("CRSF:   Port A (0x%02X)", crsfAddr);

    oledDisplay.setCursor(0, 45);
    oledDisplay.printf("Speich: %s", storage ? storage : "Flash");

    oledDisplay.setCursor(0, 55);
    if (wifiActive && ipStr && ipStr[0] != '\0') {
        oledDisplay.printf("AP:     %s", ipStr);
    } else if (wifiActive) {
        oledDisplay.print("WLAN:   AP aktiv");
    } else {
        oledDisplay.print("WLAN:   Aus");
    }

    oledDisplay.display();

    // Start-Ausgabe bleibt für 3,5 Sekunden sichtbar
    s_bootScreenUntil = millis() + 3500;
}

void displayUpdate(bool busOk,
                   uint8_t engineState,
                   float throttle,
                   const char* activeSoundName,
                   bool wifiActive,
                   const char* ipStr,
                   const char* portBName) {
    if (!s_hasDisplay) return;
    if (millis() < s_bootScreenUntil) return; // Noch in der Boot-Phase

    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < 250) return; // 4 Hz Refresh
    lastUpdate = millis();

    oledDisplay.clearDisplay();

    // Kopfzeile: Titel + RC-Status
    oledDisplay.setTextColor(SSD1306_WHITE);
    oledDisplay.setTextSize(1);
    oledDisplay.setCursor(0, 1);
    oledDisplay.print("SOUNDMODUL");

    oledDisplay.setCursor(80, 1);
    if (busOk) {
        oledDisplay.print("[RC:OK]");
    } else {
        oledDisplay.print("[RC:--]");
    }
    oledDisplay.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);

    // Zeile 1: Motor-Status & Gas
    const char* stStr = "AUS";
    if (engineState == 1)      stStr = "START";
    else if (engineState == 2) stStr = "RUN";
    else if (engineState == 3) stStr = "STOP";

    oledDisplay.setCursor(0, 14);
    oledDisplay.printf("Motor: %-5s %3d%%", stStr, (int)throttle);

    // Zeile 2: Sound
    oledDisplay.setCursor(0, 24);
    oledDisplay.printf("Sound: %s", (activeSoundName && activeSoundName[0]) ? activeSoundName : "--");

    // Zeile 3: Port B
    oledDisplay.setCursor(0, 34);
    oledDisplay.printf("Port B: %s", portBName ? portBName : "Aus");

    // Zeile 4: WLAN
    oledDisplay.setCursor(0, 44);
    if (wifiActive && ipStr && ipStr[0] != '\0') {
        oledDisplay.printf("AP:    %s", ipStr);
    } else {
        oledDisplay.print("WLAN:  Aus");
    }

    // Zeile 5: Gas-Balken (0-100%) + Heartbeat-Pixel
    oledDisplay.drawRect(0, 55, 116, 7, SSD1306_WHITE);
    int barWidth = (int)((throttle / 100.0f) * 112.0f);
    if (barWidth > 112) barWidth = 112;
    if (barWidth > 0) {
        oledDisplay.fillRect(2, 57, barWidth, 3, SSD1306_WHITE);
    }

    // Heartbeat-Punkt rechts unten (blinkt mit 1 Hz)
    if ((millis() / 500) % 2 == 0) {
        oledDisplay.fillCircle(123, 58, 2, SSD1306_WHITE);
    }

    oledDisplay.display();
}
