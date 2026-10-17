#ifndef DISPLAY_H
#define DISPLAY_H

/*
 * display.h  -  Soundmodul Mini (Heltec WiFi LoRa 32 V3)
 *
 * Ansteuerung des Onboard-OLED-Displays (SSD1306 128x64 über I2C).
 * Bietet eine informative Start-Ausgabe beim Booten und einen
 * schlanken Live-Status-Screen im Betrieb.
 */

#include <Arduino.h>

// Initialisiert I2C und das Onboard-OLED-Display (SSD1306 128x64).
// Gibt true zurück, wenn das Display gefunden wurde.
bool displayInit();

// Zeigt den Start-Bildschirm (Splash / Boot-Screen) für eine definierte Zeit an.
void displayShowBoot(const char* version,
                     const char* storage,
                     uint8_t crsfAddr,
                     const char* portBName,
                     bool wifiActive,
                     const char* ipStr);

// Aktualisiert den Live-Status-Screen im laufenden Betrieb (periodisch in loop()).
void displayUpdate(bool busOk,
                   uint8_t engineState,
                   float throttle,
                   const char* activeSoundName,
                   bool wifiActive,
                   const char* ipStr,
                   const char* portBName);

#endif // DISPLAY_H
