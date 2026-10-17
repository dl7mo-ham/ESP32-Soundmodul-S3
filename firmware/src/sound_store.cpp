/*
 * sound_store.cpp  -  Soundmodul Mini, siehe sound_store.h
 */

#include "sound_store.h"
#include "pins.h"
#include <SPI.h>
#include <SD.h>
#include <LittleFS.h>

static bool    sdOk      = false;
static bool    flashOk   = false;
static bool    useSd     = false;

// LittleFS darf mehr Dateien gleichzeitig offen halten als die SD-Karte
// (dort max_files=5, siehe SD.begin unten): 3 Motor-Dateien + 4 Sounds +
// Upload passen ohne Freigabe-Trick hinein.
static constexpr uint8_t FLASH_MAX_FILES = 10;

void soundStoreBegin(uint8_t wanted) {
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
#if defined(PIN_SD_CLK) && defined(PIN_SD_MISO) && defined(PIN_SD_MOSI)
    SPI.begin(PIN_SD_CLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    sdOk = SD.begin(PIN_SD_CS, SPI);
#else
    sdOk = SD.begin(PIN_SD_CS);
#endif
    if (sdOk && SD.cardType() == CARD_NONE) { SD.end(); sdOk = false; }

    // formatOnFail=true: eine leere oder beschaedigte Partition (z. B. nach
    // dem ersten Flashen ohne uploadfs) wird einmal formatiert statt
    // dauerhaft "kein Speicher" zu melden.
    flashOk = LittleFS.begin(true, "/littlefs", FLASH_MAX_FILES);

    switch (wanted) {
        case STORAGE_SD:    useSd = true;  break;
        case STORAGE_FLASH: useSd = false; break;
        default:            useSd = sdOk;  break;   // Auto
    }

    Serial.printf("Sound-Speicher: %s (Einstellung %s, SD %s, Flash %s)\n",
                  soundStoreName(),
                  wanted == STORAGE_SD ? "SD" : wanted == STORAGE_FLASH ? "Flash" : "Auto",
                  sdOk ? "ok" : "fehlt", flashOk ? "ok" : "FEHLER");
    if (!soundStoreReady()) Serial.println("FEHLER: gewaehlter Sound-Speicher nicht verfuegbar!");
}

fs::FS& soundFS() {
    if (useSd) return SD;
    return LittleFS;
}

bool        soundStoreIsSd()  { return useSd; }
bool        soundStoreReady() { return useSd ? sdOk : flashOk; }
const char* soundStoreName()  { return useSd ? "SD" : "Flash"; }
bool        sdCardMounted()   { return sdOk; }

uint64_t soundStoreTotalBytes() {
    if (!soundStoreReady()) return 0;
    return useSd ? SD.totalBytes() : (uint64_t)LittleFS.totalBytes();
}

uint64_t soundStoreUsedBytes() {
    if (!soundStoreReady()) return 0;
    return useSd ? SD.usedBytes() : (uint64_t)LittleFS.usedBytes();
}
