/*
   Soundmodul Mini (ESP32-S3) - Soundmodul-Firmware fuer RC-Modelle
   PiperPilot

   Abgeleitet von ESP32-RC-Sound v7.19 (github.com/mnorden81/
   mnorden81-ESP32-Sound-bidirectional_code), reduziert auf:
     - nur CRSF als RC-Bus (Port A), busbasiert: kein SBUS, keine PWM-/
       GPIO-Eingaenge, keine Hardware-Varianten V1/V2/V3
     - Motorsound (start/loop/shut) + 4 Zusatzsounds
   Unveraendert uebernommen: Motorlogik (Gasrampe, Standgas-Verzoegerung,
   Tonhoehe folgt Gas), Port B (zweiter UART, Rolle waehlbar), Port C
   (GPS), Weboberflaeche mit Sound-Upload und OTA, CRSF-/Lua-Menue,
   WLAN-Failsafe.

   ── Quellen (Motor EIN und Sound 1-4) ─────────────────────────────────
   BUS Kanal Low/High (1-16), Einkanal (1-8, Schalter an der eigenen
   WM-Adresse), MKan (1-16, zwei frei waehlbare CRSF-Gruppenadressen),
   Dauerbetrieb, Deaktiviert.
   Rohwerte wie in V7 (0-15, 20-35, 70-77, 80-95, 200, 999), damit
   Konfigurationsdateien austauschbar bleiben.

   ── Pins ──────────────────────────────────────────────────────────────
   Siehe pins.h (vorlaeufig, wird in Schritt 5 am Board geprueft).

   ── Das Port-Modell ───────────────────────────────────────────────────
   Port A = CRSF (Serial2), fest.
   Port B = zweiter Hardware-UART (Serial1), Rolle waehlbar in Web UND
            CRSF/Lua-Menue: Aus, CRSF (zweite Instanz), S.Port-Master
            (LiPo-Telemetrie) oder Raw-UART, je mit eigener Baudrate.
            Welches Modul fuer welche Rolle zustaendig ist, kommt aus der
            Registry in port_function.h/.cpp.
   Port C = GPS (SoftwareSerial) fuer CRSF-GPS-Telemetrie, An/Aus.

   ── WLAN & Weboberflaeche ─────────────────────────────────────────────
   Der Access Point laesst sich einschalten ueber:
     - "WLAN dauerhaft an" (Default AN)
     - PIN_WIFI beim Einschalten auf GND
     - Schalter im Web-Tab "WiFi" bzw. Lua-Feld "WLAN jetzt" (sofort,
       nicht persistiert)
     - Auto-Failsafe: schaltet ein, wenn laenger als die eingestellte Zeit
       kein gueltiges CRSF-Signal anliegt; schaltet NIE automatisch aus
       (sonst bricht eine laufende Konfiguration/OTA ab).

   ── CRSF-/Lua-Parametermenue ──────────────────────────────────────────
   CRSF_PARAM_COUNT Feld-IDs, lueckenlos 1..51, Liste direkt beim
   Parameter-System weiter unten. CRSF-Busadresse 0xC0+WM-Adresse und
   Telemetrie-Zeitslot nach dem Wilhelm-Meier-Schema.

   ── Module ────────────────────────────────────────────────────────────
   config.cpp/.h          Persistente Konfiguration (NVS)
   WebServerManager.cpp   Weboberflaeche, REST-API, OTA-Update
   XT_I2S_Audio.cpp/.h    WAV-Wiedergabe ueber I2S
   sound_store.cpp/.h     Sound-Speicher Flash (LittleFS) / SD / Auto
   CRSF_ESP32/            Gemeinsame CRSF-Bibliothek (identische Kopie
                          wie im Soundmodul V7 und im Multiswitch)
   sport_lipo.cpp/.h      S.Port-Telemetrie (Port B)
   gps_speed.cpp/.h       GPS (Port C)
   port_function.cpp/.h   Port-B-Rollen-Registry
   pins.h                 Pinbelegung
*/

#include <Arduino.h>
#include "XT_I2S_Audio.h"
#include <WiFi.h>
#include "CRSF_ESP32/crsf_esp32.h"
#include "config.h"
#include "pins.h"
#include "WebServerManager.h"
#include "sport_lipo.h"
#include "gps_speed.h"
#include "port_function.h"
#include "sound_store.h"
#include "display.h"

uint16_t Version = 1;  // Firmware-Version fuer Web-/Lua-Anzeige: major*100+minor (1 -> "0.01")
char versionString[6];
// Port C (GPS) kann beim Mini nicht mit Port A kollidieren (CRSF-only). Die
// Variable bleibt, weil die Weboberflaeche sie im Debug-JSON meldet.
bool gpsPinConflict = false;

// ── Zustand ───────────────────────────────────────────────────────────
bool Sound_on[RCSOUND_NUM_SLOTS]     = {};
bool Sound_play[RCSOUND_NUM_SLOTS]   = {};
bool Sound_on_web[RCSOUND_NUM_SLOTS] = {};

bool Sound_on_Motor       = false;
bool Sound_on_Motor_state = false;
bool engine_break         = false;
unsigned long shutdown_timer = 0;

enum Engine_State : uint8_t { OFF, STARTING, RUNNING, STOPPING };
volatile uint8_t engine_State = OFF;

// Boot-Snapshot: Port-B-Rolle/Baudrate und Port-C(GPS)-Zustand werden
// einmalig in setup() aus config uebernommen. Aendern ueber Web/Lua schreibt
// sofort in config, wirkt aber erst nach einem Neustart - die UARTs sind
// dann schon initialisiert und werden im Betrieb nicht umgeschaltet.
int      PortB_Mode_boot        = PORTB_OFF;
uint32_t PortB_Baud_boot        = PORTB_BAUD_DEFAULT_CRSF;
int      PortC_GPS_Enabled_boot = 0;
// Port A (CRSF) belegt Serial2, Port B bekommt Serial1.
static HardwareSerial* const PORTA_SERIAL = &Serial2;
static HardwareSerial* const PORTB_SERIAL = &Serial1;

uint16_t einkanal_Data = 0;   // 1 Bit/Schalter - Quelltyp "Einkanal" 70-77

// Je ein 8-Bit-Wert (1 Bit/Schalter) aus den 2 CRSF-Gruppenadressen sowie
// deren Kombination zu einem 16-Bit-Flachwert - Quelltyp "MKan" 80-95.
uint8_t  einkanal_Gruppe[RCSOUND_NUM_GROUPS] = {};
uint32_t einkanal_Flat16          = 0;

// ── Throttle / Rampe ─────────────────────────────────────────────────
unsigned long previousTime_ramp = 0;
float last_throttle = 0.0f;
float throttle      = 0.0f;

char ssid[32];
char password[64];
unsigned long currentTime = 0;

bool BUS_OK = false;

// ── CRSF ──────────────────────────────────────────────────────────────
CRSF crsf;       // Port A - der Haupt-RC-Bus
// Zweite, unabhaengige CRSF-Instanz fuer Port B (Rolle PORTB_CRSF).
CRSF crsfPortB;
#define MWset       0x01
#define MWprop      0x02
#define MWset4      0x07
#define MWset4m     0x09
#define Multiswitch 0xA1

static constexpr unsigned long CRSF_TIMEOUT_MS = 2000;
static unsigned long lastCrsfPacket = 0;

static void checkCrsfTimeout() {
    if (millis() - lastCrsfPacket > CRSF_TIMEOUT_MS) BUS_OK = false;
}

uint16_t channel_output[16] = {};

// ── I2S Audio ─────────────────────────────────────────────────────────
XT_I2S_Class I2SAudio(PIN_I2S_LRC, PIN_I2S_BCLK, PIN_I2S_DOUT, I2S_NUM_0);

XT_Wav_Class Sound_loop ("/loop.wav");
XT_Wav_Class Sound_shut ("/shut.wav");
XT_Wav_Class Sound_start("/start.wav");
XT_Wav_Class Sound1("/sound1.wav");   XT_Wav_Class Sound2("/sound2.wav");
XT_Wav_Class Sound3("/sound3.wav");   XT_Wav_Class Sound4("/sound4.wav");

XT_Wav_Class* Sounds[RCSOUND_NUM_SLOTS] = {
    nullptr, &Sound1, &Sound2, &Sound3, &Sound4
};

// ── Vorwaertsdeklarationen ────────────────────────────────────────────
void Config(); void LoadFiles();
void AllocateSoundBuffers(); // reserviert die Sample-Puffer erst in setup() NACH dem WLAN-AP-Start
void einkanalFunctionCRSF(); uint8_t compressSwitches(uint16_t state);
static void updateGroupFromAddress(uint8_t addr, uint8_t bits);
float ramp_throttle(float t); float floatMap(float x,float a,float b,float c,float d);
void handleSound(uint8_t idx, XT_Wav_Class* snd);
static void crsfSendParam(uint8_t idx);
static void crsfWriteParam(uint8_t idx, uint8_t val);

// ======== CRSF-Parametersystem ==========================================
//
//   0: Root  1: Info Version
//   2: Folder Motor {3..14}
//    3: Sel MotorMode  4: Sel EINModus  5: Sel QuelleEIN  6: U8 KanalEIN
//    7: Sel QuelleSpd  8: U8 KanalSpd   9: U8 Vol
//   10: U8 Drzmin  11: U8 Drzmax/2  12: U8 Standgas  13: U8 Rampe  14: U8 Totband
//
//   Sound s(1-4): fi=15+(s-1)*6  -> 15, 21, 27, 33 (endet bei 38)
//    fi+0:Folder fi+1:Sel Quelle fi+2:U8 Kanal fi+3:U8 Vol fi+4:Sel Mode fi+5:Sel Test
//
//  39: Folder Einstellungen {40..49}
//   40:U8 Modul Adresse
//   41:U8 Gruppenadr.1  42:U8 Gruppenadr.2  (255=aus)
//   43:Sel Port B Modus  44:Sel Port B Baudrate  45:Sel Port C (GPS)
//   46:Sel WLAN jetzt (sofort, nicht persistiert)
//   47:Sel Auto-WLAN bei Signalverlust  48:U8 Auto-WLAN Timeout (s)
//   49:Sel WLAN dauerhaft an
//   50:Sel Sound-Speicher (Auto/Flash/SD, wirkt nach Neustart)
//  51: Info Geraetename
static constexpr uint8_t SOUND_FIRST_FI   = 15;
static constexpr uint8_t FIELDS_PER_SOUND = 6;
static constexpr uint8_t SOUND_LAST_FI    = SOUND_FIRST_FI + RCSOUND_NUM_SOUNDS*FIELDS_PER_SOUND - 1;
static constexpr uint8_t SETTINGS_FI      = SOUND_LAST_FI + 1;
static constexpr uint8_t DEVNAME_FI       = SETTINGS_FI + 12;
static constexpr uint8_t CRSF_PARAM_COUNT = DEVNAME_FI;
// Die Folder-Kinderlisten unten sind Literale (std::initializer_list) - bei
// einer Aenderung der Sound-Anzahl muessen sie von Hand nachgezogen werden.
static_assert(SOUND_LAST_FI == 38 && SETTINGS_FI == 39 && CRSF_PARAM_COUNT == 51,
              "CRSF-Feld-IDs passen nicht mehr zu den Folder-Listen in crsfSendParam()");

// ── CRSF-Geraeteadresse + Ping-Slot aus der WM-Adresse (Wilhelm-Meier-Schema) ──
// Adresse = 0xC0 + WM-Adresse, Slot = WM-Adresse * 2. Die Ping-Antwort wird
// erst im eigenen Zeit-Slot gesendet, damit sich mehrere Module nicht
// ueberlappen.
#define CRSF_SLOT_MS 5
static inline uint8_t  crsfAddrFromWM()  { return 0xC0 + (uint8_t)constrain(config.modul_adress, 0, 15); }
static inline uint16_t crsfSlotDelayMs() { return (uint16_t)((uint8_t)constrain(config.modul_adress, 0, 15) * 2) * CRSF_SLOT_MS; }

// ── Typ-Mapping Quelle Motor/Sound EIN ───────────────────────────────
// Typ: 0=BUS-L(0-15) 1=BUS-H(20-35) 5=Einkanal(70-77) 6=MKan(80-95)
//      7=Dauerbetrieb(200) 8=Deaktiviert(999)
// (Typen 2-4 waren in V7 PWM/Pin und entfallen.)
static uint8_t einTyp(int c) {
    if(c==999)return 8; if(c==200)return 7;
    if(c>=80&&c<=95)return 6; if(c>=70&&c<=77)return 5;
    if(c>=20&&c<=35)return 1;
    if(c>=0&&c<=15)return 0; return 8;
}

// Prueft, ob ein "Quelle"-Rohwert gueltig ist. Bewusst NICHT static:
// config.cpp (loadConfig()) und WebServerManager.cpp (handleApiSound(),
// auch beim Konfigurations-Import) nutzen dieselbe Definition. Ein
// ungueltiger Rohwert (z.B. PWM/Pin aus einer V7-Konfiguration) wird auf
// 999 (Deaktiviert) zurueckgesetzt.
bool isValidEinSource(int c) {
    return c==999 || c==200 ||
           (c>=80&&c<=95) || (c>=70&&c<=77) || (c>=20&&c<=35) ||
           (c>=0&&c<=15);
}
static uint8_t einNr(int c) {
    if(c>=80&&c<=95)return c-80+1; if(c>=70&&c<=77)return c-70+1;
    if(c>=20&&c<=35)return c-20+1;
    if(c>=0&&c<=15)return c+1; return 1;
}
static int setEin(uint8_t t, uint8_t nr) {
    uint8_t i=(nr>0)?nr-1:0;
    if(t==0)return min(i,(uint8_t)15);
    if(t==1)return 20+min(i,(uint8_t)15);
    if(t==5)return 70+min(i,(uint8_t)7);
    if(t==6)return 80+min(i,(uint8_t)15);
    if(t==7)return 200; return 999;
}
static uint8_t einMaxNr(uint8_t t) {
    if(t==0||t==1)return 16;
    if(t==5)return 8; if(t==6)return 16; return 1;
}

// ── Quelle Speed: 0-15 = CRSF-Kanal, 999 = Deaktiviert ────────────────
static bool spdActive(int c) { return c>=0&&c<=15; }

static bool testSoundActive[RCSOUND_NUM_SLOTS]={};

// ── Port B: Baudraten-Liste fuer das CRSF/Lua-Menue ───────────────────
// TEXT_SELECTION uebertraegt 1 Byte - eine frei eintippbare Baudrate bis
// 420000 passt da nicht hinein. Im Lua-Menue daher eine kuratierte Auswahl;
// im Web-UI bleibt die Baudrate frei eingebbar.
static const char* PORTB_BAUD_LIST_STR = "9600;19200;38400;57600;115200;230400;420000";
static const uint32_t portBBaudList[] = {9600,19200,38400,57600,115200,230400,420000};
static constexpr uint8_t PORTB_BAUD_LIST_COUNT = 7;
static uint8_t portBBaudToIndex(uint32_t baud) {
    for (uint8_t i=0;i<PORTB_BAUD_LIST_COUNT;i++) if (portBBaudList[i]==baud) return i;
    // Frei ueber's Web eingegebener Wert: naechstgelegenen Eintrag ANZEIGEN,
    // ohne den gespeicherten Wert zu veraendern.
    uint8_t best=0; uint32_t bestDiff=0xFFFFFFFF;
    for (uint8_t i=0;i<PORTB_BAUD_LIST_COUNT;i++) {
        uint32_t diff=(baud>portBBaudList[i])?(baud-portBBaudList[i]):(portBBaudList[i]-baud);
        if (diff<bestDiff){bestDiff=diff;best=i;}
    }
    return best;
}

// ── CRSF Quelle EIN: Auswahl L/H/EK/MKan/Dauer/Aus ────────────────────
static const char* EIN_SEL_STR = "L;H;EK;MKan;Dauer;Aus";
static constexpr uint8_t EIN_SEL_MAX = 5;
static uint8_t einSelVal(int cfg) {
    uint8_t t = einTyp(cfg);
    // Auswahl-Index: 0=L,1=H,2=EK,3=MKan,4=Dauer,5=Aus
    if (t==5) return 2; if (t==6) return 3; if (t==7) return 4; if (t>=8) return 5;
    return t; // 0,1
}
static int einSelWrite(uint8_t val, int oldCfg) {
    static const uint8_t selToTyp[] = {0,1,5,6,7,8};
    uint8_t t = (val < 6) ? selToTyp[val] : 8;
    return setEin(t, einNr(oldCfg));
}

// ======== CRSF: Parameter senden =====================================
static void crsfSendParam(uint8_t idx) {
    char buf[48];

    if(idx==0) {
        crsf.send_param_response_CRSF_FOLDER(0,0,"",{1,2,15,21,27,33,39,51});
    }
    else if(idx==1) {
        snprintf(buf,sizeof(buf),"%s Mini S3",versionString);
        crsf.send_param_response_CRSF_INFO(1,0,"Version",buf);
    }

    // ── Motor (2..14) ─────────────────────────────────────────────────
    else if(idx==2) crsf.send_param_response_CRSF_FOLDER(2,0,"Motor",
        {3,4,5,6,7,8,9,10,11,12,13,14});
    else if(idx==3) crsf.send_param_response_CRSF_TEXT_SELECTION(3,2,
        "Motor Mode","Eine Richtung;Zwei Richtungen",
        (uint8_t)constrain(config.throttle_mode,0,1),0,1);
    else if(idx==4) crsf.send_param_response_CRSF_TEXT_SELECTION(4,2,
        "Motor EIN Modus","Normal;Toggle",
        (uint8_t)constrain(config.engine_on_toggle,0,1),0,1);
    else if(idx==5) crsf.send_param_response_CRSF_TEXT_SELECTION(5,2,
        "Quelle Motor", EIN_SEL_STR,
        einSelVal(config.Source_Start_Sound[0]),0,EIN_SEL_MAX);
    else if(idx==6) {
        uint8_t t=einTyp(config.Source_Start_Sound[0]);
        if(t==7||t==8)
            crsf.send_param_response_CRSF_INFO(6,2,"Kanal EIN",
                (t==7)?"Dauerbetrieb":"Deaktiviert");
        else crsf.send_param_response_CRSF_UINT8(6,2,"Kanal Nr EIN",
            einNr(config.Source_Start_Sound[0]),1,einMaxNr(t),"");
    }
    else if(idx==7) crsf.send_param_response_CRSF_TEXT_SELECTION(7,2,
        "Quelle Speed","BUS Kanal;Deaktiviert",
        spdActive(config.Source_Speed_Sound_0)?0:1,0,1);
    else if(idx==8) {
        if(!spdActive(config.Source_Speed_Sound_0))
            crsf.send_param_response_CRSF_INFO(8,2,"Kanal Speed","Deaktiviert");
        else crsf.send_param_response_CRSF_UINT8(8,2,"Kanal Nr Speed",
            config.Source_Speed_Sound_0+1,1,16,"");
    }
    else if(idx==9)  crsf.send_param_response_CRSF_UINT8(9, 2,"Volumen Motor",
        (uint8_t)constrain(config.Volumen_Sound[0],0,200),0,200,"");
    else if(idx==10) crsf.send_param_response_CRSF_UINT8(10,2,"Drehzahl min %",
        (uint8_t)constrain(config.Min_Speed_Sound_0,0,200),0,200,"%");
    else if(idx==11) crsf.send_param_response_CRSF_UINT8(11,2,"Drehzahl max /2",
        (uint8_t)constrain(config.Max_Speed_Sound_0/2,50,255),50,255,"%");
    else if(idx==12) crsf.send_param_response_CRSF_UINT8(12,2,"Motor aus Standgas",
        (uint8_t)constrain(config.shutdowndelay,0,60),0,60,"s");
    else if(idx==13) crsf.send_param_response_CRSF_UINT8(13,2,"Motor Rampe %/s",
        (uint8_t)constrain(config.throttle_ramp,0,50),0,50,"");
    else if(idx==14) crsf.send_param_response_CRSF_UINT8(14,2,"Standgas Totband",
        (uint8_t)constrain(config.throttle_dead_band,0,50),0,50,"%");

    // ── Sound 1-4: fi=15+(s-1)*6 ──────────────────────────────────────
    else if(idx>=SOUND_FIRST_FI&&idx<=SOUND_LAST_FI) {
        uint8_t s=(idx-SOUND_FIRST_FI)/FIELDS_PER_SOUND+1, sub=(idx-SOUND_FIRST_FI)%FIELDS_PER_SOUND;
        uint8_t fi=SOUND_FIRST_FI+(s-1)*FIELDS_PER_SOUND;
        switch(sub) {
        case 0:
            snprintf(buf,sizeof(buf),"Sound %d",s);
            crsf.send_param_response_CRSF_FOLDER(fi,0,buf,
                {(uint8_t)(fi+1),(uint8_t)(fi+2),
                 (uint8_t)(fi+3),(uint8_t)(fi+4),(uint8_t)(fi+5)});
            break;
        case 1:
            crsf.send_param_response_CRSF_TEXT_SELECTION(fi+1,fi,
                "Quelle",EIN_SEL_STR,
                einSelVal(config.Source_Start_Sound[s]),0,EIN_SEL_MAX);
            break;
        case 2: {
            uint8_t t=einTyp(config.Source_Start_Sound[s]);
            if(t==7||t==8)
                crsf.send_param_response_CRSF_INFO(fi+2,fi,"Kanal Nr",
                    t==7?"Dauerbetrieb":"Deaktiviert");
            else crsf.send_param_response_CRSF_UINT8(fi+2,fi,"Kanal Nr",
                einNr(config.Source_Start_Sound[s]),1,einMaxNr(t),"");
            break;
        }
        case 3: crsf.send_param_response_CRSF_UINT8(fi+3,fi,"Volumen",
            (uint8_t)constrain(config.Volumen_Sound[s],0,200),0,200,""); break;
        case 4: crsf.send_param_response_CRSF_TEXT_SELECTION(fi+4,fi,
            "Wiedergabe Mode","Normal;Loop;Tippbetrieb",
            (uint8_t)constrain(config.Mode_Sound[s],0,2),0,2); break;
        case 5: crsf.send_param_response_CRSF_TEXT_SELECTION(fi+5,fi,
            "Test Sound","Aus;Ein",testSoundActive[s]?1:0,0,1); break;
        }
    }

    // ── Einstellungen (39..50) ────────────────────────────────────────
    else if(idx==39) crsf.send_param_response_CRSF_FOLDER(39,0,"Einstellungen",
        {40,41,42,43,44,45,46,47,48,49,50});
    else if(idx==40) crsf.send_param_response_CRSF_UINT8(40,39,"Modul Adresse",
        (uint8_t)constrain(config.modul_adress,0,20),0,20,"");
    else if(idx==41) crsf.send_param_response_CRSF_UINT8(41,39,"Gruppenadr. 1 (255=aus)",
        (uint8_t)constrain(config.EK_Gruppen_Adresse[0],0,255),0,255,"");
    else if(idx==42) crsf.send_param_response_CRSF_UINT8(42,39,"Gruppenadr. 2 (255=aus)",
        (uint8_t)constrain(config.EK_Gruppen_Adresse[1],0,255),0,255,"");
    // Optionsliste aus der Port-B-Rollen-Registry (port_function.h/.cpp).
    else if(idx==43) crsf.send_param_response_CRSF_TEXT_SELECTION(43,39,
        "Port B Modus",portBFunctionOptionsString(),
        (uint8_t)constrain(config.PortB_Mode,0,(int)portBFunctionMaxIndex()),0,portBFunctionMaxIndex());
    else if(idx==44) crsf.send_param_response_CRSF_TEXT_SELECTION(44,39,
        "Port B Baudrate",PORTB_BAUD_LIST_STR,
        portBBaudToIndex(config.PortB_Baud),0,PORTB_BAUD_LIST_COUNT-1);
    else if(idx==45) crsf.send_param_response_CRSF_TEXT_SELECTION(45,39,
        "Port C (GPS)","Aus;An",
        config.PortC_GPS_Enabled?1:0,0,1);
    // 46 wirkt SOFORT (siehe crsfWriteParam) und wird bewusst NICHT
    // persistiert - wie "Test Sound". 47/48/49 sind persistierte Konfiguration.
    else if(idx==46) crsf.send_param_response_CRSF_TEXT_SELECTION(46,39,
        "WLAN jetzt","Aus;Ein",
        WebServerManager::isApActive()?1:0,0,1);
    else if(idx==47) crsf.send_param_response_CRSF_TEXT_SELECTION(47,39,
        "Auto-WLAN bei Signalverlust","Aus;Ein",
        config.WifiAutoEnable?1:0,0,1);
    else if(idx==48) crsf.send_param_response_CRSF_UINT8(48,39,"Auto-WLAN Timeout (s)",
        (uint8_t)constrain(config.WifiAutoTimeoutSec,5,240),5,240,"");
    else if(idx==49) crsf.send_param_response_CRSF_TEXT_SELECTION(49,39,
        "WLAN dauerhaft an","Aus;Ein",
        config.WifiAlwaysOn?1:0,0,1);
    else if(idx==50) crsf.send_param_response_CRSF_TEXT_SELECTION(50,39,
        "Sound-Speicher",STORAGE_OPTIONS,
        (uint8_t)constrain(config.Sound_Storage,0,(int)STORAGE_MAX),0,STORAGE_MAX);

    // ── Geraetename (51) ──────────────────────────────────────────────
    else if(idx==51) {
        snprintf(buf,sizeof(buf),"%s (Web aendern)",config.Device_Name);
        crsf.send_param_response_CRSF_INFO(51,0,"Geraetename",buf);
    }
}

// ======== CRSF: Parameter schreiben ==================================
static void crsfWriteParam(uint8_t idx, uint8_t val) {
    if     (idx==3) {config.throttle_mode=constrain(val,0,1);markDirty();}
    else if(idx==4) {config.engine_on_toggle=constrain(val,0,1);markDirty();}
    else if(idx==5) {config.Source_Start_Sound[0]=einSelWrite(val,config.Source_Start_Sound[0]);markDirty();}
    else if(idx==6) {uint8_t t=einTyp(config.Source_Start_Sound[0]);config.Source_Start_Sound[0]=setEin(t,val);markDirty();}
    else if(idx==7) {
        // val 0=BUS-Kanal (bisherige Kanalnummer behalten, sonst Kanal 1), 1=Deaktiviert
        if(val==0){ if(!spdActive(config.Source_Speed_Sound_0)) config.Source_Speed_Sound_0=0; }
        else config.Source_Speed_Sound_0=999;
        markDirty();
    }
    else if(idx==8) {if(spdActive(config.Source_Speed_Sound_0)){config.Source_Speed_Sound_0=constrain((int)val-1,0,15);markDirty();}}
    else if(idx==9) {config.Volumen_Sound[0]=constrain(val,0,200);markDirty();}
    else if(idx==10){config.Min_Speed_Sound_0=constrain(val,0,200);markDirty();}
    else if(idx==11){config.Max_Speed_Sound_0=constrain((int)val*2,100,510);markDirty();}
    else if(idx==12){config.shutdowndelay=constrain(val,0,60);markDirty();}
    else if(idx==13){config.throttle_ramp=constrain(val,0,50);markDirty();}
    else if(idx==14){config.throttle_dead_band=constrain(val,0,50);markDirty();}
    else if(idx>=SOUND_FIRST_FI&&idx<=SOUND_LAST_FI) {
        uint8_t s=(idx-SOUND_FIRST_FI)/FIELDS_PER_SOUND+1, sub=(idx-SOUND_FIRST_FI)%FIELDS_PER_SOUND;
        if(s<1||s>RCSOUND_NUM_SOUNDS)return;
        if     (sub==1){config.Source_Start_Sound[s]=einSelWrite(val,config.Source_Start_Sound[s]);markDirty();}
        else if(sub==2){config.Source_Start_Sound[s]=setEin(einTyp(config.Source_Start_Sound[s]),val);markDirty();}
        else if(sub==3){config.Volumen_Sound[s]=constrain(val,0,200);markDirty();}
        else if(sub==4){config.Mode_Sound[s]=constrain(val,0,2);markDirty();}
        else if(sub==5){if(val==1)Sound_on_web[s]=true;testSoundActive[s]=(val==1);}
    }
    else if(idx==40){config.modul_adress=constrain(val,0,20);crsf.setDeviceAddress(crsfAddrFromWM());markDirty();}
    else if(idx==41){config.EK_Gruppen_Adresse[0]=constrain(val,0,255);markDirty();}
    else if(idx==42){config.EK_Gruppen_Adresse[1]=constrain(val,0,255);markDirty();}
    // ── Port B / Port C - wirken erst nach einem Neustart (siehe
    // PortB_Mode_boot/PortC_GPS_Enabled_boot).
    else if(idx==43){
        uint8_t m=constrain(val,0,(int)portBFunctionMaxIndex());
        if (m!=config.PortB_Mode) {
            // Baudrate beim Wechsel auf den protokolltypischen Standard
            // vorbelegen (0 = PORTB_OFF -> unveraendert lassen).
            uint32_t defBaud = portBFunctionDefaultBaud((PortBMode)m);
            if (defBaud != 0) config.PortB_Baud = defBaud;
        }
        config.PortB_Mode=m; markDirty();
        Serial.printf("!!! Port B Modus -> %d (wirkt erst nach Neustart) – NEUSTART ERFORDERLICH !!!\n",m);
    }
    else if(idx==44){
        uint8_t i=constrain(val,0,(int)PORTB_BAUD_LIST_COUNT-1);
        config.PortB_Baud=portBBaudList[i]; markDirty();
        Serial.printf("!!! Port B Baudrate -> %u (wirkt erst nach Neustart) – NEUSTART ERFORDERLICH !!!\n",(unsigned)config.PortB_Baud);
    }
    else if(idx==45){
        config.PortC_GPS_Enabled=constrain(val,0,1);markDirty();
        Serial.printf("!!! Port C (GPS) -> %d (wirkt erst nach Neustart) – NEUSTART ERFORDERLICH !!!\n",config.PortC_GPS_Enabled);
    }
    // ── WLAN jetzt (46) - wirkt SOFORT, absichtlich ohne Persistierung:
    // der naechste Start soll wieder vom Bootpin/"dauerhaft an" abhaengen.
    else if(idx==46){
        if (constrain(val,0,1)) WebServerManager::enableAP();
        else                    WebServerManager::disableAP();
    }
    else if(idx==47){config.WifiAutoEnable=constrain(val,0,1);markDirty();}
    else if(idx==48){config.WifiAutoTimeoutSec=(uint16_t)constrain((int)val,5,240);markDirty();}
    // 49: persistiert, wirkt erst ab dem naechsten Start (kein enableAP() aus
    // dem Parameter-Handler heraus - WLAN-Start zur Laufzeit ist die
    // fehleranfaelligste Operation, siehe V7).
    else if(idx==49){config.WifiAlwaysOn=constrain(val,0,1);markDirty();}
    else if(idx==50){
        config.Sound_Storage=constrain(val,0,(int)STORAGE_MAX);markDirty();
        Serial.printf("!!! Sound-Speicher -> %d (wirkt erst nach Neustart) – NEUSTART ERFORDERLICH !!!\n",config.Sound_Storage);
    }
}

// ======== Setup ======================================================
void setup() {
#ifdef PIN_VEXT
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, LOW); // LOW = Vext 3.3V-Stromversorgung aktiv (versorgt OLED & Vext-Pin)
#endif
#ifdef PIN_LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
#endif
    Serial.begin(115200);
    displayInit();
    loadConfig();

    strncpy(ssid,    config.WiFi_SSID,    sizeof(ssid)-1);    ssid[sizeof(ssid)-1]='\0';
    strncpy(password,config.WiFi_Password,sizeof(password)-1); password[sizeof(password)-1]='\0';

    // Port A: CRSF
    crsf.init_crsf(PORTA_SERIAL, PIN_CRSF_RX, PIN_CRSF_TX);
    crsf.setDeviceAddress(crsfAddrFromWM());   // eindeutige CRSF-Adresse aus WM-Adresse
    Serial.printf("CRSF (RX=%d, TX=%d)  Geraeteadresse 0x%02X  Slot %d\n",
                  PIN_CRSF_RX, PIN_CRSF_TX, crsfAddrFromWM(),
                  (uint8_t)constrain(config.modul_adress,0,15) * 2);

    pinMode(PIN_WIFI, INPUT_PULLUP);

    soundStoreBegin((uint8_t)config.Sound_Storage);
    LoadFiles();

    // begin() registriert nur die HTTP-Routen und merkt sich SSID/Passwort.
    // Das Einschalten von AP + Webserver passiert getrennt ueber enableAP().
    WebServerManager::begin(ssid, password);

    if (config.WifiAlwaysOn) {
        Serial.println("WLAN dauerhaft aktiv (config.WifiAlwaysOn) - schalte AP ein...");
        WebServerManager::enableAP();
    } else if (!digitalRead(PIN_WIFI)) {
        Serial.println("AP Modus (PIN_WIFI beim Booten auf LOW)...");
        WebServerManager::enableAP();
    }

    // SD-Lese-Task und Sample-Puffer erst NACH dem WLAN-AP-Start, damit
    // WiFi.softAP() auf einem moeglichst freien Heap laeuft (siehe
    // XT_I2S_Audio.h).
    I2SAudio.StartSdReaderTask();
    AllocateSoundBuffers();

    PortB_Mode_boot         = config.PortB_Mode;
    PortB_Baud_boot         = config.PortB_Baud;
    PortC_GPS_Enabled_boot  = config.PortC_GPS_Enabled;
    snprintf(versionString, sizeof(versionString), "%d.%02d", Version/100, Version%100);
    saveConfig();

    // Port B: welches Modul fuer die gewaehlte Rolle zustaendig ist, kommt
    // aus der Port-B-Rollen-Registry (port_function.h/.cpp).
    {
        const PortBFunctionModule* portBModule = getPortBModule((PortBMode)PortB_Mode_boot);
        if (portBModule != nullptr) {
            if (portBModule->isActive != nullptr && portBModule->isActive()) {
                Serial.println("!!! Port B: Rolle ist bereits aktiv, ueberspringe erneute Initialisierung !!!");
            } else if (portBModule->begin != nullptr) {
                portBModule->begin(PORTB_SERIAL, PIN_PORTB_RX, PIN_PORTB_TX, PortB_Baud_boot);
            }
        }
        // PORTB_OFF -> getPortBModule() liefert nullptr, Port B bleibt aus.
    }

    // Port C (GPS)
    if (PortC_GPS_Enabled_boot) gpsSpeedInit();

    Serial.printf("Soundmodul Mini %s (ESP32-S3)\n", versionString);

    char ipStr[20] = "";
    if (WebServerManager::isApActive()) {
        snprintf(ipStr, sizeof(ipStr), "%s", WiFi.softAPIP().toString().c_str());
    }
    displayShowBoot(versionString,
                    soundStoreName(),
                    crsfAddrFromWM(),
                    portBFunctionName((PortBMode)PortB_Mode_boot),
                    WebServerManager::isApActive(),
                    ipStr);
}

// Auto-WLAN-Failsafe: schaltet das WLAN ein, wenn eine Zeit lang kein
// gueltiges CRSF-Signal anliegt (z.B. Modell auf der Werkbank, Sender aus).
// Schaltet NUR EIN, nie automatisch wieder aus - sonst wuerde eine laufende
// Konfiguration/ein OTA-Update abbrechen, sobald kurz wieder Signal kommt.
// Mit "WLAN dauerhaft an" ist der AP schon beim Booten an, dann greift diese
// Funktion nicht.
void wifiFailsafeCheck() {
    static unsigned long busLostSinceMs = 0;
    static bool          busLostSincePending = false;

    if (config.WifiAlwaysOn || BUS_OK || !config.WifiAutoEnable || WebServerManager::isApActive()) {
        busLostSincePending = false;
        return;
    }
    if (!busLostSincePending) {
        busLostSincePending = true;
        busLostSinceMs = millis();
        return;
    }
    unsigned long timeoutMs = (unsigned long)config.WifiAutoTimeoutSec * 1000UL;
    if (millis() - busLostSinceMs >= timeoutMs) {
        Serial.printf("Auto-WLAN-Failsafe: %u s ohne gueltiges RC-Signal - schalte WLAN ein.\n",
                      (unsigned)config.WifiAutoTimeoutSec);
        WebServerManager::enableAP();
    }
}

// ======== Loop =======================================================
void loop() {
    currentTime = millis();
    if (WebServerManager::isApActive()) WebServerManager::Webpage();

    // Port B - Boot-Snapshot verwenden, generischer Registry-Aufruf.
    {
        const PortBFunctionModule* portBModule = getPortBModule((PortBMode)PortB_Mode_boot);
        if (portBModule != nullptr && portBModule->update != nullptr) portBModule->update();
    }
    // Port C (GPS) laufend einlesen
    if (PortC_GPS_Enabled_boot) gpsSpeedUpdate();

    // Port A: CRSF
    crsf.read_packets(0);
    for (int i=0;i<16;i++) channel_output[i]=crsf.get_crfs_channels(i);
    // BUS_OK haengt daran, ob gerade ein gueltiger CRSF-Frame geparst wurde
    // (Zaehler validFrames), nicht an einem Kanalwert.
    static uint32_t lastValidFrameCount = 0;
    uint32_t vf = crsf.getValidFrames();
    if (vf != lastValidFrameCount) { lastValidFrameCount = vf; BUS_OK = true; lastCrsfPacket = millis(); }
    checkCrsfTimeout();

    static unsigned long lastTelem = 0;
    if (millis() - lastTelem >= 100) {
        lastTelem = millis();
        // Nur Einzelzellen-Telemetrie (0x0E), kein Batterie-Frame (0x08) -
        // sonst wuerde die vom Empfaenger gemessene RxBt ueberschrieben.
        if (PortB_Mode_boot == PORTB_SPORT) {
            if (lipoSensor[0].online) sportSendCellsTelemetry(0);
            if (lipoSensor[1].online) sportSendCellsTelemetry(1);
        }
        // GPS-Telemetrie (CRSF-GPS-Frame) -> EdgeTX zeigt "GSpd"
        if (PortC_GPS_Enabled_boot) {
            uint16_t gspd = (uint16_t)(gpsGetSpeedKmh() * 10.0f); // CRSF: km/h * 10
            crsf.send_tele_GPS(gpsGetLatE7(), gpsGetLonE7(), gspd,
                               gpsGetHeadingCdeg(), gpsGetAltMeters(), gpsGetSatellites());
        }
    }
    if(crsf.getDeviceInfoReplyPending() && (millis() - crsf.getPingTime() >= crsfSlotDelayMs())){
        // Erst im eigenen Zeit-Slot auf den Broadcast-Ping antworten.
        crsf.setDeviceInfoReplyPending(false);
        char dn[24]; snprintf(dn,sizeof(dn),"%s@%d",config.Device_Name,config.modul_adress);
        // Firmware-Version als "Parameter Version Number", damit Konfiguratoren
        // eine geaenderte Parameterstruktur neu einlesen.
        crsf.send_device_info(dn,CRSF_PARAM_COUNT,(uint8_t)(Version & 0xFF));
    }
    if(crsf.getDeviceReadReplyPending()){
        uint8_t pi=crsf.getParamReadIndex();
        crsf.setDeviceReadReplyPending(false);
        crsfSendParam(pi);
    }
    if(crsf.getDeviceWriteReplyPending()){
        uint8_t wi=crsf.getParamWriteIndex(),wv=crsf.getParamWriteValue();
        crsf.setDeviceWriteReplyPending(false);
        crsfWriteParam(wi,wv); crsfSendParam(wi);
    }
    if(crsf.getDeviceCommandReplyPending()){
        crsf.setDeviceCommandReplyPending(false);
        einkanalFunctionCRSF();
    }

    // NVS-Speicher: 500 ms nach der letzten Aenderung schreiben, und nicht
    // waehrend des Cooldowns nach einer SD-Schreibaktivitaet (Brownout-Schutz,
    // siehe WebServerManager::sdActivityCooldownActive()). Einzige Stelle, an
    // der im Betrieb gespeichert wird.
    if(configDirty&&(millis()-configDirtyMs)>=500UL&&!WebServerManager::sdActivityCooldownActive())
        saveConfigForce();

    wifiFailsafeCheck();

    Config();
    if(currentTime<5000) return;
    for(uint8_t i=1;i<=RCSOUND_NUM_SOUNDS;i++) handleSound(i,Sounds[i]);

    // Motor Toggle
    if(Sound_on[0]&&!Sound_on_Motor_state&&config.engine_on_toggle){Sound_on_Motor=!Sound_on_Motor;Sound_on_Motor_state=true;}
    else if(!Sound_on[0]&&Sound_on_Motor_state&&config.engine_on_toggle) Sound_on_Motor_state=false;
    else if(!config.engine_on_toggle) Sound_on_Motor=Sound_on[0];

    bool mw=Sound_on_Motor||Sound_on_web[0];
    if((mw&&!engine_break&&!Sound_play[0])||(throttle>0&&engine_break&&!Sound_play[0])||(throttle>0&&Sound_play[0])){Sound_play[0]=true;shutdown_timer=millis();}
    if(!mw){Sound_play[0]=false;engine_break=false;}
    if(config.shutdowndelay>0&&Sound_play[0]&&(millis()-shutdown_timer)>((unsigned long)config.shutdowndelay*1000UL)){Sound_play[0]=false;engine_break=true;}

    switch(engine_State){
        case OFF:    if(Sound_play[0]){Sound_start.Volume=config.Volumen_Sound[0];I2SAudio.Play(&Sound_start);engine_State=STARTING;} break;
        case STARTING: if(!Sound_start.Playing){throttle=0;last_throttle=0;Sound_loop.RepeatForever=true;Sound_loop.Volume=config.Volumen_Sound[0];I2SAudio.Play(&Sound_loop);engine_State=RUNNING;} break;
        case RUNNING:  Sound_loop.Volume=config.Volumen_Sound[0]; if(!Sound_play[0]){throttle=0;if(last_throttle==0){Sound_loop.RepeatForever=false;I2SAudio.Stop(&Sound_loop);engine_State=STOPPING;}} break;
        case STOPPING: if(!Sound_loop.Playing){Sound_shut.Volume=config.Volumen_Sound[0];I2SAudio.Play(&Sound_shut);engine_State=OFF;} break;
    }
    throttle=ramp_throttle(throttle);
    if(config.Min_Speed_Sound_0!=config.Max_Speed_Sound_0)
        Sound_loop.Speed=floatMap(throttle,0,100,config.Min_Speed_Sound_0,config.Max_Speed_Sound_0)/100.0f;

    // Display-Live-Status aktualisieren
    {
        const char* activeSound = nullptr;
        for (uint8_t i = 1; i <= RCSOUND_NUM_SOUNDS; i++) {
            if (Sound_play[i]) {
                static char sndBuf[12];
                snprintf(sndBuf, sizeof(sndBuf), "Sound %d", i);
                activeSound = sndBuf;
                break;
            }
        }
        char ipBuf[20] = "";
        if (WebServerManager::isApActive()) {
            snprintf(ipBuf, sizeof(ipBuf), "%s", WiFi.softAPIP().toString().c_str());
        }
        displayUpdate(BUS_OK,
                      engine_State,
                      throttle,
                      activeSound,
                      WebServerManager::isApActive(),
                      ipBuf,
                      portBFunctionName((PortBMode)PortB_Mode_boot));
    }
}

// ======== Sound ======================================================
void handleSound(uint8_t idx, XT_Wav_Class* snd) {
    if(!snd)return;
    if(Sound_on[idx]||Sound_on_web[idx]){
        snd->Volume=config.Volumen_Sound[idx];
        if(!Sound_play[idx]){Sound_play[idx]=true;snd->LoadWavFile();if(config.Mode_Sound[idx]==1)snd->RepeatForever=true;I2SAudio.Play(snd);Sound_on_web[idx]=false;}
    }else{
        snd->RepeatForever=false;
        // Lautstaerke auch waehrend des Nachspielens aktualisieren (z.B.
        // Web-Testbutton, der Sound_on_web sofort wieder zuruecksetzt).
        if(Sound_play[idx]) snd->Volume=config.Volumen_Sound[idx];
        if(config.Mode_Sound[idx]==2&&Sound_play[idx])I2SAudio.Stop(snd);
        // testSoundActive (Lua "Test Sound") zuruecksetzen, sobald die
        // Wiedergabe endet. Im Loop-Mode bleibt "Ein" bewusst stehen.
        if(!snd->Playing&&Sound_play[idx]){snd->UnLoadWavFile();Sound_play[idx]=false;testSoundActive[idx]=false;}
    }
}

// ======== Config: Quellen auswerten ==================================
void Config() {
    for (int x=0; x<RCSOUND_NUM_SLOTS; x++) {
        int src = config.Source_Start_Sound[x];
        if      (src>=0   && src<=15)  Sound_on[x] = BUS_OK && (channel_output[src] < 624);
        else if (src>=20  && src<=35)  Sound_on[x] = BUS_OK && (channel_output[src-20] > 1424);
        else if (src>=70  && src<=77)  Sound_on[x] = BUS_OK && bitRead(einkanal_Data, src-70);
        else if (src>=80  && src<=95) {
            // "MKan": 16 Einzelkanaele aus 2 CRSF-Gruppenadressen (je 8
            // Schalter). nr 0..15 -> Gruppe = nr/8, Bit = nr%8.
            uint8_t nr = (uint8_t)(src - 80);
            Sound_on[x] = BUS_OK && bitRead(einkanal_Flat16, nr);
        }
        else if (src==200)             Sound_on[x] = true;
        else                           Sound_on[x] = false;
    }

    // Motor Speed
    int spd = config.Source_Speed_Sound_0;
    if (spdActive(spd)) throttle = map(channel_output[spd], 82, 1900, 0, 100);
    else                throttle = 0;
    throttle = constrain(throttle, 0, 100);

    if (config.throttle_mode) {
        if      (throttle > 50+config.throttle_dead_band) throttle=map((long)throttle,50+config.throttle_dead_band,100,0,100);
        else if (throttle < 50-config.throttle_dead_band) throttle=map((long)throttle,50-config.throttle_dead_band,0,0,100);
        else throttle=0;
    } else {
        throttle=(throttle>config.throttle_dead_band)?map((long)throttle,config.throttle_dead_band,100,0,100):0;
    }
    throttle = constrain(throttle, 0, 100);
}

// ======== Hilfsfunktionen ============================================
float floatMap(float x,float a,float b,float c,float d){if(b==a)return c;return(x-a)*(d-c)/(b-a)+c;}
// max_files NICHT erhoehen (Erfahrung aus V7, v7.07..v7.10): ein groesserer
// Wert belegt beim Mounten dauerhaft einen grossen DRAM-Block und laesst den
// WLAN-Start an Fragmentierung scheitern. Der Deskriptor-Engpass wird beim
// Upload durch Freigeben ungenutzter Handles geloest (WebServerManager.cpp).
void LoadFiles(){Sound_loop.LoadWavFile();Sound_shut.LoadWavFile();Sound_start.LoadWavFile();}

// Holt die Sample-Puffer ALLER Sound-Objekte vom Heap - aufgerufen aus
// setup() NACH dem WLAN-AP-Start (siehe XT_I2S_Audio.h).
void AllocateSoundBuffers(){
    Sound_loop.AllocateBuffers();
    Sound_shut.AllocateBuffers();
    Sound_start.AllocateBuffers();
    for (int i = 1; i < RCSOUND_NUM_SLOTS; i++) {
        if (Sounds[i]) Sounds[i]->AllocateBuffers();
    }
}

float ramp_throttle(float tn){
    unsigned long rt=currentTime-previousTime_ramp; previousTime_ramp=currentTime;
    if(rt>1000)rt=1000; float step=(config.throttle_ramp/1000.0f)*(float)rt;
    if(tn>last_throttle){float s=last_throttle+step;tn=(s<tn)?s:tn;}
    else{float s=last_throttle-step;tn=(s>tn)?s:tn;}
    last_throttle=tn; return tn;
}

// ======== Einkanal CRSF ==============================================
// Jede eingehende Schalter-Adresse wird gegen die eigene Adresse
// (config.modul_adress, Quelltyp 70-77) UND gegen die 2 Gruppenadressen
// (config.EK_Gruppen_Adresse[], Quelltyp "MKan" 80-95) geprueft.
static void updateGroupFromAddress(uint8_t addr, uint8_t bits){
    for(uint8_t g=0; g<RCSOUND_NUM_GROUPS; g++){
        if(config.EK_Gruppen_Adresse[g]!=255 && addr==(uint8_t)config.EK_Gruppen_Adresse[g]){
            einkanal_Gruppe[g]=bits;
        }
    }
    // Bewusst per Hand (2 Gruppen -> 16 Bit); eine Aenderung von
    // RCSOUND_NUM_GROUPS muss diese Zeile mitziehen.
    einkanal_Flat16 = (uint32_t)einkanal_Gruppe[0]
                     | ((uint32_t)einkanal_Gruppe[1] << 8);
}
void einkanalFunctionCRSF(){
    uint8_t wmc=crsf.get_cmd_buffer(5),cmd=crsf.get_cmd_buffer(6);
    if(wmc!=Multiswitch)return;
    switch(cmd){
        // MWset4: 8 Schalter, 2 Bit/Schalter an einer Adresse - ausgewertet
        // wird 1 Bit/Schalter (compressSwitches()).
        case MWset4:{
            uint8_t a=crsf.get_cmd_buffer(7);
            uint16_t s=((uint16_t)crsf.get_cmd_buffer(8)<<8)|crsf.get_cmd_buffer(9);
            uint8_t bits=compressSwitches(s);
            if(a==(uint8_t)config.modul_adress) einkanal_Data=bits;
            updateGroupFromAddress(a,bits);
            break;
        }
        case MWset4m:{
            uint8_t c=min((uint8_t)crsf.get_cmd_buffer(7),(uint8_t)7);
            for(uint8_t i=0;i<c;i++){
                uint8_t a=crsf.get_cmd_buffer(8+(3*i));
                uint16_t s=((uint16_t)crsf.get_cmd_buffer(9+(3*i))<<8)|crsf.get_cmd_buffer(10+(3*i));
                uint8_t bits=compressSwitches(s);
                if(a==(uint8_t)config.modul_adress) einkanal_Data=bits;
                updateGroupFromAddress(a,bits);
            }
            break;
        }
        // MWset liefert bereits 1 Bit/Schalter direkt.
        case MWset:{
            uint8_t a=crsf.get_cmd_buffer(7);
            uint8_t b=crsf.get_cmd_buffer(8);
            if(a==(uint8_t)config.modul_adress) einkanal_Data=b;
            updateGroupFromAddress(a,b);
            break;
        }
        case MWprop:break;
    }
}
uint8_t compressSwitches(uint16_t s){uint8_t r=0;for(uint8_t i=0;i<8;i++)r|=((s>>(i*2))&0x1)<<i;return r;}
