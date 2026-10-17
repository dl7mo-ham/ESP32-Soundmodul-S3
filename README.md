# Soundmodul Mini (Heltec V3 / S3)

Kompaktes Soundmodul für Motorgeräusche und Effekte auf Basis **Heltec WiFi LoRa 32 V3** (ESP32-S3FN8, 8 MB Flash, kein externes PSRAM).
Abgeleitet von [ESP32-RC-Sound V7](https://github.com/mnorden81/mnorden81-ESP32-Sound-bidirectional_code), reduziert auf:

- **Eingang:** nur CRSF (Port A), busbasiert
- **Sounds:** Motor (Start/Loop/Stopp) + 4 Zusatzsounds
- **Speicher:** Flash (LittleFS), SD-Karte optional
- **Ports:** Port A (CRSF), Port B (zweiter UART), Port C (GPS)
- **Web-UI:** Sound-Upload und OTA-Update per WLAN (AP über Onboard-PRG-Taster aktivierbar)
- **Verstärker:** MAX98357A (I2S)

**Aktuelle Version:** Grundgerüst angepasst für Heltec V3

Pinbelegung und Anschlussplan: siehe [docs/pinout.md](docs/pinout.md).

## Flash-Aufteilung (8 MB)

Mit `partitions_8mb.csv`:
- 2 × 2,5 MB Firmware (OTA)
- ca. 2,87 MB Sounds im LittleFS (≈ 65 s bei 22,05 kHz, 16 Bit mono)

## Sounds

Dateinamen: `start.wav`, `loop.wav`, `shut.wav`, `sound1.wav` … `sound4.wav` (16 Bit PCM, 22,05 kHz mono empfohlen).

Einstellung **Sound-Speicher** (Web-UI → Einstellungen oder Lua-Menü → Einstellungen), wirkt nach Neustart:

- **Auto** (Standard): SD-Karte, wenn eine steckt, sonst Flash
- **Flash**: interner Speicher (LittleFS), keine SD-Karte nötig
- **SD**: nur SD-Karte

Sounds ins Flash bringen:

1. Web-UI: Tab „Motor“ bzw. „Sounds“ → WAV hochladen (geht in den aktiven Speicher)
2. USB: Dateien nach `firmware/data/` legen, dann `pio run -t uploadfs`

Konfigurations-Backups liegen immer auf der SD-Karte.

## Bauen & Flashen

```bash
cd firmware
pio run                 # Firmware bauen (Default: env heltec_v3)
pio run -t upload       # flashen
pio run -t uploadfs     # Sounds aus firmware/data/ ins Flash
```

## Ordner

- `firmware/` PlatformIO-Projekt (`platformio.ini`, `partitions_8mb.csv`)
- `hardware/` Schaltplan, PCB
- `docs/` Pinout (`docs/pinout.md`), Projektplan
- `backups/` funktionierende Stände (nur lokal, nicht im Repo)

## Lizenz

GPL-3.0, wie das Ursprungsprojekt (enthält Code von XTronical).
