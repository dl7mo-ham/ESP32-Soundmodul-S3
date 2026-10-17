# Agent Guidelines & Rules

## Arbeitsweise: Auto-Modus & Autonomie
- **Keine unnötigen Rückfragen:** Bei normaler Entwicklungsarbeit (Code schreiben, Dateien bearbeiten, Fehler analysieren, Dokumentation aktualisieren) selbstständig und zügig arbeiten, ohne für Standardentscheidungen Bestätigungen zu erfragen.
- **Vollständige Umsetzung:** Aufgaben direkt in einem Durchgang abschließen.
- **Prägnante Antworten:** Ergebnisse kompakt und übersichtlich zusammenfassen.

## Projekt-Kontext (Soundmodul Mini)
- **Ziel-Hardware:** Heltec WiFi LoRa 32 V3 (ESP32-S3FN8, 8 MB Flash, kein PSRAM).
- **Environment:** PlatformIO `heltec_v3` mit `partitions_8mb.csv` (2,5 MB OTA0, 2,5 MB OTA1, 2,87 MB LittleFS).
- **Pins:** Zentrale Definition in [pins.h](file:///home/michael/Schreibtisch/soundmodul/firmware/src/pins.h) und Dokumentation in [pinout.md](file:///home/michael/Schreibtisch/soundmodul/docs/pinout.md). Niemals interne LoRa- (8..14) oder OLED-Pins (17, 18, 21, 36) für externe Schnittstellen umbelegen.
