# ESP32-S3 USB HID Wake Tool mit VL53L0X ToF Sensor

Weckt Windows 11 PC aus dem Standby wenn Bewegung vor dem ToF-Sensor erkannt wird.

## Features

- **VL53L0X ToF Sensor** - Laser-Distanzmessung (Pololu-Port mit voller Kalibrierung)
- **USB HID Keyboard** - Remote Wakeup Signal an Windows PC
- **Motion Detection** - Delta-basiert (Änderung > 500mm = Trigger)
- **Cooldown** - 4 Minuten Pause nach Wake

## Hardware

| Komponente | Pin |
|------------|-----|
| VL53L0X SDA | GPIO 14 |
| VL53L0X SCL | GPIO 21 |
| VL53L0X XSHUT | GPIO 47 |
| USB OTG | Nativer USB Port (nicht UART!) |

**Board:** ESP32-S3 WROOM (z.B. Freenove)

## Schnellstart

```bash
# 1. ESP-IDF installieren (einmalig)
./00-init.sh

# 2. Target setzen (einmalig)
./00-set-target.sh esp32s3

# 3. Bauen
./01-build.sh

# 4. Flashen (USB-Serial Adapter an TX/RX)
./02-flash.sh

# 5. Monitor (optional - zum Debuggen)
./03-monitor.sh
```

## ⚙️ Parameter anpassen

Die wichtigsten Parameter findest du in `main/tof_sensor.h`:

```c
// Motion Detection Parameter
#define MOTION_THRESHOLD_MM     500     // Distanzänderung für Trigger (in mm)
#define MOTION_COOLDOWN_MS      240000  // Pause nach Wake (240000 = 4 Minuten)
#define TOF_SAMPLE_INTERVAL_MS  200     // Abtastrate (200 = 5x pro Sekunde)
#define MAX_DISTANCE_MM         2000    // Alles darüber = "nichts erkannt"
```

### Parameter-Bedeutung:

| Parameter | Beschreibung | Empfehlung |
|-----------|--------------|------------|
| `MOTION_THRESHOLD_MM` | Wie viel mm muss sich ändern für Motion? | 300-500mm |
| `MOTION_COOLDOWN_MS` | Wie lange warten nach Wake? | 60000-300000 (1-5 Min) |
| `TOF_SAMPLE_INTERVAL_MS` | Wie oft messen? | 100-500ms |

### Nach Änderung neu flashen:

```bash
cd esp-idf-wakeup

# Bauen und Flashen in einem Schritt:
source esp-idf/export.sh
idf.py build flash -p /dev/cu.usbserial-110

# Oder mit den Skripten:
./01-build.sh
./02-flash.sh
```

**Hinweis:** Der Port `/dev/cu.usbserial-110` kann bei dir anders heißen. Prüfe mit `ls /dev/cu.*`

## Windows Setup

1. ESP32 mit **USB OTG Port** (nicht UART!) an Windows PC anschließen
2. Windows erkennt "Wakeup Keyboard Device"
3. **Device Manager** → Keyboards → "Wakeup Keyboard Device"
4. **Properties** → **Power Management**
5. ✅ **"Allow this device to wake the computer"** aktivieren

## Funktionsweise

1. Sensor misst kontinuierlich Distanz
2. Bei großer Änderung (> 500mm) → Motion erkannt
3. Wenn PC schläft → USB Remote Wakeup Signal
4. Wenn PC wach → Leertaste (optional)
5. 4 Minuten Cooldown bis nächster Trigger

## Troubleshooting

### Sensor zeigt nur 65535 / 2000mm
- Kabel prüfen (SDA, SCL, VCC, GND)
- XSHUT an GPIO47?
- Nach Kabel-Änderung: Reset-Knopf drücken

### PC wacht nicht auf
- "Allow device to wake computer" in Windows aktiviert?
- USB OTG Port verwendet (nicht UART)?
- Remote Wakeup im Log: `remote_wakeup_en=1`?

### Zu viele False-Positives
- `MOTION_THRESHOLD_MM` erhöhen (z.B. 600-800)
- `TOF_SAMPLE_INTERVAL_MS` erhöhen (z.B. 300-500)

## Ursprüngliches Projekt

Basiert auf [esp-remote-wakeup](https://github.com/original/esp-remote-wakeup) - 
erweitert um VL53L0X ToF Sensor für automatische Bewegungserkennung.

