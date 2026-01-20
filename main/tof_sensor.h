#pragma once

#include <stdbool.h>
#include <stdint.h>

// I2C Pins für VL53L0X
#define TOF_I2C_SDA     14
#define TOF_I2C_SCL     21
#define TOF_XSHUT_PIN   47

// Motion Detection Parameter
#define MOTION_THRESHOLD_MM     500   // Distanzänderung für Bewegung
#define MOTION_COOLDOWN_MS      240000  // 4 Minuten Cooldown nach Wake
#define TOF_SAMPLE_INTERVAL_MS  200   // Abtastrate
#define MAX_DISTANCE_MM         2000  // Alles darüber = "nichts erkannt"

// Initialisiert den VL53L0X Sensor
void tof_sensor_init(void);
