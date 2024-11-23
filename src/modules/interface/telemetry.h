// File: src/modules/interface/telemetry.h

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

typedef struct {
    uint8_t droneID;
    double batteryVoltage;
    double roll;
    double pitch;
    double yaw;
} __attribute__((packed)) TelemetryData_t;

#endif // TELEMETRY_H
