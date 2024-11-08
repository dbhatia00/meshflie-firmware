// File: src/modules/interface/telemetry_task.h

#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

#include <stdint.h>

/**
 * @brief Initializes the telemetry task and registers the CPX receive callback.
 *
 * This function sets up the telemetry task that collects telemetry data
 * (battery voltage, roll, pitch, yaw) and sends it via CPX to the ESP32.
 * It also registers a callback to handle incoming telemetry data from other drones.
 */
void telemetryTaskInit();

#endif // TELEMETRY_TASK_H
