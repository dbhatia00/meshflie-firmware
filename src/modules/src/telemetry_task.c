#include "FreeRTOS.h"
#include "task.h"
#include "cpx.h"
#include "pm.h"           // For battery voltage
#include "sensfusion6.h"  // For attitude data
#include "log.h"
#include "telemetry.h"
#include "cpx_internal_router.h"

#include "debug.h"
#define DEBUG_MODULE "TELEMTASK"

static void telemetryTask(void* parameters) {
    // Initialize CPX communication if not already initialized
    TelemetryData_t telemetryData;

    while (1) {
        // Collect telemetry data
        // Retrieve roll, pitch, yaw using sensorfusion6GetEulerRPY
        float roll, pitch, yaw;
        sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
        telemetryData.roll = roll;
        telemetryData.pitch = pitch;
        telemetryData.yaw = yaw;
        
        telemetryData.batteryVoltage = pmGetBatteryVoltage();
        
            // Process the received telemetry data
        DEBUG_PRINT("Telemetry: Packaging Values: Voltage=%.2fV, Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
                  telemetryData.batteryVoltage,
                  telemetryData.roll,
                  telemetryData.pitch,
                  telemetryData.yaw);

        // Prepare CPX packet
        // Delay before sending the next packet
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust the delay as needed
    }
}
/*
static void cpxReceiveCallback(const CPXPacket_t* cpxRx) {
    if (cpxRx->route.source == CPX_T_ESP32 && cpxRx->route.function == CPX_F_APP) {
        TelemetryData_t receivedData;
        if (cpxRx->dataLength == sizeof(TelemetryData_t)) {
            memcpy(&receivedData, cpxRx->data, sizeof(TelemetryData_t));

            // Process the received telemetry data
            DEBUG_PRINT("Telemetry: Received from another drone: Voltage=%.2fV, Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
                  receivedData.batteryVoltage,
                  receivedData.roll,
                  receivedData.pitch,
                  receivedData.yaw);

            // Can embed commands here! May wanna enact a storage mechanism?

        } else {
            consolePrintf("Telemetry Received data of unexpected length: %d", cpxRx->dataLength);
        }
    }
}
*/
void telemetryTaskInit() {
    DEBUG_PRINT("Initializing Telemetry Task...\n");

    if (xTaskCreate(telemetryTask, "TelemetryTask", 200, NULL, tskIDLE_PRIORITY + 100, NULL) != pdPASS) {
        DEBUG_PRINT("Failed to create Telemetry Task\n");
    } else {
        DEBUG_PRINT("Telemetry Task Created Successfully\n");
    }
}
