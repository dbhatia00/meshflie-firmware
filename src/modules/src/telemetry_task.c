#include "FreeRTOS.h"
#include "task.h"
#include "cpx.h"
#include "pm.h"           // For battery voltage
#include "sensfusion6.h"  // For attitude data
#include "log.h"
#include "telemetry.h"
#include "cpx_internal_router.h"
#include "system.h"       // For getting drone ID
#include <stdlib.h>
#include "debug.h"
#define DEBUG_MODULE "TELEMTASK"

uint8_t droneId = 40;

static void telemetryTask(void* parameters) {
    // Initialize CPX communication if not already initialized
    //CPXPacket_t packet;
    TelemetryData_t telemetryData;
    CPXRouting_t route;

    // Initialize the route
    cpxInitRoute(
        CPX_T_STM32,       // Source: STM32 (Crazyflie)
        CPX_T_ESP32,       // Destination: ESP32 (AI Deck)
        CPX_F_APP,         // Function: Application-specific
        &route
    );

    while (1) {
        // Collect telemetry data
        // Retrieve roll, pitch, yaw using sensorfusion6GetEulerRPY
        float roll, pitch, yaw;
        sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
        telemetryData.roll = roll;
        telemetryData.pitch = pitch;
        telemetryData.yaw = yaw;
        
        telemetryData.batteryVoltage = pmGetBatteryVoltage();
        telemetryData.droneID = droneId;
        
        // Process the telemetry data
        /*
        DEBUG_PRINT("Telemetry: Packaging Values: DroneID=%d, Voltage=%.2fV, Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
                  telemetryData.droneID,
                  telemetryData.batteryVoltage,
                  telemetryData.roll,
                  telemetryData.pitch,
                  telemetryData.yaw);
        */

        // Prepare CPX packet
        CPXPacket_t packet;
        packet.route = route;
        packet.route.lastPacket = true;  // Set to true if this is the last packet in a sequence
        packet.route.version = CPX_VERSION;  // Use the CPX version defined in cpx.h
        packet.dataLength = sizeof(TelemetryData_t);
        memcpy(packet.data, &telemetryData, sizeof(TelemetryData_t));

        // Send the packet to ESP32
        cpxSendPacketBlocking(&packet);

        // Delay before sending the next packet
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust the delay as needed
    }
}

// Callback to handle received telemetry data from other drones via ESP32
static void cpxReceiveCallback(const CPXPacket_t* cpxRx) {
    if (cpxRx->route.source == CPX_T_ESP32 && cpxRx->route.function == CPX_F_APP) {
        TelemetryData_t receivedData;
        if (cpxRx->dataLength == sizeof(TelemetryData_t)) {
            memcpy(&receivedData, cpxRx->data, sizeof(TelemetryData_t));

            // Process the received telemetry data from other drones
            DEBUG_PRINT("Received telemetry from another drone: DroneID=%d, Voltage=%.2fV, Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
                      receivedData.droneID,
                      receivedData.batteryVoltage,
                      receivedData.roll,
                      receivedData.pitch,
                      receivedData.yaw);
        } else {
            DEBUG_PRINT("Received data of unexpected length: %d\n", cpxRx->dataLength);
        }
    }
}

void telemetryTaskInit() {

    DEBUG_PRINT("Initializing Telemetry Task...\n");

    if (xTaskCreate(telemetryTask, "TelemetryTask", 200, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        DEBUG_PRINT("Failed to create Telemetry Task\n");
    } else {
        DEBUG_PRINT("Telemetry Task Created Successfully\n");
    }

    // Register the callback to handle incoming CPX packets from ESP32
    cpxRegisterAppMessageHandler(cpxReceiveCallback);
}
