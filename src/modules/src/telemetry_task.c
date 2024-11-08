#include "FreeRTOS.h"
#include "task.h"
#include "cpx.h"
#include "pm.h"           // For battery voltage
#include "sensfusion6.h"  // For attitude data
#include "log.h"
#include "telemetry.h"
#include "cpx_internal_router.h"
#include "console.h"          // Include console.h for consolePrintf

static void telemetryTask(void* parameters) {
    // Initialize CPX communication if not already initialized
    cpxInit();

    CPXPacket_t packet;
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
        
        // Prepare CPX packet
        packet.route = route;
        packet.route.lastPacket = true;  // Set to true if this is the last packet in a sequence
        packet.route.version = CPX_VERSION;  // Use the CPX version defined in cpx.h
        packet.dataLength = sizeof(TelemetryData_t);

        // Ensure data fits within CPX_MAX_PAYLOAD_SIZE
        if (packet.dataLength <= CPX_MAX_PAYLOAD_SIZE) {
            memcpy(packet.data, &telemetryData, packet.dataLength);

            // Send the packet to the ESP32
            cpxSendPacketBlocking(&packet);
        } else {
            consolePrintf("Telemetry: Telemetry data size exceeds CPX_MAX_PAYLOAD_SIZE\n");
        }

        // Delay before sending the next packet
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust the delay as needed
    }
}

static void cpxReceiveCallback(const CPXPacket_t* cpxRx) {
    if (cpxRx->route.source == CPX_T_ESP32 && cpxRx->route.function == CPX_F_APP) {
        TelemetryData_t receivedData;
        if (cpxRx->dataLength == sizeof(TelemetryData_t)) {
            memcpy(&receivedData, cpxRx->data, sizeof(TelemetryData_t));

            // Process the received telemetry data
            consolePrintf("Telemetry: Received from another drone: Voltage=%.2fV, Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
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

void telemetryTaskInit() {
    // Initialize CPX communication
    cpxInit();

    // Register the CPX receive callback
    cpxRegisterAppMessageHandler(cpxReceiveCallback);

    // Create the telemetry task
    xTaskCreate(telemetryTask, "TelemetryTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
}
