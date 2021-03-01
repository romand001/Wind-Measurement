#include <Arduino.h>
#include <Seeed_Arduino_FreeRTOS.h>
#include <SparkFunMPU9250-DMP.h>

TaskHandle_t accelTaskHandle;

MPU9250_DMP imu;

static void accelThread(void* pvParameters) {

    while (imu.begin() != INV_SUCCESS) {
        Serial.println("IMU setup failed");
        delay(3000);
    }

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                 DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 100); // Set DMP FIFO rate to 100 Hz

    // accel task endless loop
    while (1) {
        
        // check for new data in FIFO buffer and update readings
        if (imu.fifoAvailable() && imu.dmpUpdateFifo() == INV_SUCCESS) {

            imu.computeEulerAngles(); // update quaternion

            Serial.print("roll = "); Serial.print(imu.roll);
            Serial.print(", pitch = "); Serial.print(imu.pitch);
            Serial.print(", yaw = "); Serial.println(imu.yaw);

        }

        // check remaining stack just in case
        UBaseType_t remStackBytes = uxTaskGetStackHighWaterMark(NULL);
        if (remStackBytes < 100) {
            Serial.print("Low remaining stack: "); Serial.println(remStackBytes);
        }
        

        vTaskDelay(5 / portTICK_PERIOD_MS); // non-blocking delay

    }

}

void setup() {
    Serial.begin(115200);

    delay(2000);
    while (!Serial); // wait for Serial to initialize

    Serial.println("Serial initialized successfully");

    xTaskCreate(
        accelThread,            // function to run
        "Accel Task",           // task name
        512,                   // stack size for task
        NULL,                   // parameters
        tskIDLE_PRIORITY + 1,   // priority
        &accelTaskHandle        // task handle address
    );

    vTaskStartScheduler(); // start RTOS

}

void loop() {
  // do nothing
}