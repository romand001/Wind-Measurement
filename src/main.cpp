#include <Arduino.h>
#include <Seeed_Arduino_FreeRTOS.h>
#include <MPU9250.h>

TaskHandle_t accelTaskHandle;

MPU9250 mpu;

static void accelThread(void* pvParameters) {

    vNopDelayMS(1000); // wait another second for I2C

    // settings for MPU
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    // check if properly initialized
    while (!mpu.setup(0x68, setting)) {
        Serial.println("MPU connection failed.");
        vTaskDelay(3000 / portTICK_PERIOD_MS); // 3s delay
    }

    // accel task endless loop
    while (1) {

        if (mpu.update()) {
            Serial.print("yaw: "); Serial.print(mpu.getYaw()); Serial.print(", ");
            Serial.print("pitch: "); Serial.print(mpu.getPitch()); Serial.print(", ");
            Serial.print("roll: "); Serial.println(mpu.getRoll());
        }

        UBaseType_t remStackBytes = uxTaskGetStackHighWaterMark(NULL);
        if (remStackBytes < 100) {
            Serial.print("Low remaining stack: "); Serial.println(remStackBytes);
        }
        

        vTaskDelay(100 / portTICK_PERIOD_MS); // non-blocking delay

    }

}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    vNopDelayMS(1000); // NOP instructions for processor to avoid crash
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