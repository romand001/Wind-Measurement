#include <Arduino.h>
#include <Seeed_Arduino_FreeRTOS.h>
#include <SparkFunMPU9250-DMP.h>

#define WIND A1

#define G 9.807

TaskHandle_t comTaskHandle;
TaskHandle_t sensorTaskHandle;
SemaphoreHandle_t dirMutex;

// run tasks every __ ms
const TickType_t comFrequency = 100 / portTICK_RATE_MS;
const TickType_t sensorFrequency = 10 / portTICK_RATE_MS;

const uint32_t averageOver = 10; // take the average over this many sensor measurements
const float zeroWindVolts = 1.54;

MPU9250_DMP imu;

double phi;

static void comThread(void* pvParameters) {

    TickType_t xLastWakeTime; // loop start ticks

    xLastWakeTime = xTaskGetTickCount(); // get starting tick count

    while (1) {

        // check if GPS data incoming from DR2000
        if (Serial1.available()) {
            // parse GPS data here

        }

        // check remaining stack just in case
        UBaseType_t remStackBytes = uxTaskGetStackHighWaterMark(NULL);
        if (remStackBytes < 100) {
            Serial.print("comThread low rem stack: "); Serial.println(remStackBytes);
        }

        vTaskDelayUntil(&xLastWakeTime, comFrequency); // non-blocking rate delay
    }

}

static void sensorThread(void* pvParameters) {

    TickType_t xLastWakeTime; // loop start ticks

    analogReadResolution(12); // set ADC to 12 bits (0V-3.3V = 0-4095)

    // initialize IMU
    while (imu.begin() != INV_SUCCESS) {
        Serial.println("IMU setup failed");
        delay(3000);
    }
    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                 DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 100); // Set DMP FIFO rate to 100 Hz

    // variables for average filtering
    float avgRoll = 0.0, avgPitch = 0.0, avgYaw = 0.0, avgAx = 0.0, avgAy = 0.0;
    long avgTempRaw = 0;
    uint32_t avgWindRaw = 0;
    int i = 0;

    xLastWakeTime = xTaskGetTickCount(); // get starting tick count

    // accel task endless loop
    while (1) {
        
        // check for new data in FIFO buffer and update readings
        if (imu.fifoAvailable() && imu.dmpUpdateFifo() == INV_SUCCESS) {

            imu.computeEulerAngles(); // update quaternion
            imu.updateTemperature();

            // add measurements
            avgRoll += imu.roll; avgPitch += imu.pitch; avgYaw += imu.yaw;
            avgAx += imu.calcAccel(imu.ax); avgAy += imu.calcAccel(imu.ay); avgWindRaw += analogRead(WIND);
            avgTempRaw += imu.temperature;


            i++; // increment avg counter

            // enough values were added
            if (i == averageOver) {

                // unsigned long start = micros();

                // compute averages
                avgRoll /= averageOver; avgPitch /= averageOver; avgYaw /= averageOver;
                avgAx /= averageOver; avgAy /= averageOver; 
                avgWindRaw /= averageOver;
                avgTempRaw /= averageOver;

                float avgTemp = (float)(avgTempRaw) / 333.87 + 21.0; // convert raw temp data to degrees celsius

                float windVolts = avgWindRaw / 1240.91;
                // consider approximating this with integer math
                float windSpeed = 0.44704 * pow( (( (windVolts - zeroWindVolts) / ( 3.038517 * pow(avgTemp, 0.115157)) ) / 0.087288), 3.009364);
                if (isnan(windSpeed)) windSpeed = 0.0;

                // offset the pitch and roll
                if (avgPitch > 180) avgPitch -= 360;
                if (avgRoll > 180) avgRoll -= 360;

                // implement wind direction equation
                double num = cos(avgRoll) * (G * sin(avgPitch) - avgAy);
                double den = cos(avgPitch) * (G * sin(avgRoll) - avgAx);
                double phiTemp = atan( sqrt(num / den) ) * 180 / PI;

                if (isnan(phiTemp)) phiTemp = 0.0;

                // use mutex for updating global variable
                xSemaphoreTake(dirMutex, portMAX_DELAY);
                phi = phiTemp;
                xSemaphoreGive(dirMutex);

                Serial.print(avgRoll); Serial.print(", "); 
                Serial.print(avgPitch); Serial.print(", "); 
                Serial.print(avgYaw); Serial.print(", "); 
                Serial.print(windSpeed); Serial.print(", ");
                Serial.println(phiTemp);
                //Serial.println(avgTemp);



                // reset avg variables
                avgRoll = 0.0; avgPitch = 0.0; avgYaw = 0.0;
                avgAx = 0.0; avgAy = 0.0; avgWindRaw = 0.0; avgTempRaw = 0.0;
                i = 0;

                // unsigned long end = micros();
                // Serial.print("calcs took "); Serial.print((end - start)/1000); Serial.println(" ms");
            }


        }

        // check remaining stack just in case
        UBaseType_t remStackBytes = uxTaskGetStackHighWaterMark(NULL);
        if (remStackBytes < 100) {
            Serial.print("accelThread low rem stack: "); Serial.println(remStackBytes);
        }
        

        vTaskDelayUntil(&xLastWakeTime, sensorFrequency); // non-blocking rate delay

    }

}

void setup() {
    Serial.begin(115200); // USB port communication
    Serial1.begin(115200); // DR2000 UART

    delay(2000);
    while (!Serial); // wait for Serial to initialize

    Serial.println("Serial initialized successfully");

    dirMutex = xSemaphoreCreateMutex();

    // create task for handling IMU and calculating airspeed direction
    xTaskCreate(
        sensorThread,            // function to run
        "Accel Task",           // task name
        512,                    // stack size for task
        NULL,                   // parameters
        tskIDLE_PRIORITY + 2,   // priority
        &sensorTaskHandle        // task handle address
    );

    // create task for UART communication
    xTaskCreate(
        comThread,              // function to run
        "Communication Task",   // task name
        256,                    // stack size for task
        NULL,                   // parameters
        tskIDLE_PRIORITY + 1,   // priority
        &comTaskHandle          // task handle address
    );

    vTaskStartScheduler(); // start RTOS

}

void loop() {
  // do nothing
}