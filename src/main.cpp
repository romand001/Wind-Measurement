#include <Arduino.h>
#include <Seeed_Arduino_FreeRTOS.h>
#include <Wire.h>
// #include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"

#define WIND A1

#define G 9.807
#define MAGDEC 10.3 // magnetic declination in Toronto

RTIMU *imu;
RTFusionRTQF fusion;
RTIMUSettings settings;

TaskHandle_t comTaskHandle;
TaskHandle_t sensorTaskHandle;
SemaphoreHandle_t dirMutex;

// run tasks every __ ms
const TickType_t comFrequency = 100 / portTICK_RATE_MS;
const TickType_t sensorFrequency = 10 / portTICK_RATE_MS;

const uint32_t averageOver = 10; // take the average over this many sensor measurements
const float zeroWindVolts = 1.54;

const float pi = 3.14159;

volatile double phi;

/*
__attribute__((optimize("O3"))) void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }

*/


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
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C
    delay(500);
    
    imu = RTIMU::createIMU(&settings);

    int errcode;
    Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if (imu->getCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");


    float roll = 0.0, pitch = 0.0, yaw = 0.0, lin_ax = 0.0, lin_ay = 0.0, temperature = 0.0;


    xLastWakeTime = xTaskGetTickCount(); // get starting tick count

    // accel task endless loop
    while (1) {

        // unsigned long start = micros();

        if (imu->IMURead()) {

            fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());

            // if (imu->IMUGyroBiasValid())
            //     Serial.println("gyro bias valid");
            // else
            //     Serial.println("calculating gyro bias");

            RTVector3 accel = imu->getAccel();
            RTVector3 euler = fusion.getFusionPose();

            roll = euler.x(); pitch = euler.y(); yaw = euler.z();
            lin_ax = accel.x(); lin_ay = accel.y();

            // need to read temperature too (use kris winer method)

            // RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose());

        }

        float windVolts = analogRead(WIND) / 1240.91;
        // consider approximating this with integer math
        float windSpeed = 0.44704 * pow( (( (windVolts - zeroWindVolts) / ( 3.038517 * pow(temperature, 0.115157)) ) / 0.087288), 3.009364);
        if (isnan(windSpeed)) windSpeed = 0.0;

        // offset the pitch and roll
        // if (avgPitch > 180) avgPitch -= 360;
        // if (avgRoll > 180) avgRoll -= 360;

        // implement wind direction equation
        double num = cos(roll * pi/180) * (G * sin(pitch * pi/180) - lin_ay);
        double den = cos(pitch * pi/180) * (G * sin(roll * pi/180) - lin_ax);
        double phiTemp = atan( sqrt(num / den) ) * 180 / pi;

        if (isnan(phiTemp)) phiTemp = 0.0;

        // use mutex for updating global variable
        xSemaphoreTake(dirMutex, portMAX_DELAY);
        phi = phiTemp;
        xSemaphoreGive(dirMutex);

        Serial.print(roll); Serial.print(", "); 
        Serial.print(pitch); Serial.print(", "); 
        Serial.println(yaw);// Serial.print(", "); 
        // Serial.print(windSpeed); Serial.print(", ");
        // Serial.print(phiTemp); Serial.print(", ");
        // Serial.println(temperature);


        // unsigned long end = micros();
        // Serial.print("calcs took "); Serial.print((end - start)/1000); Serial.println(" ms");


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