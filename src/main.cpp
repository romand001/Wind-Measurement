#include <Arduino.h>
#include <Seeed_Arduino_FreeRTOS.h>
#include <Wire.h>
#include <MPU9250.h>
//#include <MadgwickFilter.h>

#define WIND A1

#define G 9.807
#define MAGDEC 10.3 // magnetic declination in Toronto

uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;         
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float motion = 0; // check on linear acceleration to determine motion
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
bool wakeup;

bool newMagData = false;

int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float   magCalibration[3] = {0.5, 0.5, 0.5};  // Factory mag calibration and mag bias
float   temperature;    // Stores the MPU9250 internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// These can be measured once and entered here or can be calculated each time the device is powered on
float   gyroBias[3] = {12.81, -7.79, -4.85}, accelBias[3] = {-5.25, -8.54, -76.66};
float   magBias[3] = {71.04, 122.43, -36.90}, magScale[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer


uint32_t delt_t = 0;                      // used to control display output rate
uint32_t count = 0, sumCount = 0;         // used to control display output rate
float pitch, yaw, roll;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

MPU9250 imu(420); // instantiate MPU9250 class


TaskHandle_t comTaskHandle;
TaskHandle_t sensorTaskHandle;
SemaphoreHandle_t dirMutex;

// run tasks every __ ms
const TickType_t comFrequency = 100 / portTICK_RATE_MS;
const TickType_t sensorFrequency = 10 / portTICK_RATE_MS;

const uint32_t averageOver = 10; // take the average over this many sensor measurements
const float zeroWindVolts = 1.54;

volatile double phi;

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
    // imu.I2Cscan();
    // Serial.println("MPU9250 9-axis motion sensor...");
    uint8_t c = imu.getMPU9250ID();
    // Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x70, HEX);
    // delay(1000);

    // somehow WHO_AM_I got changed to 0x70
    if (c == 0x70 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
    {  
        // Serial.println("MPU9250 is online...");
        
        imu.resetMPU9250(); // start by resetting MPU9250
        
        // imu.SelfTest(SelfTest); // Start by performing self test and reporting values
        // Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
        // Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
        // Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
        // Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
        // Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
        // Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
        delay(500);

        // get sensor resolutions, only need to do this once
        aRes = imu.getAres(Ascale);
        gRes = imu.getGres(Gscale);
        mRes = imu.getMres(Mscale);

        // Comment out if using pre-measured, pre-stored offset biases
        // imu.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        // Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
        // Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
        // delay(1000); 
        
        imu.initMPU9250(Ascale, Gscale, sampleRate); 
        Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        
        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        byte d = imu.getAK8963CID();  // Read WHO_AM_I register for AK8963
        // Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
        delay(500); 
        
        // Get magnetometer calibration from AK8963 ROM
        imu.initAK8963Slave(Mscale, Mmode, magCalibration); 
        // Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

        // Comment out if using pre-measured, pre-stored offset biases
        //  MPU9250.magcalMPU9250(magBias, magScale);
        // Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]); 
        // Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
        // delay(2000); // add delay to see results before serial spew of data

        // Serial.println("Calibration values: ");
        // Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
        // Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
        // Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
    
    
    }
    else
    {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);
        while(1) ; // Loop forever if communication doesn't happen
    }

    xLastWakeTime = xTaskGetTickCount(); // get starting tick count

    // accel task endless loop
    while (1) {

        //unsigned long start = micros();
        
        imu.readMPU9250Data(MPU9250Data);
   
        // Now we'll calculate the accleration value into actual g's
        ax = (float)MPU9250Data[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)MPU9250Data[1]*aRes - accelBias[1];   
        az = (float)MPU9250Data[2]*aRes - accelBias[2];  

        // Calculate the gyro value into actual degrees per second
        gx = (float)MPU9250Data[4]*gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)MPU9250Data[5]*gRes;  
        gz = (float)MPU9250Data[6]*gRes; 
    
        // if( MPU9250.checkNewMagData() == true) { // wait for magnetometer data ready bit to be set
        imu.readMagData(magCount);  // Read the x/y/z adc values
    
        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
        mx *= magScale[0];
        my *= magScale[1];
        mz *= magScale[2]; 
    
    
        for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
            Now = micros();
            deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
            lastUpdate = Now;

            sum += deltat; // sum for averaging filter update rate
            sumCount++;

            MadgwickQuaternionUpdate(-ax, +ay, +az, gx*pi/180.0f, -gy*pi/180.0f, -gz*pi/180.0f,  my,  -mx, mz);
        }

        temperature = ((float) MPU9250Data[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
        //Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C     

        a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
        a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
        a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
        a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        pitch = -asinf(a32);
        roll  = atan2f(a31, a33);
        yaw   = atan2f(a12, a22);
        pitch *= 180.0f / pi;
        yaw   *= 180.0f / pi; 
        yaw   += MAGDEC; 
        if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
        roll  *= 180.0f / pi;
        lin_ax = ax + a31;
        lin_ay = ay + a32;
        lin_az = az - a33;

        //sumCount = 0; 
        //sum = 0;

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
        Serial.print(yaw); Serial.print(", "); 
        Serial.print(windSpeed); Serial.print(", ");
        Serial.print(phiTemp); Serial.print(", ");
        Serial.println(temperature);


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