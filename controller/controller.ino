// Arduino standard library imports
#include <Arduino.h>
#include <Wire.h>

// Custom imports
#include "controller.h"
#include "receiver.h"
#include "PilotCommandProcessor.h"
#include "PID.h"
#include "esc.h"

#include "MAF.h"
//#include "kinematics_ARG.h"
#include "kinematics_CMP.h"
#include "mpu6050.h"

// MAF definitions
MAF gyroXmaf(4);
MAF gyroYmaf(4);
MAF gyroZmaf(4);

MAF accelXmaf(4);
MAF accelYmaf(4);
MAF accelZmaf(4);

// MPU definitions
MPU6050 mpu;

// PID definitions
double YawCommandPIDSpeed, PitchCommandPIDSpeed, RollCommandPIDSpeed;
double YawMotorSpeed, PitchMotorSpeed, RollMotorSpeed;

PID yaw_command_pid(&kinematicsAngleZ, &YawCommandPIDSpeed, &commandYaw, 4.0, 0.0, 0.0, 25.0);
PID pitch_command_pid(&kinematicsAngleY, &PitchCommandPIDSpeed, &commandPitch, 4.0, 0.0, 0.0, 25.0);
PID roll_command_pid(&kinematicsAngleX, &RollCommandPIDSpeed, &commandRoll, 4.0, 0.0, 0.0, 25.0);

PID yaw_motor_pid(&gyroZsumRate, &YawMotorSpeed, &YawCommandPIDSpeed, 200.0, 5.0, 0.0, 1000.0);
PID pitch_motor_pid(&gyroYsumRate, &PitchMotorSpeed, &PitchCommandPIDSpeed, 80.0, 0.0, -300.0, 1000.0);
PID roll_motor_pid(&gyroXsumRate, &RollMotorSpeed, &RollCommandPIDSpeed, 80.0, 0.0, -300.0, 1000.0);
  
void setup() {    
    // Initialize serial communication
    Serial.begin(38400);   
 
    // Join i2c bus as master
    Wire.begin();

    // ESC timer and PIN setup
    setupFTM0();
    
    // RX timer and PIN setup
    setupFTM1();    
 
    // PIN settings
    pinMode(LED_PIN, OUTPUT); // build in status LED
    pinMode(LED_ORIENTATION, OUTPUT); // orientation lights
    
    // Initialize sensors
    mpu.initialize();
    mpu.calibrate_gyro();
    
    // All is ready, start the loop
    all_ready = true;
}

void loop() {   
    // Dont start the loop until everything is ready
    if (!all_ready) return; 
 
    // Used to measure loop performance
    itterations++;
    
    // Timer
    currentTime = micros();
    
    // Read data (not faster then every 1 ms)
    // Reading gyro and accel via i2c (consists of 12 bytes) takes about 1715us to complete (terrible)
    // This is as fast as we can go on standard i2c bus (we could go much faster on SPI)
    if (currentTime - sensorPreviousTime >= 1000) {
        unsigned long test_start = micros();
        
        mpu.readGyroRaw();
        mpu.readAccelRaw(); 
        
        
        gyroXsumRate = gyroXmaf.update(gyroX);
        gyroYsumRate = gyroYmaf.update(gyroY);
        gyroZsumRate = gyroZmaf.update(gyroZ);

        accelXsumAvr = accelXmaf.update(accelX);
        accelYsumAvr = accelYmaf.update(accelY);
        accelZsumAvr = accelZmaf.update(accelZ);        
        
        mpu.evaluateGyro();
        mpu.evaluateAccel();        
        
        kinematics_update(&accelXsumAvr, &accelYsumAvr, &accelZsumAvr, &gyroXsumRate, &gyroYsumRate, &gyroZsumRate);

        // 781 us
        // Serial.println(micros() - test_start);
        
        sensorPreviousTime = currentTime;
    }    
    
    // 100 Hz task loop (10 ms)
    if (currentTime - previousTime > 10000) {
        frameCounter++;
        
        process100HzTask();
        
        // 50 Hz tak (20 ms)
        if (frameCounter % TASK_50HZ == 0) {
            process50HzTask();
        }
        
        // 10 Hz task (100 ms)
        if (frameCounter % TASK_10HZ == 0) {
            process10HzTask();
        }  
        
        // 1 Hz task (1000 ms)
        if (frameCounter % TASK_1HZ == 0) {
            process1HzTask();
        }
        
        previousTime = currentTime;
    }
    
    if (frameCounter >= 100) {
        frameCounter = 0;
    }
}

void process400HzTask() {
    if (flightMode == ATTITUDE_MODE) {
        // Compute command PIDs (with kinematics correction)
        yaw_command_pid.Compute();
        pitch_command_pid.Compute();
        roll_command_pid.Compute();
        
        // Compute motor PIDs (rate)    
        yaw_motor_pid.Compute();
        pitch_motor_pid.Compute();
        roll_motor_pid.Compute();   
    } else if (flightMode == RATE_MODE) {
        // * 4.0 is the rotation speed factor
        YawCommandPIDSpeed = commandYaw * 4.0;
        PitchCommandPIDSpeed = commandPitch * 4.0;
        RollCommandPIDSpeed = commandRoll * 4.0;
        
        // Compute motor PIDs (rate)    
        yaw_motor_pid.Compute();
        pitch_motor_pid.Compute();
        roll_motor_pid.Compute();         
    }   
    
    if (armed) {               
        MotorOut[0] = constrain(TX_throttle + PitchMotorSpeed + RollMotorSpeed + YawMotorSpeed, 1000, 2000);
        MotorOut[1] = constrain(TX_throttle + PitchMotorSpeed - RollMotorSpeed - YawMotorSpeed, 1000, 2000);
        MotorOut[2] = constrain(TX_throttle - PitchMotorSpeed - RollMotorSpeed + YawMotorSpeed, 1000, 2000);
        MotorOut[3] = constrain(TX_throttle - PitchMotorSpeed + RollMotorSpeed - YawMotorSpeed, 1000, 2000);

        updateMotors();
    } else {
        MotorOut[0] = 1000;
        MotorOut[1] = 1000;
        MotorOut[2] = 1000;
        MotorOut[3] = 1000;
        
        updateMotors();
    } 
}

void process100HzTask() { 
    // temporary
    process400HzTask();
}

void process50HzTask() {
    processPilotCommands();
}

void process10HzTask() {
    // Trigger RX failsafe function every 100ms
    RX_failSafe();
    
    #ifdef DISPLAY_ITTERATIONS
        // Print itterations per 100ms
        Serial.println(itterations);
    #endif
    
    // Reset Itterations
    itterations = 0;    
}

void process1HzTask() {
    
    // Blink LED to indicated activity
    Alive_LED_state = !Alive_LED_state;
    digitalWrite(LED_PIN, Alive_LED_state);

    // Orientation ligts
    // also displaying armed / dis-armed status
    if (armed) {
        digitalWrite(LED_ORIENTATION, HIGH);
    } else {
        digitalWrite(LED_ORIENTATION, Alive_LED_state);
    }
}