// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define LED_PIN 13 //used for debugging status

//uncomment the base you're building
// #define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER            // 4WD robot
#define LINO_BASE MECANUM               // Mecanum drive robot

//uncomment the motor driver you're using
// #define USE_GENERIC_2_IN_MOTOR_DRIVER      // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
// #define USE_GENERIC_1_IN_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
#define USE_BTS7960_MOTOR_DRIVER        // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER            // Motor ESC for brushless motors

//uncomment the IMU you're using
// #define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU
#define USE_BNO055_IMU

#define K_P 65                            // P constant
#define K_I 110                           // I constant
#define K_D 0                             // D constant

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

//define your robot' specs here
#define MOTOR_MAX_RPS 8.4 //rps                   // motor's max RPM          
#define MAX_RPS_RATIO 0.9                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 24          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 23.5          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 2700              // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 2700              // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 2700              // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 2700              // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.1             // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.475          // distance between left and right wheels
#define PWM_BITS 8                          // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000                 // PWM Frequency

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV false 
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV false 
#define MOTOR4_ENCODER_INV false 

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV true
#define MOTOR2_INV true
#define MOTOR3_INV true
#define MOTOR4_INV true

// ENCODER PINS
#define MOTOR1_ENCODER_A 14
#define MOTOR1_ENCODER_B 15 

#define MOTOR2_ENCODER_A 11
#define MOTOR2_ENCODER_B 12 

#define MOTOR3_ENCODER_A 20
#define MOTOR3_ENCODER_B 21 

#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 10

#ifdef USE_BTS7960_MOTOR_DRIVER
  #define MOTOR1_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 1 // Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 instead.
  #define MOTOR1_IN_B 2 // Pin no 20 is not a PWM pin on Teensy 4.x, you can use pin no 0 instead.

  #define MOTOR2_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 3
  #define MOTOR2_IN_B 4

  #define MOTOR3_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23

  #define MOTOR4_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 5
  #define MOTOR4_IN_B 6

const int cw[4] = {MOTOR4_IN_A, MOTOR3_IN_A, MOTOR2_IN_A, MOTOR1_IN_A};
const int ccw[4] = {MOTOR4_IN_B, MOTOR3_IN_B, MOTOR2_IN_B,MOTOR1_IN_B};


  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif


#endif
