
#ifndef IMU_H
#define IMU_H

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include "helper_3dmath.h"
#include "MPU6050_6Axis_MotionApps20.h"

// -- Exported Defines Declarations --

#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// -- Variable Declarations --

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
extern MPU6050 mpu;

// MPU control/status vars
extern bool dmpReady;  // set true if DMP init was successful
extern uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
extern uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
extern uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
extern uint16_t fifoCount;     // count of all bytes currently in FIFO
extern uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
extern Quaternion q;           // [w, x, y, z]         quaternion container
extern VectorInt16 aa;         // [x, y, z]            accel sensor measurements
extern VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
extern VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
extern VectorFloat gravity;    // [x, y, z]            gravity vector
extern float euler[3];         // [psi, theta, phi]    Euler angle container
extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// -- Function Declarations --
bool initializeDMP();

// Interrupt Detection Routine
extern volatile bool mpuInterrupt;     // indicates whether MPU interrupt pin has gone high
//void dmpDataReady();/


#endif
