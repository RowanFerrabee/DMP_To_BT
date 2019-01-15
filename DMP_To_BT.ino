
#include "IMU.h"

// Communication bridge between module and laptop for debugging
#include <SoftwareSerial.h>

#define BT_RST 4
#define SRL_RX 12
#define SRL_TX 11
#define MOD_BAUD 19200
#define BT_BAUD 19200

SoftwareSerial moduleSerial(SRL_RX, SRL_TX);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void initializeI2C() {
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}

void initializeDMP() {

  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220); //Gyroscope: X = -3396 | Y = 53 | Z = 45
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        moduleSerial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        moduleSerial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        moduleSerial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        moduleSerial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        moduleSerial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        moduleSerial.print(F("DMP Initialization failed (code "));
        moduleSerial.print(devStatus);
        moduleSerial.println(F(")"));
    }
}

void setup() {
    
    // Initialize serial pins
    pinMode(BT_RST, OUTPUT);
    pinMode(SRL_RX, INPUT);   // do we need these lines? software serial probably initializes it for us
    pinMode(SRL_TX, OUTPUT);

    digitalWrite(BT_RST, HIGH);
  
    // Initialize serial port
    Serial.begin(BT_BAUD);
    moduleSerial.begin(MOD_BAUD);

    delay(10);

    // initialize device
    moduleSerial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    moduleSerial.println(F("Testing device connections..."));
    moduleSerial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    moduleSerial.println(F("Initializing DMP..."));

    initializeDMP();
    
    
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        moduleSerial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // mpu.dmpGetEuler(euler, &q);
        // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        // mpu.dmpGetAccel(&aa, fifoBuffer);
        // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        float quat[] = {q.w, q.x, q.y, q.z};
        Serial.write((uint8_t*)quat, 16);
        moduleSerial.write((uint8_t*)quat, 16);
    }
}
