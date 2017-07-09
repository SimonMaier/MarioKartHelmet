#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Joystick.h"

/*  A basic IMU-Joystick with a physical calibration push button.

    Works out of the box with most racing games, can be tested with e.g.
    extremetuxracer. */

// Initialize Joystick with no buttons and 2 axes (X,Y)
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
    JOYSTICK_TYPE_MULTI_AXIS, 0, 0, true, true, false,
    false, false, false, false, false, false, false, false);
#define SCALING 3        // max Joystick value  at 90deg/SCALING.

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // interrupt status byte from MPU
uint8_t devStatus;       // for return status checking (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // current FIFO length
uint8_t fifoBuffer[64];  // FIFO storage buffer
// MPU orientation/motion vars
Quaternion q;            // [w, x, y, z]
VectorFloat gravity;     // [x, y, z]
float ypr[3];            // [yaw, pitch, roll]
float pitchOffset = 0;   // set with push button press
float rollOffset = 0;

// ISR to indicate new MPU data
#define MPU_INTERRUPT_PIN 7
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

#define BUTTON_PIN A0
bool buttonPressed = false;


void setup() {

    // set up USB joystick
    Joystick.setXAxisRange(-512, 511);
    Joystick.setYAxisRange(-512, 511);
    // if Joystick is initiated with false, Joystick.sendState() must be called
    // to send updated values. Useful because state transmission is
    // comparatively slow and should be minimized!
    Joystick.begin(false);

    // set up push button
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // set up MPU6050
    pinMode(MPU_INTERRUPT_PIN, INPUT);
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        // set up ISR for MPU6050 FIFO "data ready"-interrupts
        attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN),
                        dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
       fail();
    }
}


void loop() {

    // push button: when released, take current pitch & roll as offset
    if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
        buttonPressed = true;
    } else if (digitalRead(BUTTON_PIN) == HIGH && buttonPressed) {
        pitchOffset = ypr[1];
        rollOffset = ypr[2];
        buttonPressed = false;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) { }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // check for DMP data ready interrupt (this should happen frequently)
    if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // process packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // SCALING of 3 means that max value is achieved
        // at around 30° tilt instead of 90°.
        Joystick.setXAxis((int)((ypr[2]-rollOffset)  * 1024/M_PI*SCALING));
        Joystick.setYAxis((int)((ypr[1]-pitchOffset) * 1024/M_PI*SCALING));
        Joystick.sendState(); // this is slow; might cause FIFO overflow.
    }

    // check for MPU FIFO overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    }
}

void fail() {
    while (1) {
        RXLED0;
        delay(250);
        RXLED1;
        delay(250);
    }
}
