#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" // from i2cdevlib
//#include "MPU6050.h"
#include "Joystick.h"

// (un)comment as needed:
#define JOYSTICK_MODE     // send IMU Joystick commands to PC?
//#define DEBUG_OUTPUT    // receive serial debug output?
                          // this may break device recognition by PC.
//#define ULTRASONIC_MODE // use ultrasonic sensor (for additional buttons)?


// Initialize Joystick with 10 buttons (0,1..9) and 2 axes (X,Y)
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
    JOYSTICK_TYPE_MULTI_AXIS, 10, 0, true, true, false,
    false, false, false, false, false, false, false, false);
// Joystick status vars
bool driftButtonPressed = false;
bool startButtonPressed = false;
unsigned long startButtonPressedTime;

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // interrupt status byte from MPU
uint8_t devStatus;       // for return status checking (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // current FIFO length
uint8_t fifoBuffer[64];  // FIFO storage buffer
// MPU orientation/motion vars
Quaternion q;            // [w, x, y, z]         quaternion container
VectorInt16 aa;          // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;      // [x, y, z]            gravity-free accel sensor data
VectorInt16 aaWorld;     // [x, y, z]            world-frame accel sensor data
VectorFloat gravity;     // [x, y, z]            gravity vector
float ypr[3];            // [yaw, pitch, roll]   yaw/pitch/roll container
float avgYpr[3];         // [yaw, pitch, roll]   stores cumulative average ypr
#define Y_AVG_WINDOW 2000 // window size of the moving averages
#define P_AVG_WINDOW 125
#define R_AVG_WINDOW 2000
bool initAvg = false;

// ISR to indicate new MPU data
#define MPU_INTERRUPT_PIN 7
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// HC-SR04 Ultrasonic vars
#define US_TRIGGER_PIN 4
#define US_ECHO_PIN 5
#define US_TRIGGER_THRESHOLD 10  // threshhold distance for button push (in cm)

#define BUTTON_PIN A0



void setup() {

    // (0) set up USB joystick
    Joystick.setXAxisRange(-512, 511);
    Joystick.setYAxisRange(-512, 511);
    // if Joystick is initiated with false, Joystick.sendState() must be called
    // to send updated values. Useful because state transmission is
    // comparatively slow and should be minimized!
    Joystick.begin(false);

    // (1) set up push button
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // (2) set up distance sensor
    #ifdef ULTRASONIC_MODE
        pinMode(US_TRIGGER_PIN, OUTPUT);
        pinMode(US_ECHO_PIN, INPUT);
        digitalWrite(US_TRIGGER_PIN, LOW); // shut off any ultrasonic signal.
    #endif

    // (3) set up MPU6050
    pinMode(MPU_INTERRUPT_PIN, INPUT);
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock.
    mpu.initialize();
    #ifdef DEBUG_OUTPUT
        Serial.begin(115200);
        while (!Serial);
        Serial.println("Testing I2C...");
        Serial.println(mpu.testConnection() ?
                "MPU6050 connected" : "MPU6050 connection failed");
    #endif

    devStatus = mpu.dmpInitialize();
    #ifdef DEBUG_OUTPUT
        Serial.println("Initializing DMP...");
    #endif
    if (devStatus == 0) {
        #ifdef DEBUG_OUTPUT
            Serial.println("DMP Ready; Enabling DMP...");
        #endif
        mpu.setDMPEnabled(true);
        // set up ISR for MPU6050 FIFO "data ready"-interrupts
        attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN),
                        dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        #ifdef DEBUG_OUTPUT
            Serial.print("DMP Initialization failed (code ");
            Serial.print(devStatus);
            Serial.println(")");
       #endif
       fail();
    }
}



void loop() {

    // (1) push button handling
    if (digitalRead(BUTTON_PIN) == LOW && !startButtonPressed) {
        Joystick.pressButton(9);
        Joystick.sendState();
        startButtonPressed = true;
        startButtonPressedTime = millis();
    } else if (digitalRead(BUTTON_PIN) == HIGH && startButtonPressed) {
        Joystick.releaseButton(9);
        Joystick.sendState();
        startButtonPressed = false;

        // long press -> recalibrate helmet
        if(millis() - startButtonPressedTime > 1000) {
            // reset roll (i.e. set current joystick position to normal)
            avgYpr[2]=ypr[2];
        }
    }

    // (2) distance sensor handling
    #ifdef ULTRASONIC_MODE
        if (getDistance() < US_TRIGGER_THRESHOLD) {
            Joystick.pressButton(1);
            Joystick.sendState();
            driftButtonPressed = true;
            #ifdef DEBUG_OUTPUT
                Serial.println("Pressing drift");
            #endif
        } else if (driftButtonPressed) {
            Joystick.releaseButton(1);
            Joystick.sendState();
            driftButtonPressed = false;
        }
    #endif

    // (3) IMU sensor handling

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) { }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // check for DMP data ready interrupt (this should happen frequently)
    if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // process packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // update moving averages
        if (initAvg) {
            // UPDATE: only use cumulative avgs for pitch in Mario Kart DD GC
            //avgYpr[0] = (ypr[0] + avgYpr[0]*Y_AVG_WINDOW) / (Y_AVG_WINDOW+1);
            avgYpr[1] = (ypr[1] + avgYpr[1]*P_AVG_WINDOW) / (P_AVG_WINDOW+1);
            //avgYpr[2] = (ypr[2] + avgYpr[2]*R_AVG_WINDOW) / (R_AVG_WINDOW+1);
        } else {
            avgYpr[0] = ypr[0];
            avgYpr[1] = ypr[1];
            avgYpr[2] = ypr[2];
            initAvg = true;
        }

        #ifdef DEBUG_OUTPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef JOYSTICK_MODE
            /***** mupen64plus button reference
              Start = button(9)
              Z Trig = button(7)
              B Button = button(8)
              A Button = button(6)
              C Button R = button(1)
              C Button L = button(3)
              C Button D = button(2)
              C Button U = button(0)
              R Trig = button(5)
              L Trig = button(4)
              Y Axis = axis(1-,1+)
              X Axis = axis(0-,0+)
            **********************************/

            // *3 means that max value is achieved
            // at around 30° tilt instead of 90°.
            Joystick.setXAxis((int)((ypr[2]-avgYpr[2])*1024/M_PI*3));
            Joystick.setYAxis((int)((ypr[1]-avgYpr[1])*1024/M_PI*3));

            if (-(ypr[1]-avgYpr[1])*1024/M_PI > -30) {  // Accelerate?
              Joystick.pressButton(6);
            } else {
              Joystick.releaseButton(6);
            }

            if (-(ypr[1]-avgYpr[1])*1024/M_PI <= -60) {  // Brake?
              Joystick.pressButton(8);
            } else {
              Joystick.releaseButton(8);
            }

            // For jump detection: compute acceleration values adjusted to
            // remove gravity and rotated based on known orientation
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            #ifdef DEBUG_OUTPUT
                Serial.print("acceleration\t");
                Serial.print(aaWorld.x);
                Serial.print("\t");
                Serial.print(aaWorld.y);
                Serial.print("\t");
                Serial.println(aaWorld.z);
            #endif

            if (-aaWorld.z > 2500) {  // Use Item?
              Joystick.pressButton(7);
            } else {
              Joystick.releaseButton(7);
            }

            Joystick.sendState(); // this is slow; might cause FIFO overflow.
            #ifdef DEBUG_OUTPUT
                Serial.println("Joystick data sent.");
            #endif
        #endif
    }

    // check for MPU FIFO overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        #ifdef DEBUG_OUTPUT
            Serial.println("FIFO overflow!");
        #endif
    }
}


// returns distance of first obstacle to  ultrasonic sensor, in cm.
int getDistance() {
    long distance = 0;
    long travel_time = 0;

    digitalWrite(US_TRIGGER_PIN, LOW);
    delayMicroseconds(3);
    noInterrupts();
    digitalWrite(US_TRIGGER_PIN, HIGH); // trigger impulse for 1us.
    delayMicroseconds(10);
    digitalWrite(US_TRIGGER_PIN, LOW);
    // travel time is half of round-trip time returned by pulseIn().
    travel_time = pulseIn(US_ECHO_PIN, HIGH) / 2;
    interrupts();
    distance = travel_time / 29.1; // transfer travel time to cm.
    return(distance);
}

void fail() {
    while (1) {
        RXLED0;
        delay(250);
        RXLED1;
        delay(250);
    }
}
