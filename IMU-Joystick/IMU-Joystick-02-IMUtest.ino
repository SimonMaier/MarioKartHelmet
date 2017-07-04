
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h"
#include "Joystick.h"

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_MULTI_AXIS, 4, 0, true, true, false,  // 4 buttons (0,1,2,3), 2 axes (X,Y)
  false, false, false, false, false, false, false, false); // Nothing else

MPU6050 mpu;  // I2C address is 0x68

#define INTERRUPT_PIN 7
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

#define SERIAL_OUTPUT_YAWPITCHROLL // for debugging, requires joystick_mode is commented out
// #define JOYSTICK_MODE //  actually send joyystick values

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
int16_t ax, ay, az, gx, gy, gz;  //             raw accel/gyro data
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     
void dmpDataReady() { mpuInterrupt = true; } // ISR to set mpuInterrupt

void fail(void) {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
}


void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.
  
  mpu.initialize();

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Testing I2C connection...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  pinMode(INTERRUPT_PIN, INPUT);
  // load and configure the DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

//////////////////////////7777777777
  // calibrate ----------- wait  10s???
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
////////////////////////////////777

    if (devStatus == 0) {
        Serial.println("DMP Ready; Enabling DMP...");
        mpu.setDMPEnabled(true);

        // set up ISR for MPU6050 FIFO interrupt
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize(); // expected DMP packet size for later comparison
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
    
  Joystick.setXAxisRange(-127, 127);
  Joystick.setYAxisRange(-127, 127);
  Joystick.begin(false);  // if false, Joystick.sendState() must be called to send updated values

  pinMode(A0, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);  // Onboard indicator LED
}



void loop() {
    if (!dmpReady) fail();
/////// TODO: calibration interrupt handling
//  // Button on A0 -Y reset (do with 2nd interrupt instead)
//  if (digitalRead(A0) != 0) {
//    digitalWrite(13, 0);
//    return;
//  }
// digitalWrite(LED_PIN, 1);



    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) { }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // check for FIFO overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // process packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #ifdef SERIAL_OUTPUT_YAWPITCHROLL
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef JOYSTICK_MODE
            Joystick.setXAxis((int)(ypr[1]*128/M_PI));
            Joystick.setYAxis((int)(ypr[2]*128/M_PI));
            Joystick.pressButton(1); //  button is int 0-4
        //  Joystick.releaseButton(button);
            //Joystick.sendState();
        #endif


        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

