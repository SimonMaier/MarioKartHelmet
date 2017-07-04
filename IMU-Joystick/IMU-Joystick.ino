#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h"
#include "Joystick.h"

#define DEBUG_OUTPUT  // use to receive serial debug output
#define JOYSTICK_MODE  // use to actually emulate a joystick, may be removed for debugging

#define INTERRUPT_PIN 7
#define BUTTON_PIN A0

#define trigger 4 // Arduino Pin an HC-SR04 Trig
#define echo 5    // Arduino Pin an HC-SR04 Echo


Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_MULTI_AXIS, 10, 0, true, true, false,  // 10 buttons (0,1..9), 2 axes (X,Y)
  false, false, false, false, false, false, false, false); // Nothing else

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// MPU orientation/motion vars
int16_t ax, ay, az, gx, gy, gz;  //             raw accel/gyro data
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/* UPDATE: not enough interrupt pins, poll button 
// ISR to indicate button push (calibration request)
volatile bool buttonInterrupt = false;
void buttonPushed() { buttonInterrupt = true; }  */
// ISR to indicate new MPU data in FIFO
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

bool startPressed = false;

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.

  pinMode(trigger, OUTPUT); // Set up trigger pin for the ultrasonic-sensor.
  pinMode(echo, INPUT); // Set up echo pin for the ultrasonic-sensor.
  digitalWrite(trigger, HIGH); //Shut down ultrasonic signal.
  
  mpu.initialize();

  #ifdef DEBUG_OUTPUT
      Serial.begin(115200);
      while (!Serial);
      Serial.println("Testing I2C connection...");
      Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  #endif
  
  pinMode(INTERRUPT_PIN, INPUT);
  #ifdef DEBUG_OUTPUT
      Serial.println("Initializing DMP...");
  #endif
  devStatus = mpu.dmpInitialize();

  calibrate_mpu();

  if (devStatus == 0) {
      #ifdef DEBUG_OUTPUT
          Serial.println("DMP Ready; Enabling DMP...");
      #endif
      mpu.setDMPEnabled(true);
      // set up ISR for MPU6050 FIFO interrupt
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize(); // expected DMP packet size for later comparison
  } else {
      #ifdef DEBUG_OUTPUT
          Serial.print("DMP Initialization failed (code ");
          Serial.print(devStatus);
          Serial.println(")");
     #endif
  }
    
  Joystick.setXAxisRange(-512, 511);
  Joystick.setYAxisRange(-512, 511);
  Joystick.begin(false);  // if false, Joystick.sendState() must be called to send updated values

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}



void loop() {
    if (!dmpReady) fail();
    
    // calibration button handling
    if (digitalRead(A0) == LOW && !startPressed) {
      calibrate_mpu(); // recompute offsets
      
      Joystick.pressButton(9); // press Start
      Joystick.sendState();
      startPressed = true;
    } else if (startPressed) {
      Joystick.releaseButton(9);
      Joystick.sendState();
      startPressed = false;
    }
      
     int trigger_offset = 10; // Define the offset for the ultrasonic-sensor (in cm.). 
     
     // Check wheather the distance of the players hand to the ultrasonic-sensor is lower than the given offset. 
     if (entfernung < trigger_offset) { 
      Joystick.pressButton(1); // Press the drift button.
      Joystick.sendState();  // Update the state of the controller.
      Serial.print(entfernung)
     }

     int entfernung=getEntfernung(); // Get the distance of the ultrasonic-sensor to the players hand.     
     // If the distance is bigger than 10 cm., release the button.
     else{
      Joystick.releaseButton(1); 
      Joystick.sendState(); // Update the state of the controller.
     }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) { }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // check for FIFO overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        #ifdef DEBUG_OUTPUT
            Serial.println("FIFO overflow!");
        #endif
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

        #ifdef DEBUG_OUTPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef JOYSTICK_MODE
            /***** mupen64plus button reference *****
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
            ************************************/
            Joystick.setXAxis((int)(-ypr[2]*1024/M_PI*2)); // *2 means that max value is achieved
            Joystick.setYAxis((int)( ypr[1]*1024/M_PI  )); // at around 45° tilt instead of 90°.

            
            if (ypr[1]*512/M_PI > -50) {  // Accelerate?
              Joystick.pressButton(6);
            } else {
              Joystick.releaseButton(6);
            }  
            
            if (ypr[1]*512/M_PI < -150) {  // Brake?
              Joystick.pressButton(8);
            } else {
              Joystick.releaseButton(8);
            }

            // Detect Jumping
            // compute world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation
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
            if (abs(aaWorld.z) > 3000) {  // Shoot?
              Joystick.pressButton(7);
            } else {
              Joystick.releaseButton(7);
            }
        
            Joystick.sendState(); // -- this is so slow that it causes FIFO overflows. (about 1 per call)
            #ifdef DEBUG_OUTPUT
                Serial.println("Joystick data sent.");
            #endif
        #endif
    }
}


void fail(void) {
  while (1) {
//    TXLED0;  // maybe triggers button press ISR?
    RXLED0;
    delay(250);
//    TXLED1;
    RXLED1;
    delay(250);
  }
}


void calibrate_mpu() { }

// Get the distance of the players hand to the ultrasonic-sensor.
int getEntfernung()
{
  // Initialize the variables used for the calculations.
  long distance=0;
  long travel_time=0;

  digitalWrite(trigger, LOW); // Reset the trigger pin.
  delayMicroseconds(3);
  noInterrupts();
  digitalWrite(trigger, HIGH); //Trigger Impuls for 10 us
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  travel_time = pulseIn(echo, HIGH); // Measure the echo time.
  interrupts();
  travel_time = (travel_time/2); // Only use half of the time as it travels two ways.
  distance = travel_time / 29.1; // Transfer time to cm.
  return(distance);
}

