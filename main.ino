
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Keyboard.h>
#include "Wire.h"

MPU6050 accelgyro;  // default I2C address is 0x68
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;

int LEFT_THRESHHOLD = -5000;
int LEFT_THRESHHOLD_RELEASE = -5000;
int RIGHT_THRESHHOLD = 5000;
int RIGHT_THRESHHOLD_RELEASE = 5000;
int SPEED_TRESHHOLD = -6000;
int SLOW_THRESHHOLD = -2000;

char KEY_LEFT_ARROW_1 = 'j';
char KEY_RIGHT_ARROW_1 = 'l';
char KEY_UP_ARROW_1 = 'i';
char KEY_DOWN_ARROW_1 = 'k';


void setup() {
    Wire.begin();  // join I2C bus
    Keyboard.begin();

    Serial.begin(38400);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // trigger keypresses according to the sensor values
    perform_controls(ax, ay, az, gx, gy, gz);
    reset_controls(ax, ay, az, gx, gy, gz);
    shoot_item(gx, gy, gz);

    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}


void perform_controls(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    // check if the helmet is tilted to the left
    if (ay <  LEFT_THRESHHOLD) {
        Keyboard.press(KEY_LEFT_ARROW);
        Serial.println("Moving Left");
    }
    // check if the helmet is tilted to the right
    else if (ay > RIGHT_THRESHHOLD) {
        Keyboard.press(KEY_RIGHT_ARROW);
        Serial.println("Moving Right");
    }
      
    // Speed up if the helmet is tilted to the front
    if (ax < SPEED_TRESHHOLD) {
        Keyboard.press(KEY_LEFT_SHIFT);
        Serial.println("Speeding");
    }
    else if (ax > SLOW_THRESHHOLD) {
        Keyboard.press(KEY_LEFT_CTRL);
        Serial.println("Slowing down");
    }
}

void reset_controls(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    // check if the speed has to be reset
    if (ay > LEFT_THRESHHOLD_RELEASE && ay < RIGHT_THRESHHOLD_RELEASE) {
        Keyboard.release(KEY_RIGHT_ARROW);
        Keyboard.release(KEY_LEFT_ARROW);
        Serial.println("Reset");
    }
    if (ax > SPEED_TRESHHOLD && ax < SLOW_THRESHHOLD) {
        Keyboard.release(KEY_LEFT_SHIFT);
        Keyboard.release(KEY_LEFT_CTRL);
        Serial.println("Reset");
    }
}

void shoot_item(int16_t gx, int16_t gy, int16_t gz) {
    if (abs(gy) > 7000) {
      Keyboard.write('y');
      Serial.println("Shooting");
    }
}
