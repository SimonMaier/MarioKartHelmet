// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of the project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Keyboard.h"
#include "Wire.h"

#define LED_PIN 13
bool blinkState = false;

#define LEFT_THRESHHOLD          -7000
#define LEFT_THRESHHOLD_RELEASE  -6500
#define RIGHT_THRESHHOLD          7000
#define RIGHT_THRESHHOLD_RELEASE  6500
#define SPEED_TRESHHOLD          -6000
#define SLOW_THRESHHOLD          -2000

MPU6050 accelgyro;  // I2C address is 0x68
int16_t ax, ay, az, gx, gy, gz;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Wire.begin();
    Keyboard.begin();
    accelgyro.initialize();

    Serial.begin(115200);
    Serial.println("Testing I2C connection...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // trigger keypresses according to the sensor values
    perform_controls(ax, ay, az, gx, gy, gz);
    release_controls(ax, ay, az, gx, gy, gz);
    shoot_item(gx, gy, gz);

    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
}


void perform_controls(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    // Trigger keypresses in case helmet is tilted to left, right, front, or back.
    if (ay <  LEFT_THRESHHOLD) {
        Keyboard.press(KEY_LEFT_ARROW);
        Serial.println("Moving Left");
        digitalWrite(LED_PIN, 1);
    }
    else if (ay > RIGHT_THRESHHOLD) {
        Keyboard.press(KEY_RIGHT_ARROW);
        Serial.println("Moving Right");
        digitalWrite(LED_PIN, 1);
    }
    if (ax < SPEED_TRESHHOLD) {
        Keyboard.press(KEY_LEFT_SHIFT);
        Serial.println("Speeding");
    }
    else if (ax > SLOW_THRESHHOLD) {
        Keyboard.press(KEY_LEFT_CTRL);
        Serial.println("Slowing down");
    }
}

void release_controls(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    // release pressed keys when tilt back in normal range
    if (ay > LEFT_THRESHHOLD_RELEASE && ay < RIGHT_THRESHHOLD_RELEASE) {
        Keyboard.release(KEY_RIGHT_ARROW);
        Keyboard.release(KEY_LEFT_ARROW);
        Serial.println("Release Y");
        digitalWrite(LED_PIN, 0);
    }
    if (ax > SPEED_TRESHHOLD && ax < SLOW_THRESHHOLD) {
        Keyboard.release(KEY_LEFT_SHIFT);
        Keyboard.release(KEY_LEFT_CTRL);
        Serial.println("Release X");
    }
}

void shoot_item(int16_t gx, int16_t gy, int16_t gz) {
    if (abs(gy) > 7000) {
        Keyboard.write('y');
        Serial.println("Shooting");
    }
}
