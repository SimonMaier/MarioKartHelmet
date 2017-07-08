# Introduction:
### What is Mario-Helmet?

This Arduino Mario-Helmet project is designed for of playing Mario Kart with an interactive helmet controller rather than a traditional control handle. By turning the head, the user can simply control the direction and speed of the kart. Physically “Jump” can trigger the weapon system to shoot Red Koopa Shell or use other weapons. The player can also put his/her hand on the side of the helmet to drift the kart. In short, it is an easy-to-build and low-cost Arduino game controller project which brings the users a different game experience.

# How to build an IMU Joystick yourself

You can build your own IMU Joystick in about 2h for less than 5€, by following the instructions below.

### 1. Get the hardware components
  - Helmet (Polylon hat is a good choice) 
  - Gyroscope (MPU6050 - Gy-521)
  - Ultrasonic Ranging Sensor (HC - SR04)
  - Arduino Pro Micro
  - Tiny Bread Board
  - USB cable (1m ~ 2m)

The main components needed for this projects are Arduino Pro Micro, Tiny Breadboard, Gyroscope (MPU6050 - GG521) and Ultrasonic Ranging Sensor (HC - SR04). MPU6050 is a 3-Axis Gyroscope and Accelerometer, which is used for detecting the axis change of player’s head movement to control the direction and the acceleration of jump to launch the weapon. HC-SR04 can determine the distance to an object like bats or dolphins do, it offers great non-contact range detection for the feature of putting the hand on the side of the helmet to drift.

### 2. Assemble the hardware
Solder pin headers, place Pro Micro, GY-521 & wire on breadboard as follows:
<p align="center">
  <img src="https://github.com/SimonMaier/MarioKartHelmet/blob/master/IMU-Joystick/schematics/IMU-Joystick_bb.png" title="Breadboard Assembly" />
</p>


### 3. Get the software components
- [Arduino IDE (>=1.6.6)](https://www.arduino.cc/en/main/software)
- external Arduino libraries:
  - [i2cdevlib](https://github.com/jrowberg/i2cdevlib)
    collection (at least [I2Cdev](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev) and [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)).

    Follow the [installation instructions on the website](https://www.i2cdevlib.com/usage), or copy the folders `i2cdevlib/Arduino/I2Cdev` and `i2cdevlib/Arduino/MPU6050` from the project repository into your Arduino [libraries directory](https://www.arduino.cc/en/hacking/libraries) (`~/Arduino/libraries/` on Linux).
  - [ArduinoJoystickLibrary (>=2.0)](https://github.com/MHeironimus/ArduinoJoystickLibrary)

    Copy the folder `ArduinoJoystickLibrary/Joystick` to your Arduino libraries directory.
- our Arduino code.

- N64 emulation - Mupen64Plus (http://mupen64plus.org/)
    
### 4. Upload the software

### 5. Test the Joystick (Helmet)

### 6. Enjoy!

We recommend that you play some *Mario Kart 64* with your IMU Joystick used as a head tracker (on ArchLinux).
Do the following steps:
  - Step 1: `~# pacman -S mupen64plus`
  - Step 2: Add controller specification to `/usr/share/mupen64plus/InputAutoCfg.ini` file.
  - Step 3: Legally obtain ROM file for *Mario Kart 64*.
  - Step 4: Connect controller & carefully place it on top of your head
  - Step 5: `~# mupen64plus --fullscreen --gfx mupen64plus-video-glide64 <location-of-ROM-file.n64>`
  - Step 6: \o/ Control race cars using your head movements.

# How to *develop* the controller yourself

This section lists some tools, that we found useful during development.
  - the linux command line tool package `joyutils` allow testing and simple debugging of joysticks, e.g. with
```bash
~$ jstest /dev/input/js0
```

  - the N64 emulator `mupen64plus` is useful to evaluate gameplay experience. Alternatively, the free game `extremetuxracer` also has Joystick support.

