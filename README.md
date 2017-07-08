# Introduction:
### What is Mario-Helmet?

This Arduino Mario-Helmet project is designed for of playing Mario Kart with an interactive helmet controller rather than a traditional control handle. By turning the head, the user can simply control the direction and speed of the kart. Physically “Jump” can trigger the weapon system to shoot Red Koopa Shell or use other weapons. The player can also put his/her hand on the side of the helmet to drift the kart. In short, it is an easy-to-build and low-cost Arduino game controller project which brings the users a different game experience.


# How to build an IMU Joystick yourself

You can build your own IMU Joystick in about 2h for less than 5€, by following the instructions below.

### 1. Get the hardware components
  - [Arduino(-compatible) Pro Micro](https://www.aliexpress.com/item/New-Pro-Micro-for-arduino-ATmega32U4-5V-16MHz-Module-with-2-row-pin-header-For-Leonardo/32773740303.html), any ATmega32U4-based Arduino will do.
  - [GY-521](https://www.aliexpress.com/item/versandkostenfrei-gy-521-mpu-6050-mpu6050-modul-3-achse-analog-Gyro-Sensoren-beschleunigungsmesser-modul/32315092057.html), a low-cost MPU6050 accelerometer + gyroscope.
  - mini-sized [bread board](https://www.aliexpress.com/item/Mini-Breadboard-Protoboard-DIY-Kit-Universal-Transparent-Solderless-SYB-170-Breadboard-170Tie-points-Prototype-Boards-35X47MM/32717999019.html)
  - (optional) [push buttons](https://www.aliexpress.com/item/100pcs-6-6-5mm-4pin-Quality-Mini-Micro-Momentary-Tactile-Push-Button-Switch/32753141267.html)
  - some wire

**Total material cost** as of June 2017: €2,72 Pro Micro + €0.93 GY-521 + €0.80 bread board = **4.45€**, shipping included.

### 2. Assemble the hardware
Solder pin headers where needed; Place Pro Micro, GY-521 & wire on bread board as follows:
<p align="center">
  <img src="https://github.com/SimonMaier/MarioKartHelmet/blob/master/IMU-Joystick/schematics/IMU-Joystick_bb.png" title="Breadboard Assembly" />
</p>
Note that `` `` do not need to connect.

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

Install the required libraries and upload the sketch `IMU-Joystick.ino` to your Pro Micro board. If Pro Micro is not a possible  target in your Arduino environment, select Arduino Leonardo instead.

### 5. Enjoy!

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
  - the linux command line tool package `joyutils` allow testing and simple debugging of joysticks, e.g. with `~$ jstest /dev/input/js0`. A graphical version [jstest-gtk](https://github.com/Grumbel/jstest-gtk) is also available.
  - the N64 emulator `mupen64plus` is useful to evaluate gameplay experience. Alternatively, the free game `extremetuxracer` also has Joystick support.

