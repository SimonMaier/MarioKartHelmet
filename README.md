
# IMU-Joystick

**A configurable DIY game controller, based on an Arduino Pro Micro and the MPU6050 accelerometer & gyroscope.** 

:video_game:

For tested applications check out the `example`  section. In particular, `examples/MarioKartHelmet/` demonstrates how to set up  IMU-Joystick as a head-mounted game controller for Mario Kart 64 or Mario Kart Double Dash. It allows karts to be controlled through head movements and items triggered via jumping. After a lot of playing, we can assure that it brings a highly immersive game experience to what is already a great party game :tada:.

The Arduino Pro Micro-based controller can be assembled on a breadcoard within 15 minutes and costs less than 5$ (including the breadboard). When connected to a computer the board presents itself as a USB joystick. Using the MPU6050 sensor, the Arduino measures acceleration & rotation data and maps it to joystick behaviour. The controller is automatically detected as a generic joystick device on Linux, Mac, and Windows, without further software:

<p align="center">
  <img src="https://github.com/SimonMaier/MarioKartHelmet/blob/master/examples/demo1-jstest.gif" width="640" title="Joystick Demo using jstest-gtk" />
</p>



# How to build an IMU Joystick yourself

You can build your own IMU Joystick in about 1-2h, by following the instructions below. Except for mounting pin headers, no soldering is required.

### 1. Get the hardware components
  - [Arduino(-compatible) Pro Micro](https://www.aliexpress.com/item/New-Pro-Micro-for-arduino-ATmega32U4-5V-16MHz-Module-with-2-row-pin-header-For-Leonardo/32773740303.html), any ATmega32U4-based Arduino will do.
  - [GY-521](https://www.aliexpress.com/item/versandkostenfrei-gy-521-mpu-6050-mpu6050-modul-3-achse-analog-Gyro-Sensoren-beschleunigungsmesser-modul/32315092057.html), a low-cost MPU6050 accelerometer + gyroscope.
  - mini-sized [breadboard](https://www.aliexpress.com/item/Mini-Breadboard-Protoboard-DIY-Kit-Universal-Transparent-Solderless-SYB-170-Breadboard-170Tie-points-Prototype-Boards-35X47MM/32717999019.html)
  - (optional) [push buttons](https://www.aliexpress.com/item/100pcs-6-6-5mm-4pin-Quality-Mini-Micro-Momentary-Tactile-Push-Button-Switch/32753141267.html)

We assume that you have some wire to make breadboard connections.

**Total material cost** as of June 2017: €2,72 Pro Micro + €0.93 GY-521 + €0.80 breadboard = **4.45€**, shipping included.

### 2. Assemble the hardware
Solder pin headers where needed; Place Pro Micro, GY-521 & wire on breadboard as follows:
<p align="center">
  <img src="https://github.com/SimonMaier/MarioKartHelmet/blob/master/schematics/IMU-Joystick_bb.png" title="Breadboard Assembly" />
</p>
Note that Pro Micro pins `8` and `9` do not need to connect to the IMU.

### 3. Get the software components
- [Arduino IDE (>=1.6.6)](https://www.arduino.cc/en/main/software)
- external Arduino libraries:
  - [i2cdevlib](https://github.com/jrowberg/i2cdevlib)
    collection (at least [I2Cdev](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev) and [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)).
    Follow the [installation instructions on the website](https://www.i2cdevlib.com/usage), or copy the folders `i2cdevlib/Arduino/I2Cdev` and `i2cdevlib/Arduino/MPU6050` from the project repository into your Arduino [libraries directory](https://www.arduino.cc/en/hacking/libraries) (`~/Arduino/libraries/` on Linux).
  - [ArduinoJoystickLibrary (>=2.0)](https://github.com/MHeironimus/ArduinoJoystickLibrary)
    Copy the folder `ArduinoJoystickLibrary/Joystick` to your Arduino libraries directory.
- our Arduino code.
    
### 4. Upload the software

Install the required libraries and upload the sketch `IMU-Joystick.ino` to your Pro Micro board. If Pro Micro is not a possible  target in your Arduino environment, select Arduino Leonardo instead.

### 5. Enjoy!

We recommend that you play some *Mario Kart 64* with your IMU Joystick used as a head tracker (on ArchLinux).
Do the following steps:
  - Step 0: Upload the [MarioKartHelmet](examples/MarioKartHelmet/MarioKartHelmet.ino) sketch
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

# License

[GNU GPL v3](https://github.com/SimonMaier/MarioKartHelmet/blob/master/LICENSE) except for [README.md](https://github.com/SimonMaier/MarioKartHelmet/blob/master/README.md), which is released as [![License: CC BY-SA 4.0](https://licensebuttons.net/l/by-sa/4.0/80x15.png)](http://creativecommons.org/licenses/by-sa/4.0/).
