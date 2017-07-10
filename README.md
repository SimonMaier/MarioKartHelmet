
# IMU-Joystick

**An easy-to-build DIY game controller for computers, based on the MPU6050 gyroscope & Arduino Pro Micro.** 

:video_game:

## Features

The IMU-Joystick is assembled in less than 15 minutes and costs about €5.
When connected to a computer, the board presents itself as a USB joystick. Using the MPU6050, the Arduino measures acceleration & rotation data and maps it to joystick behaviour. The precise mapping of sensor data to joystick axes/buttons can be configured in the Arduino sketch. The controller is automatically detected as a generic joystick device on Linux, Mac, and Windows, without further software:

<p align="center">
  <img src="https://github.com/SimonMaier/MarioKartHelmet/blob/master/examples/demo1-jstest.gif" width="640" title="Joystick Demo using jstest-gtk" />
</p>

## Application Examples

For applications check out the [examples](/examples) folder. 
  - `examples/BasicHeadTracker` sets up a generic Joystick with 2 axes and a physical calibration button. It works out of the box  with many games. Below you can watch a video of someone using a head-mounted IMU-Joystick with the `BasicHeadTracker` sketch to collect 89 herrings in *Extreme Tux Racer*:

<p align="center"><a href="https://www.youtube.com/watch?v=GGOQOM26r2M">
  <img src="https://img.youtube.com/vi/GGOQOM26r2M/0.jpg" title="ExtremeTuxRacer with IMU-Joystick video" width="640" />
</a></p>

  - `examples/MarioKartHelmet` demonstrates how to configure IMU-Joystick as a head-mounted game controller, intended for *Mario Kart 64* or *Mario Kart Double Dash*. It allows karts to be controlled through head movements and items triggered via jumping, by wearing a Toad-ish styrofoam hat with an IMU-Joystick inside. After a lot of playing, we can assure that it brings immersive game experience to what is already a great party game :tada:

<p align="center">
  <img src="http://chrisittner.de/frame-052.jpg" title="Mario Kart 64 with IMU-Joystick" width="640" />
</p>


## How to build an IMU-Joystick

You can put together your own IMU-Joystick in about 1-2h. Except for mounting pin headers, no soldering is required.

### 1. Get the hardware
  - [Arduino(-compatible) Pro Micro](https://www.aliexpress.com/item/New-Pro-Micro-for-arduino-ATmega32U4-5V-16MHz-Module-with-2-row-pin-header-For-Leonardo/32773740303.html), any ATmega32U4-based Arduino will do.
  - [MPU6050](https://www.aliexpress.com/item/versandkostenfrei-gy-521-mpu-6050-mpu6050-modul-3-achse-analog-Gyro-Sensoren-beschleunigungsmesser-modul/32315092057.html) (GY-521 breakout), a low-cost accelerometer + gyroscope.
  - mini-sized [breadboard](https://www.aliexpress.com/item/Mini-Breadboard-Protoboard-DIY-Kit-Universal-Transparent-Solderless-SYB-170-Breadboard-170Tie-points-Prototype-Boards-35X47MM/32717999019.html)
  - (optional) [push buttons](https://www.aliexpress.com/item/100pcs-6-6-5mm-4pin-Quality-Mini-Micro-Momentary-Tactile-Push-Button-Switch/32753141267.html)
  - some wire to make breadboard connections.

**Total material cost** as of June 2017: €2,72 Pro Micro + €0.93 GY-521 + €0.80 breadboard = **4.45€**, shipping included.

### 2. Assemble the hardware
Solder pin headers where needed; Place Pro Micro, GY-521 & wire on breadboard as follows:
<p align="center">
  <img src="https://github.com/SimonMaier/MarioKartHelmet/blob/master/schematics/IMU-Joystick_bb.png" title="Breadboard Assembly" />
</p>
Note that Pro Micro 8 & 9 do not need to connect to the MPU6050.

### 3. Get the software
- [Arduino IDE (>=1.6.6)](https://www.arduino.cc/en/main/software)
- external Arduino libraries:
  - [i2cdevlib](https://github.com/jrowberg/i2cdevlib)
    collection (at least [I2Cdev](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev) and [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)).
    
    Follow the [installation instructions on the website](https://www.i2cdevlib.com/usage), or copy the folders `i2cdevlib/Arduino/I2Cdev` and `i2cdevlib/Arduino/MPU6050` from the project repository into your Arduino [libraries directory](https://www.arduino.cc/en/hacking/libraries) (`~/Arduino/libraries/` on Linux).
  - [ArduinoJoystickLibrary (>=2.0)](https://github.com/MHeironimus/ArduinoJoystickLibrary)
  
    Copy the folder `ArduinoJoystickLibrary/Joystick` to your Arduino libraries directory.
- This repository. Copy any folder from `examples` into your Arduino directory.
    
### 4. Upload the software

Install the required libraries and upload the one of the example sketches to your Pro Micro board. Note that you might need to select Arduino Leonardo as a target board, when using the Arduino GUI.

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

## How to *develop* the controller

This section lists some tools, that we found useful during development.

  - the linux command line tool package `joyutils` allow testing and simple debugging of joysticks, e.g. with `~$ jstest /dev/input/js0`. A graphical version [jstest-gtk](https://github.com/Grumbel/jstest-gtk) is also available.
  - the N64 emulator `mupen64plus` is useful to evaluate gameplay experience. Alternatively, the free game `extremetuxracer` also has Joystick support.

## Issues & Contributing

If you have any questions or problems, feel free to communicate with us using Github Issues. Contributed PRs will be merged.

## Authors

 Chris, Tobias, Simon, Sean, and anybody in the contributers list.

## License

[GNU GPL v3](https://github.com/SimonMaier/MarioKartHelmet/blob/master/LICENSE), except for this introduction (`README.md`) which is [![License: CC BY-SA 4.0](https://licensebuttons.net/l/by-nc/4.0/80x15.png)](http://creativecommons.org/licenses/by-nc/4.0/).
