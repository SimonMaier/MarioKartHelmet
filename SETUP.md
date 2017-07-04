# How to build an IMU Joystick yourself

You can build your own IMU Joystick in about 2h for less than 5€, by following the instructions below.

## 1. Get the hardware components
  - Arduino Pro Micro
  - GY-521 board (breakout for MPU6050)
  - breadboard
  - some wire
## 2. Assemble the hardware
Mount pin headers, place parts & wire on breadboard according to fritzing image.
## 3. Get the software components
- [Arduino IDE (>=1.6.6)](https://www.arduino.cc/en/main/software)
- external Arduino libraries:
  - [i2cdevlib](https://github.com/jrowberg/i2cdevlib)
    collection (at least [I2Cdev](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev) and [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)).

    Follow the [installation instructions on the website](https://www.i2cdevlib.com/usage), or copy the folders `i2cdevlib/Arduino/I2Cdev` and `i2cdevlib/Arduino/MPU6050` from the project repository into your Arduino [libraries directory](https://www.arduino.cc/en/hacking/libraries) (`~/Arduino/libraries/` on Linux).
  - [ArduinoJoystickLibrary (>=2.0)](https://github.com/MHeironimus/ArduinoJoystickLibrary)

    Copy the folder `ArduinoJoystickLibrary/Joystick` to your Arduino libraries directory.
- our Arduino code.
## 4. Upload the software
## 5. Test the Joystick
## 6. Enjoy!

# How to play *Mario Kart 64* with your IMU Joystick on ArchLinux
  - Step 1: `~# pacman -S mupen64plus`
  - Step 2: Add controller specification to `/usr/share/mupen64plus/InputAutoCfg.ini` file.
  - Step 3: Legally obtain ROM file for *Mario Kart 64*.
  - Step 4: `~# mupen64plus --fullscreen --gfx mupen64plus-video-glide64 <location-of-ROM-file.n64>`

# How to *develop* the controller yourself

This section lists some tools, that we found useful during development.
  - the linux command line tool package `joyutils` allow testing and simple debugging of joysticks, e.g. with
```bash
~$ jstest /dev/input/js0
```

  - the N64 emulator `mupen64plus` is useful to evaluate gameplay experience. Alternatively, the free game `extremetuxracer` also has Joystick support.
