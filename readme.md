# INAV - navigation capable flight controller

![INAV](http://static.rcgroups.net/forums/attachments/6/1/0/3/7/6/a9088858-102-inav.png)

This modified unofficial version of INAV firmware brings back support for the MPU6050 as the main gyroscope of the flight controller. It also adds support for ADNS3080 as an optical flow sensor and nRF24L01+ as an SPI-based RX module.

Please note that the MPU6050 was removed starting from INAV 6.0.0 due to its limited sampling rate, internal DMP latency, and insufficient performance for modern high-loop-rate flight control. Despite these limitations, the MPU6050 can still be used reliably in low-performance, legacy, or experimental platforms, especially when operating at reduced loop frequencies and with conservative filter and PID settings.

This fork is intended primarily for:

- Legacy flight controller boards based on low-end F4 MCUs (STM32F411, STM32F405, etc)
- Educational and experimental platforms
- Resource-constrained builds where modern IMUs are unavailable
- Research and development use cases (optical flow, SPI RX experimentation)

**Important notes and limitations**

- This firmware is based on INAV 7.1.0 and is not compatible with newer INAV releases
- High gyro/loop rates are not recommended when using MPU6050
- This fork is not officially supported by the INAV project

Use at your own risk; thorough ground and hover testing is strongly recommended

**Added / restored features**

- Restored MPU6050 gyro support
- ADNS3080 optical flow support over SPI
- nRF24L01+ RX support over SPI

This project aims to extend the usable life of older hardware and provide a functional firmware option where official INAV no longer supports these components.

---
## HOW TO MAKE YOUR OWN BUILD

This fork follows the standard INAV build system and workflow, with additional target definitions for legacy hardware and SPI-based peripherals.

**1. Target creation and selection**

INAV uses target folders to describe flight-controller hardware (MCU, pin mapping, peripherals, sensors, etc.).
All modifications in this fork are implemented through custom targets, not by patching core logic.

*Creating a new target:*

Targets are located in: [src/main/target](https://github.com/MK384/inav-7.1.0-supports-legacy-hardware-and-nRF24/tree/Unofficial-7.1.0/src/main/target)

To create a new target:

- Copy an existing target that is close to your hardware (MCU family, pinout, flash size).
- Rename the folder to match your new target name (uppercase is recommended).
- Adjust the configuration files inside the folder to match your board.

**Example target: BLACKPILL_F411**

This repository includes an example target named: [BLACKPILL_F411](https://github.com/MK384/inav-7.1.0-supports-legacy-hardware-and-nRF24/tree/Unofficial-7.1.0/src/main/target/BLACKPILL_F411)

This target demonstrates:

- MPU6050 as the main gyro (I2C)
- ADNS3080 optical flow sensor over SPI
- nRF24L01+ RX module over SPI
- In addition to other INAV-supported hardware, such as BMP280, HMC5883L, VL53L1x, Neo-6M GPS, and SD-Card module, it serves as a blackbox.

![FC](https://github.com/user-attachments/assets/ef9e082b-ebb3-4491-8f6d-7b51fb34d089)
![GPS](https://github.com/user-attachments/assets/9172cd12-0d69-4dac-9f3a-cf85c3f654c4)
![SD-Card](https://github.com/user-attachments/assets/5c4be0f2-97b5-47c1-8e27-82f4e859fd7c)
![ADNS3080](https://github.com/user-attachments/assets/1a80c5de-dce3-48e7-94c5-2f3cc1fb8342)

If your hardware is similar, you can:

- Use this target as-is
- Or copy it and modify pin assignments and peripheral enables as needed. You can do so through the [target.h](https://github.com/MK384/inav-7.1.0-supports-legacy-hardware-and-nRF24/blob/Unofficial-7.1.0/src/main/target/BLACKPILL_F411/target.h) and [target.c](https://github.com/MK384/inav-7.1.0-supports-legacy-hardware-and-nRF24/blob/Unofficial-7.1.0/src/main/target/BLACKPILL_F411/target.c) files.

<img width="797" height="732" alt="image" src="https://github.com/user-attachments/assets/254409e2-06ca-4404-a44c-7a01c9b98c2a" />

**And here's a schematic for the current configuration if you aim to use it as it is**

// TO DO: add the FC schematic

**2. Required tools**

The build system is identical to upstream INAV and is based on GNU Make.

Operating system

- Linux (recommended)
- macOS
- Windows (via WSL or MSYS2)

Toolchain
- INAV requires a reasonably modern gcc-arm-none-eabi cross-compiler.
  
Note: To provide a uniform and reasonably modern cross compiler, INAV provides for the installation of a "known good / working" cross compiler, as well as a mechanism to override this if your distro provides a more modern option. That's it, the Inav MAKE script will install a suitable ARM cross compiler automatically. 

**In addition to a cross-compiler, it is necessary to install some other tools:**

- git : clone and manage the INAV code repository
- cmake : generate the build environment
- make : run the firmware compilation
- ruby : build some generated source files from JSON definitions
- gcc : native compiler used to generate settings and run tests

Note that INAV requires cmake version 3.13 or later; any distro that provides cmake 3.13 will also provide adequate versions of the other tools.

```
cd inav
# first time only, create the build directory
mkdir build
cd build
cmake ..
# note the "..", this is required as it tells cmake where to find its ruleset
```
Once cmake has generated the build/Makefile, this Makfile (with make) is used to build the firmware, again from the build directory.
```
make BLACKPILL_F411
```
The resultant hex file are in the build directory.

See: [Building in Linux.md](https://github.com/MK384/inav-7.1.0-supports-legacy-hardware-and-nRF24/blob/Unofficial-7.1.0/docs/development/Building%20in%20Linux.md) for more info.

![drone1](https://github.com/user-attachments/assets/b40e8c5c-77ec-41ad-8f63-cbff920c69c9)
![drone2](https://github.com/user-attachments/assets/99cf49c9-9865-4cbc-89cb-49369ecd94b3)
![drone3](https://github.com/user-attachments/assets/deafb0c5-a9b0-4715-b7b4-8683fecffc69)
![drone4](https://github.com/user-attachments/assets/765957ca-dd25-4ac0-b78b-a2b3249a0a3a)
