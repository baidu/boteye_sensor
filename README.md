## boteye_sensor ##
This repo contains hardware and firmware of Baidu boteye sensor.  
**If you purchase the [hardware from Kinco Automation](http://www.kinco.cn/solution/detail?id=36&type=all)(a third party manufacturer), which is already loaded with the latest firmware, you can skip the following steps. **

## Hardware Introduction

Firmware supports three hardware versions: `XP`, `XP2`, and `XP3`.

## FX3 SDK Installation ##
  In order to compile, you need cypress FX3 sdk (http://www.cypress.com/documentation/software-and-drivers/ez-usb-fx3-software-development-kit).

  Download the sdk here http://www.cypress.com/file/221546

  You can get `fx3_firmware_linux.tar.gz` and `cyusb_linux_1.0.4.tar.gz`, then extract to root directory of this repo.
- Install arm gcc compiler

  ```bash
  sudo apt-get install gcc-arm-none-eabi
  ```

- Make sure your arm compiler is `arm-none-eabi-gcc version 4.8.2`

  ```bash
  arm-none-eabi-gcc --version
  ```
- Install gcc compiler

  ```bash
  sudo apt-get install build-essential
  ```

- Modify SDK config file

  - Configure build type in the file `cyfx3sdk/fw_build/fx3_fw/fx3_build_config.mak`

    Change `CYCONFOPT=fx3_debug` to `CYCONFOPT=fx3_release`

  - Configure the arm compiler info in the file `cyfx3sdk/fw_build/fx3_fw/fx3_armgcc_config.mak`

    Change `LDLIBS += ` section to your lib locations, most likely located in `/usr/arm-linux-gnueabi/lib/libc.a` and `/usr/lib/gcc/arm-none-eabi/4.8.2/libgcc.a`, and replace the default like this:
    ```bash
    The ARM toolchain location and the version are taken from environment variables
    LDLIBS  += \
    /usr/lib/arm-none-eabi/lib/libc.a \
    /usr/lib/gcc/arm-none-eabi/4.8.2/libgcc.a \

    EXEEXT		= elf
    ```
- Compile img format convert tools: `elf2img`

  To convert elf to img file, you need to compile `cyfx3sdk/util/elf2img/elf2img.c`
  ```bash
  cd cyfx3sdk/util/elf2img
  gcc elf2img.c -o elf2img -O3
  ```

## Compile Firmware ##
- If everything is all set, you can run `./build.sh` within `boteye_sensor/firmware`, which generates  `cyfxuvc.img`.

## FX3 Image Download Tool Install ##
- Install `libusb-dev`

  ```bash
  sudo apt-get install libusb-1.0-0-dev
  ```

- Install FX3 tools, please read the README in `./cyusb_linux_1.0.4`.

  ```bash
  cd cyusb_linux_1.0.4
  make
  chmod +x install.sh
  sudo ./install.sh
  ```
- If you find an error `qmake-qt4: command not found`, please install qt4 by running

  ```bash
  sudo apt-get install qt4-dev-tools qt4-doc qt4-qtconfig qt4-demos qt4-designer
  ```
- Add execute premission of `cyusb_linux`
  ```bash
  sudo chmod +x /usr/local/bin/cyusb_linux
  ```

- Add the following two lines to `~/.bashrc`, run `source ~/.bashrc`, and assign `CYUSB_ROOT` to the corresponding file path.

  ```bash
  export CYUSB_ROOT=~/Development/sensor/cyusb_linux_1.0.4/
  export LD_LIBRARY_PATH=/user/local/lib/
  source ~/.bashrc
  ```
## Image Download ##

  - Use `cyusb_linux` to flash your img file to the board flash. If the sensor has an img on flash, it will
  go into auto-run mode with red LED blinking for 1s. If the sensor doesn't have an img on flash, the red LED will
  be always on.  We can use `./sensor/host_bin/spi_test` to erase the img on flash so that we can download a new one.
  - example: `./sensor/host_bin/spi_test /dev/video0`

## Firmware Tools ##
- There are two tools you can use in `firmware/host_bin`.
  - `spi_test`: erase flash
  - `version_test`: read hardware and software version.
- You can also install `luvcview` or other camera view tools to test the camera. However, you will most likely see green and red ghost images due to the custom formatting of the stereo raw images.

## About Version ##
- Hardware version:
  - You can use command `dmesg` to check USB `Product` attribute, it will show the hardware version.
  - Example: `Baidu_Robotics_vision_XP2`
- Software version:
  - You can also use `dmesg` to check USB `SeriaNumber` attribute, it will show the software version.
  - Example: `V0.3.7-839610e-commit` [version num - commit hash - commit/dirty]
