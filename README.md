by hoahiepk10@gmail.com

Version: 1.0.1 (13/02/2019)

## Information
The software control six-mircophones array to wake up the device and trigger a signal to the mainboard. This also create LED RING effect, detect proximity and button event.

Base on HAL STM32Cube_FW_F7_V1.12.0

## BSP direction instruction
```
[BSP]
|─ Apps
|   |─ mic_array
|   └─ src
|─ Images
|─ Include
|─ Libraries
|   |─ BSP
|   |   |─ Components
|   |   └─ STM32746G-Discovery
|   |─ CMSIS
|   |   |─ DSP_Lib
|   |   |─ Include
|   |   └─ Lib
|   |─ FreeRTOS
|   |   |─ License
|   |   └─ Source
|   |─ HAL
|   |   └─ stm32f7xx
|   |       |─ inc
|   |       |─ script
|   |       |─ src
|   |       └─ startup
|   |─ STM32_USBH_Library
|   |   |─ Class
|   |   |─ Core
|   |   |─ Core_Device
|   |   └─ inc
|─ Makefile
└─ README
```
## Setup and compile BSP:

### Downloading the ARM Compiler

GNU Arm Embedded Toolchain version 7-2017-q4-major-linux
Link at: "https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads/7-2017-q4-major-1-1"

### Clone the BSP source code
```
git clone https://github.com/olli-ai/OLLI_STM32F7_Firmware.git
```
### Export Environment
```
cd OLLI_STM32F7_FIRMWARE
export ARCH=arm
export PATH=$PATH:<path/to/your/toolchain/folder>
```
### Building
```
make
```
### Flashing
#### PC Linux
Connect to ST-Link. Type command:
```
make flash
```
#### Mainboard
Copy binary file to mainboard using scp.
```
scp Images/stm32f7-firmware.bin root@your_maiboard_ip:/home/root
```
Use `DFU_FLASHING_INTO_INTERNAL_FLASH.sh` to write Image into Internal Flash

```
cd mixes
./DFU_FLASHING_INTO_INTERNAL_FLASH.sh /path/to/bin/file
For example:
./DFU_FLASHING_INTO_INTERNAL_FLASH.sh /home/root/stm32-stm32f7-firmware.bin
```
