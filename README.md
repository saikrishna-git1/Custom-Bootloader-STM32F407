# Custom-Bootloader-STM32F407
Custom bootloader project for Cortex-M4 based STM32F407VG microcontroller on STM32F407G-DISC1 board.

## Docs to refer
1. RM0090 - Reference manual STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced Arm®-based 32-bit MCUs  
   section 2.4 - Boot configuration
2. AN2606 - STM32 microcontroller system memory boot mode  
   section 28.1.1 - STM32F40xxx/41xxx devices -> Bootloader configuration
3. UM1472 - Discovery kit with STM32F407VG MCU  
   section 6.1.3 - ST-LINK/V2-A VCP configuration
4. DS8626 - Datasheet STM32F405xx STM32F407xx  
   section 3 - Pinouts
5. Schematic - en.MB997-F407VGT6-E01_Schematic  
   BOOT0, BOOT1/PB2 pins
   


## Hardware and Software
1. STM32F407G-DISC1 board
2. USB to UART converter - 2 nos (extra optional ??)
   
1. STM32CubeIDE
2. STM32CubeProgrammer / Flash Loader Demo appln

## Exploring STM32 Native bootloader
