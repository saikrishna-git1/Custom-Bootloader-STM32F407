# Custom-Bootloader-STM32F407
Custom bootloader project for ARM Cortex-M4 based STM32F407VG microcontroller on STM32F407G-DISC1 board.

## Docs to refer
1. RM0090 - Reference manual STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced Arm®-based 32-bit MCUs  
   section 2.4 - Boot configuration  
   section 3.3 - Embedded Flash memory in STM32F405xx/07xx and STM32F415xx/17xx
3. AN2606 - STM32 microcontroller system memory boot mode  
   section 28.1.1 - STM32F40xxx/41xxx devices -> Bootloader configuration
4. UM1472 - Discovery kit with STM32F407VG MCU  
   section 6.1.3 - ST-LINK/V2-A VCP configuration
5. DS8626 - Datasheet STM32F405xx STM32F407xx  
   section 3 - Pinouts
6. Schematic - en.MB997-F407VGT6-E01_Schematic  
   BOOT0, BOOT1/PB2 pins
   


## Hardware and Software
1. STM32F407G-DISC1 board
2. FTDI serial (USB2UART) converter - 2 nos (extra optional ??)
 
1. STM32CubeIDE
2. Flash Loader Demo application / STM32CubeProgrammer



## Exploring STM32 Native bootloader present in the system memory
### To boot from system memory and communicate to PC via USART3 peripheral
1. Short BOOT1/PB2 and GND
2. Short BOOT0 and VDD
3. Set serial converter voltage to 3.3V using jumper (??)
4. Connect PC10 (USART3_TX) to RX on serial converter
5. Connect PC11 (USART3_RX) to TX on serial converter
6. Connect GND to GND on serial converter

### Flash Loader Demo application
1. Select the COM port for serial converter (not STM board) and click Next - Reset the board and try again if it doesn't work
2. Click Next if target is readable and see the target info
3. Click next if you want the bootloader to flash any .hex file into Flash memory and execute it.



## Custom bootloader communication with Host
2 USART peripherals are used to communicate with the Host:
1. Virtual COM port - to connect to Host to receive commands and send ACK/NACK.
   The ST-LINK/V2-A supports a Virtual COM port (VCP) on U2 pin 12 (ST-LINK_TX) and U2 pin 13 (ST-LINK_RX) but these pins are not connected to the USART of the STM32F407 microcontroller. To connect an STM32F407 USART to the VCP on the PC, use USART to USB dongle from the market connected for instance to STM32F407 USART2 available on connector P1 pin 14 (PA2: USART2_TX) and P1 pin 13 (PA3: USART2_RX).
   USART2 is not used to communicate with the system bootloader anyways.  
2. Debug port - to print debug messages from bootloader code.

