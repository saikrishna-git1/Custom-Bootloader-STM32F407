# Custom-Bootloader-STM32F407
Custom bootloader project for ARM Cortex-M4 based STM32F407VG microcontroller on STM32F407G-DISC1 board. 2 separate projects - custom_bootloader and user_application are compiled separately. 

## Custom bootloader features
### Features present
1. If user button is pressed at the time of reset, bootloader will wait for commands to be received via USART2 (PA2: USART2_TX, PA3: USART2_RX) peripheral. Else, it directly jumps to user application.
2. Does a CRC check of the commands and proceeds to execute them only if CRC is a pass. (see bootloader commands)
3. Updates the user application (firmware) using the custom bootloader after CRC.

### To be added :) 
1. Implement a mechanism to preserve variables of bootloader so that they can be used in user application and vice-versa.
2. Instead of having 2 separate projects for bootloader and application, the more convenient thing to do is to build a single binary which contains both programs and pad the bootloader executable to the max size so that it is always 32KB (maybe using a python script).

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
   
## Useful links
1. https://community.renesas.com/mcu-mpu/rx/f/rx-forum/6853/memory-management-for-bootloader-and-application
2. https://medium.com/@muzi.xu88/story-about-bootloader-and-iap-458b1f8dcebd
3. https://interrupt.memfault.com/blog/how-to-write-a-bootloader-from-scratch#deciding-on-a-memory-map
4. Worlds Simplest Bootloader - https://www.youtube.com/watch?v=cfNJ85cX-ms&list=PLP29wDx6QmW7HaCrRydOnxcy8QmW0SNdQ&index=5
5. Udemy - Fastbit course - https://www.udemy.com/course/stm32f4-arm-cortex-mx-custom-bootloader-development/

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
   The ST-LINK/V2-A supports a Virtual COM port (VCP) on U2 pin 12 (ST-LINK_TX) and U2 pin 13 (ST-LINK_RX) but these pins are not connected to the USART of the STM32F407 microcontroller on STM32F407G-DISC1 board (but on other boards, it is connected to USART peripheral of microcontroller).  
   To connect an STM32F407 USART to the VCP on the PC, use USART to USB dongle from the market connected for instance to STM32F407 USART2 available on connector P1 pin 14 (PA2: USART2_TX) and P1 pin 13 (PA3: USART2_RX).    
   USART2 is not used to communicate with the system bootloader anyways.  
3. Debug port - to print debug messages from bootloader code. This is optional.  
   Alternatively, you can blink LED from code to indicate success/failure of operations. USART3 - PC10 (USART3_TX pin), PC11 (USART3_RX pin).



## Bootloader code placement in Flash memory
System memory (ROM) - 30KB - ST Native bootloader is already present. Can't erase/modify it.  
Custom bootloader will be placed in main memory of the Flash memory in Sectors-0,1 (16KB each sector). Hence, base address of custom bootloader = 0x0800 0000.   
User application(s) will be placed in Sectors-2 to 11. Hence, base address of user application = 0x0800 8000.



## Bootloader custom commands
![image](https://github.com/saikrishna-git1/Custom-Bootloader-STM32F407/assets/29352891/a847e722-4f6b-4735-b459-a30257f2b68e)
![image](https://github.com/saikrishna-git1/Custom-Bootloader-STM32F407/assets/29352891/276b28b3-a2d4-4768-b091-2c1bd9431e41)
![image](https://github.com/saikrishna-git1/Custom-Bootloader-STM32F407/assets/29352891/3b249c60-534c-4f2a-9e8e-7a8a0c6c2807)
![image](https://github.com/saikrishna-git1/Custom-Bootloader-STM32F407/assets/29352891/42c149ec-98b3-48d6-800e-05f446e97ba6)
![image](https://github.com/saikrishna-git1/Custom-Bootloader-STM32F407/assets/29352891/11d556b0-a7f1-46d7-8958-ebee1d135ff2)



## Host - Bootloader communication
![image](https://github.com/saikrishna-git1/Custom-Bootloader-STM32F407/assets/29352891/52f74b02-c801-443d-9886-791adf7c4722)  
Once bootloader recieves the command packet, it first does CRC32 to ensure data integrity. It then decodes the command. If CRC is correct it sends an ACK + how many bytes bootloader is going to send in its next reply, to the command else it sends a NACK.  
It then excutes th command and sends its reply back to the Host.



## Bootloader project creation
Used CubeMX software to add all the peripheral drivers needed for this task:  
USART2 - PA2, PA3  
USART3 - PC10, PC11  
CRC  
This code has to be placed at address 0x0800 0000 (default address).  
This project should not compile if the size of the executable exceeds 32KB (2 sectors of Flash memory). Hence, put this value in the linker script of this project. (You don't actually see any errors like:  
`custom_bootloader_001.elf section .rodata will not fit in region FLASH`  
`region FLASH overflowed by 12268 bytes`
during compile if you are not using the variables created in the flash anywhere in your code which are occupying more than allocated space in the linker script, check the .map file -> 'Discarded input sections' to see if 'rodata.var' is present.  
If you don't want a section to be discarded, use the keyword KEEP in the linker script for that particular section).


## User_application project creation
This user application just toggles LED - LD3 (PD13). This code has to be placed at address 0x0800 8000.  
Change this value in linker script. This project should not compile if the size of the executable exceeds (1024-32) = 992KB (remaining sectors of Flash memory). Hence, put this value in the linker script of this project.  
Can verify it by erasing the entire Flash and then programming it and using 'Memory browser' window to view the contents of Flash.  




## To explore
1. bewitched - commandline hex editor to view .bin files in hex format
2. 







