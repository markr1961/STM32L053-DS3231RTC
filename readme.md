## STM32L053 Nucleo-64 with I2C RTC

This project appears to have been an early POC with RTC for L083 development. 
It uses a NUCLEO-L053R8 demo board with a DS3231 RTC connected over I2C. 

### Build
Project was created using CubeMX 6.3.0 with STM32Cube FW_L0 V1.12.1  
The IDE is IAR v8.40.2  

### CPU
STM32L53R8  
No external clocks are used.  
CPU is running at 24MHz.  

### Connections
Supports a DS3231 RTC over I2C:  
PB9 I2C SDA  
PB8 I2C SCL  
USART2 is used for communication through the debugger.  
PA5 is  LED "LD2".  
PC13 is button "B1" (blue). IRQ enabled on falling edge.  
