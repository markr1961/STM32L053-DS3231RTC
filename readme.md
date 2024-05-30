## STM32L053 Nucleo-64 with I2C RTC

This project appears to have been and early L083 development POC.

### Build
IAR 8.40.2  

### Brief
Supports a DS3231 RTC over I2C:  
PB9 I2C SDA  
PB8 I2C SCL  

No external clocks are used.  
CPU is running at 24MHz.  
USART2 is used for communication.  

LED is "LD2" on PA5
Button is "B1" on PC13. IRQ on falling.
