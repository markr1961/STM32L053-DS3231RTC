/*
 * stm32_ds3231.c
 *
 *  Created on: 2019. 3. 17.
 *      Author: kiki
 */
#include <stdio.h>
#include "stm32l0xx_hal.h"
#include "stm32_ds3231.h"

I2C_HandleTypeDef *myI2C;

static uint8_t Byte_Bcd2(uint8_t Value);
static uint8_t Bcd2ToByte(uint8_t Value);

//returns true if success
bool DS3231_Init(I2C_HandleTypeDef *handle)
{
  uint8_t status = 0;
  myI2C = handle;

  if(!ReadRegister(DS3231_REG_CONTROL, &status))
    return false;
  printf("control reg is 2%x",status);

  // enable osc (D7=0), turn on SQW output at 1Hz, (disables /IRQ)
  if(!WriteRegister(DS3231_REG_CONTROL, 0x40))
    return false;

  if(!ReadRegister(DS3231_REG_STATUS, &status))
    return false;
  printf("status reg is 2%x",status);

  // clear OSC stop flag and enable 32KHz output
  if(!WriteRegister(DS3231_REG_STATUS, 0x08))
    return false;

  return true;
}

//returns true if success
// Assumes 24-hour time
bool DS3231_GetTime(RTC_st *rtc)
{
  uint8_t startAddr = DS3231_REG_TIME;
  uint8_t buffer[7] = {0,};

  if(HAL_I2C_Master_Transmit(myI2C, DS3231_ADDR, &startAddr, 1, HAL_MAX_DELAY) != HAL_OK) return false;
  if(HAL_I2C_Master_Receive(myI2C, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return false;

  rtc->Sec = Bcd2ToByte(buffer[0] & 0x7F);
  rtc->Min = Bcd2ToByte(buffer[1] & 0x7F);
  rtc->Hour = Bcd2ToByte(buffer[2] & 0x3F);
  rtc->DaysOfWeek = (dow_en)(buffer[3] & 0x07);
  rtc->Date = Bcd2ToByte(buffer[4] & 0x3F);
  rtc->Month = Bcd2ToByte(buffer[5] & 0x1F);
  rtc->Year = Bcd2ToByte(buffer[6]);
  rtc->Year += (buffer[5] & 0x80)?1:0;

  return true;
}

//returns true if success
bool DS3231_SetTime(RTC_st *rtc)
{
  uint8_t startAddr = DS3231_REG_TIME;
  uint8_t buffer[8] = {startAddr, Byte_Bcd2(rtc->Sec), Byte_Bcd2(rtc->Min), Byte_Bcd2(rtc->Hour), rtc->DaysOfWeek, Byte_Bcd2(rtc->Date), Byte_Bcd2(rtc->Month), Byte_Bcd2(rtc->Year)};
  if(HAL_I2C_Master_Transmit(myI2C, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return false;

  return true;
}

bool DS3231_ReadTemperature(float *temp)
{
  uint8_t startAddr = DS3231_REG_TEMP;
  uint8_t buffer[2] = {0,};

  if(HAL_I2C_Master_Transmit(myI2C, DS3231_ADDR, &startAddr, 1, HAL_MAX_DELAY) != HAL_OK) return false;
  if(HAL_I2C_Master_Receive(myI2C, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return false;

  int16_t value = (buffer[0] << 8) | (buffer[1]);
  value = (value >> 6);

  *temp = value / 4.0f;
  return true;
}

bool DS3231_SetAlarm1(AlarmMode mode, uint8_t date, uint8_t hour, uint8_t min, uint8_t sec)
{
  uint8_t alarmSecond = Byte_Bcd2(sec);
  uint8_t alarmMinute = Byte_Bcd2(min);
  uint8_t alarmHour = Byte_Bcd2(hour);
  uint8_t alarmDate = Byte_Bcd2(date);

  switch(mode)
  {
  case ALARM_MODE_ALL_MATCHED:
    break;
  case ALARM_MODE_HOUR_MIN_SEC_MATCHED:
    alarmDate |= 0x80;
    break;
  case ALARM_MODE_MIN_SEC_MATCHED:
    alarmDate |= 0x80;
    alarmHour |= 0x80;
    break;
  case ALARM_MODE_SEC_MATCHED:
    alarmDate |= 0x80;
    alarmHour |= 0x80;
    alarmMinute |= 0x80;
    break;
  case ALARM_MODE_ONCE_PER_SECOND:
    alarmDate |= 0x80;
    alarmHour |= 0x80;
    alarmMinute |= 0x80;
    alarmSecond |= 0x80;
    break;
  default:
    break;
  }

  /* Write Alarm Registers */
  uint8_t startAddr = DS3231_REG_ALARM1;
  uint8_t buffer[5] = {startAddr, alarmSecond, alarmMinute, alarmHour, alarmDate};
  if(HAL_I2C_Master_Transmit(myI2C, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return false;

  /* Enable Alarm1 at Control Register */
  uint8_t ctrlReg = 0x00;
  ReadRegister(DS3231_REG_CONTROL, &ctrlReg);
  ctrlReg |= DS3231_CON_A1IE;
  ctrlReg |= DS3231_CON_INTCN;
  WriteRegister(DS3231_REG_CONTROL, ctrlReg);

  return true;
}

bool DS3231_ClearAlarm1()
{
  uint8_t ctrlReg;
  uint8_t statusReg;

  /* Clear Control Register */
  ReadRegister(DS3231_REG_CONTROL, &ctrlReg);
  ctrlReg &= ~DS3231_CON_A1IE;
  WriteRegister(DS3231_REG_CONTROL, ctrlReg);

  /* Clear Status Register */
  ReadRegister(DS3231_REG_STATUS, &statusReg);
  statusReg &= ~DS3231_STA_A1F;
  WriteRegister(DS3231_REG_STATUS, statusReg);

  return true;
}

//returns true if success
bool ReadRegister(uint8_t regAddr, uint8_t *value)
{
  if(HAL_I2C_Master_Transmit(myI2C, DS3231_ADDR, &regAddr, 1, HAL_MAX_DELAY) != HAL_OK) return false;
  if(HAL_I2C_Master_Receive(myI2C, DS3231_ADDR, value, 1, HAL_MAX_DELAY) != HAL_OK) return false;

  return true;
}

//returns true if success
bool WriteRegister(uint8_t regAddr, uint8_t value)
{
  uint8_t buffer[2] = {regAddr, value};
  if(HAL_I2C_Master_Transmit(myI2C, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return false;

  return true;
}

/**
  * @brief  Convert a 2 digit decimal to BCD format.
  * @param  Value Byte to be converted
  * @retval Converted byte
  */
static uint8_t Byte_Bcd2(uint8_t Value)
{
  uint32_t bcdhigh = 0U;

  while (Value >= 10U)
  {
    bcdhigh++;
    Value -= 10U;
  }

  return ((uint8_t)(bcdhigh << 4U) | Value);
}

/**
  * @brief  Convert from 2 digit BCD to Binary.
  * @param  Value BCD value to be converted
  * @retval Converted word
  */
static uint8_t Bcd2ToByte(uint8_t Value)
{
  uint32_t tmp;
  tmp = (((uint32_t)Value & 0xF0U) >> 4U) * 10U;
  return (uint8_t)(tmp + ((uint32_t)Value & 0x0FU));
}
