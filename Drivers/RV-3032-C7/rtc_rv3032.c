/******************************************************************************
 * rtc_rv3032.c
 * RV-3032-C7 RTC Driver for STM32
 *
 * Ported from SparkFun RV3032 Arduino Library
 * Original by Andy England @ SparkFun Electronics
 * Modified for RV-3032-C7 by Cole Kindall
 *
 * Ported to STM32 HAL by Claude
 *
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 ******************************************************************************/

#include "rtc_rv3032.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

/* ==================== PRIVATE VARIABLES ==================== */
static I2C_HandleTypeDef *_hi2c = NULL;
static uint8_t _time[TIME_ARRAY_LENGTH];
static bool _isTwelveHour = true;

/* I2C Timeout in milliseconds */
#define RV3032_I2C_TIMEOUT  100

/* ==================== TIME ARRAY INDICES ==================== */
#define TIME_HUNDREDTHS     0
#define TIME_SECONDS        1
#define TIME_MINUTES        2
#define TIME_HOURS          3
#define TIME_WEEKDAY        4
#define TIME_DATE           5
#define TIME_MONTH          6
#define TIME_YEAR           7

/* ==================== PRIVATE FUNCTION PROTOTYPES ==================== */
static bool RV3032_ReadBit(uint8_t regAddr, uint8_t bitAddr);
static uint8_t RV3032_ReadTwoBits(uint8_t regAddr, uint8_t bitAddr);
static bool RV3032_WriteBit(uint8_t regAddr, uint8_t bitAddr, bool bitToWrite);
static bool RV3032_WriteTwoBits(uint8_t regAddr, uint8_t bitAddr, uint8_t bitsToWrite);
static bool RV3032_SetTimeArray(uint8_t *timeArray, uint8_t len);

/* ==================== INITIALIZATION ==================== */

bool RV3032_Init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;

    /* Check if device is connected */
    if (HAL_I2C_IsDeviceReady(_hi2c, RV3032_ADDR_WRITE, 3, RV3032_I2C_TIMEOUT) != HAL_OK)
    {
        return false;
    }

    /* Clear time array */
    memset(_time, 0, sizeof(_time));

    return true;
}

bool RV3032_IsConnected(void)
{
    if (_hi2c == NULL) return false;
    return (HAL_I2C_IsDeviceReady(_hi2c, RV3032_ADDR_WRITE, 1, RV3032_I2C_TIMEOUT) == HAL_OK);
}

/* ==================== 12/24 HOUR MODE ==================== */

void RV3032_Set12Hour(void)
{
    _isTwelveHour = TWELVE_HOUR_MODE;
}

void RV3032_Set24Hour(void)
{
    _isTwelveHour = TWENTYFOUR_HOUR_MODE;
}

bool RV3032_Is12Hour(void)
{
    return _isTwelveHour;
}

bool RV3032_IsPM(void)
{
    if (RV3032_Is12Hour())
    {
        return RV3032_BCDtoDEC(_time[TIME_HOURS]) >= 12;
    }
    return false;
}

/* ==================== STRING FUNCTIONS ==================== */

char* RV3032_StringDateUSA(void)
{
    static char date[16];
    sprintf(date, "%02d/%02d/20%02d",
            RV3032_BCDtoDEC(_time[TIME_MONTH]),
            RV3032_BCDtoDEC(_time[TIME_DATE]),
            RV3032_BCDtoDEC(_time[TIME_YEAR]));
    return date;
}

char* RV3032_StringDate(void)
{
    static char date[16];
    sprintf(date, "%02d/%02d/20%02d",
            RV3032_BCDtoDEC(_time[TIME_DATE]),
            RV3032_BCDtoDEC(_time[TIME_MONTH]),
            RV3032_BCDtoDEC(_time[TIME_YEAR]));
    return date;
}

char* RV3032_StringTime(void)
{
    static char timeStr[20];

    if (RV3032_Is12Hour())
    {
        char half = 'A';
        uint8_t twelveHourCorrection = 0;
        uint8_t hours = RV3032_BCDtoDEC(_time[TIME_HOURS]);

        if (RV3032_IsPM())
        {
            half = 'P';
            if (hours > 12)
            {
                twelveHourCorrection = 12;
            }
        }
        if (hours == 0)
        {
            hours = 12;  /* Midnight is 12 AM */
        }

        sprintf(timeStr, "%02d:%02d:%02d%cM",
                hours - twelveHourCorrection,
                RV3032_BCDtoDEC(_time[TIME_MINUTES]),
                RV3032_BCDtoDEC(_time[TIME_SECONDS]),
                half);
    }
    else
    {
        sprintf(timeStr, "%02d:%02d:%02d",
                RV3032_BCDtoDEC(_time[TIME_HOURS]),
                RV3032_BCDtoDEC(_time[TIME_MINUTES]),
                RV3032_BCDtoDEC(_time[TIME_SECONDS]));
    }

    return timeStr;
}

char* RV3032_StringTimestamp(void)
{
    static char timeStr[20];

    uint8_t hoursCapture = RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_HOURS_CAPTURE));
    uint8_t minutesCapture = RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_MINUTES_CAPTURE));
    uint8_t secondsCapture = RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_SECONDS_CAPTURE));
    uint8_t hundredthsCapture = RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_HUNDREDTHS_CAPTURE));

    if (RV3032_Is12Hour())
    {
        char half = 'A';
        uint8_t twelveHourCorrection = 0;

        if (hoursCapture >= 12)
        {
            half = 'P';
            if (hoursCapture > 12)
            {
                twelveHourCorrection = 12;
            }
        }
        if (hoursCapture == 0)
        {
            hoursCapture = 12;
        }

        sprintf(timeStr, "%02d:%02d:%02d:%02d%cM",
                hoursCapture - twelveHourCorrection,
                minutesCapture,
                secondsCapture,
                hundredthsCapture,
                half);
    }
    else
    {
        sprintf(timeStr, "%02d:%02d:%02d:%02d",
                hoursCapture,
                minutesCapture,
                secondsCapture,
                hundredthsCapture);
    }

    return timeStr;
}

char* RV3032_StringTime8601(void)
{
    static char timeStamp[30];

    sprintf(timeStamp, "20%02d-%02d-%02dT%02d:%02d:%02d",
            RV3032_BCDtoDEC(_time[TIME_YEAR]),
            RV3032_BCDtoDEC(_time[TIME_MONTH]),
            RV3032_BCDtoDEC(_time[TIME_DATE]),
            RV3032_BCDtoDEC(_time[TIME_HOURS]),
            RV3032_BCDtoDEC(_time[TIME_MINUTES]),
            RV3032_BCDtoDEC(_time[TIME_SECONDS]));

    return timeStamp;
}

/* ==================== SET TIME FUNCTIONS ==================== */

bool RV3032_SetTime(uint8_t sec, uint8_t min, uint8_t hour,
                    uint8_t weekday, uint8_t date, uint8_t month, uint16_t year)
{
    _time[TIME_SECONDS] = RV3032_DECtoBCD(sec);
    _time[TIME_MINUTES] = RV3032_DECtoBCD(min);
    _time[TIME_HOURS] = RV3032_DECtoBCD(hour);
    _time[TIME_DATE] = RV3032_DECtoBCD(date);
    _time[TIME_WEEKDAY] = 1 << weekday;  /* Weekday is bit-encoded */
    _time[TIME_MONTH] = RV3032_DECtoBCD(month);
    _time[TIME_YEAR] = RV3032_DECtoBCD(year - 2000);

    return RV3032_SetTimeArray(_time, TIME_ARRAY_LENGTH);
}

bool RV3032_SetTimeFromStruct(RV3032_Time_t *time)
{
    return RV3032_SetTime(time->seconds, time->minutes, time->hours,
                          time->weekday, time->date, time->month, time->year);
}

static bool RV3032_SetTimeArray(uint8_t *timeArray, uint8_t len)
{
    if (len != TIME_ARRAY_LENGTH)
        return false;

    /* Write all registers except hundredths (read-only) */
    /* Start from SECONDS register (0x01) and write 7 bytes */
    return RV3032_WriteMultipleRegisters(RV3032_SECONDS, timeArray + 1, len - 1);
}

bool RV3032_SetEpoch(uint32_t epoch)
{
    /* Minimum epoch is 2000-01-01 00:00:00 */
    if (epoch < 946684800)
    {
        epoch = 946684800;
    }

    time_t t = epoch;
    struct tm *tmp = gmtime(&t);

    _time[TIME_SECONDS] = RV3032_DECtoBCD(tmp->tm_sec);
    _time[TIME_MINUTES] = RV3032_DECtoBCD(tmp->tm_min);
    _time[TIME_HOURS] = RV3032_DECtoBCD(tmp->tm_hour);
    _time[TIME_DATE] = RV3032_DECtoBCD(tmp->tm_mday);
    _time[TIME_WEEKDAY] = 1 << tmp->tm_wday;
    _time[TIME_MONTH] = RV3032_DECtoBCD(tmp->tm_mon + 1);
    _time[TIME_YEAR] = RV3032_DECtoBCD(tmp->tm_year - 100);

    return RV3032_SetTimeArray(_time, TIME_ARRAY_LENGTH);
}

bool RV3032_SetHundredthsToZero(void)
{
    /* Toggle STOP bit to reset hundredths */
    bool temp = RV3032_WriteBit(RV3032_CONTROL2, CONTROL2_STOP, true);
    temp &= RV3032_WriteBit(RV3032_CONTROL2, CONTROL2_STOP, false);
    return temp;
}

bool RV3032_SetSeconds(uint8_t value)
{
    _time[TIME_SECONDS] = RV3032_DECtoBCD(value);
    return RV3032_SetTimeArray(_time, TIME_ARRAY_LENGTH);
}

bool RV3032_SetMinutes(uint8_t value)
{
    _time[TIME_MINUTES] = RV3032_DECtoBCD(value);
    return RV3032_SetTimeArray(_time, TIME_ARRAY_LENGTH);
}

bool RV3032_SetHours(uint8_t value)
{
    _time[TIME_HOURS] = RV3032_DECtoBCD(value);
    return RV3032_SetTimeArray(_time, TIME_ARRAY_LENGTH);
}

bool RV3032_SetDate(uint8_t value)
{
    _time[TIME_DATE] = RV3032_DECtoBCD(value);
    return RV3032_SetTimeArray(_time, TIME_ARRAY_LENGTH);
}

bool RV3032_SetWeekday(uint8_t value)
{
    if (value > 6)
    {
        value = 6;
    }
    _time[TIME_WEEKDAY] = 1 << value;
    return RV3032_SetTimeArray(_time, TIME_ARRAY_LENGTH);
}

bool RV3032_SetMonth(uint8_t value)
{
    _time[TIME_MONTH] = RV3032_DECtoBCD(value);
    return RV3032_SetTimeArray(_time, TIME_ARRAY_LENGTH);
}

bool RV3032_SetYear(uint16_t value)
{
    _time[TIME_YEAR] = RV3032_DECtoBCD(value - 2000);
    return RV3032_SetTimeArray(_time, TIME_ARRAY_LENGTH);
}

/* ==================== GET TIME FUNCTIONS ==================== */

bool RV3032_UpdateTime(void)
{
    if (!RV3032_ReadMultipleRegisters(RV3032_HUNDREDTHS, _time, TIME_ARRAY_LENGTH))
    {
        return false;
    }

    /* If seconds are at 59, read again to avoid missing a minute rollover */
    if (RV3032_BCDtoDEC(_time[TIME_SECONDS]) == 59)
    {
        uint8_t tempTime[TIME_ARRAY_LENGTH];
        if (!RV3032_ReadMultipleRegisters(RV3032_HUNDREDTHS, tempTime, TIME_ARRAY_LENGTH))
        {
            return false;
        }

        /* If seconds changed to 0, use the new reading */
        if (RV3032_BCDtoDEC(tempTime[TIME_SECONDS]) == 0)
        {
            memcpy(_time, tempTime, TIME_ARRAY_LENGTH);
        }
    }

    return true;
}

uint8_t RV3032_GetHundredths(void)
{
    return RV3032_BCDtoDEC(_time[TIME_HUNDREDTHS]);
}

uint8_t RV3032_GetSeconds(void)
{
    return RV3032_BCDtoDEC(_time[TIME_SECONDS]);
}

uint8_t RV3032_GetMinutes(void)
{
    return RV3032_BCDtoDEC(_time[TIME_MINUTES]);
}

uint8_t RV3032_GetHours(void)
{
    uint8_t tempHours = RV3032_BCDtoDEC(_time[TIME_HOURS]);

    if (RV3032_Is12Hour())
    {
        if (tempHours > 12)
        {
            tempHours -= 12;
        }
        else if (tempHours == 0)
        {
            tempHours = 12;
        }
    }

    return tempHours;
}

uint8_t RV3032_GetDate(void)
{
    return RV3032_BCDtoDEC(_time[TIME_DATE]);
}

uint8_t RV3032_GetWeekday(void)
{
    uint8_t tempWeekday = _time[TIME_WEEKDAY];

    /* Convert from bit-encoded to 0-6 value */
    uint8_t day = 0;
    while (tempWeekday > 1)
    {
        tempWeekday >>= 1;
        day++;
    }

    return day;
}

uint8_t RV3032_GetMonth(void)
{
    return RV3032_BCDtoDEC(_time[TIME_MONTH]);
}

uint16_t RV3032_GetYear(void)
{
    return RV3032_BCDtoDEC(_time[TIME_YEAR]) + 2000;
}

uint32_t RV3032_GetEpoch(void)
{
    struct tm tm_time;

    tm_time.tm_isdst = -1;
    tm_time.tm_yday = 0;
    tm_time.tm_wday = 0;
    tm_time.tm_year = RV3032_BCDtoDEC(_time[TIME_YEAR]) + 100;  /* Years since 1900 */
    tm_time.tm_mon = RV3032_BCDtoDEC(_time[TIME_MONTH]) - 1;    /* 0-11 */
    tm_time.tm_mday = RV3032_BCDtoDEC(_time[TIME_DATE]);
    tm_time.tm_hour = RV3032_BCDtoDEC(_time[TIME_HOURS]);
    tm_time.tm_min = RV3032_BCDtoDEC(_time[TIME_MINUTES]);
    tm_time.tm_sec = RV3032_BCDtoDEC(_time[TIME_SECONDS]);

    return (uint32_t)mktime(&tm_time);
}

void RV3032_GetTimeStruct(RV3032_Time_t *time)
{
    time->hundredths = RV3032_GetHundredths();
    time->seconds = RV3032_GetSeconds();
    time->minutes = RV3032_GetMinutes();
    time->hours = RV3032_BCDtoDEC(_time[TIME_HOURS]);  /* Raw 24h value */
    time->weekday = RV3032_GetWeekday();
    time->date = RV3032_GetDate();
    time->month = RV3032_GetMonth();
    time->year = RV3032_GetYear();
}

/* ==================== TIMESTAMP CAPTURE ==================== */

uint8_t RV3032_GetHundredthsCapture(void)
{
    return RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_HUNDREDTHS_CAPTURE));
}

uint8_t RV3032_GetSecondsCapture(void)
{
    return RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_SECONDS_CAPTURE));
}

uint8_t RV3032_GetMinutesCapture(void)
{
    return RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_MINUTES_CAPTURE));
}

/* ==================== CALIBRATION ==================== */

bool RV3032_SetCalibrationOffset(float ppm)
{
    int8_t integerOffset = (int8_t)(ppm / 0.2384f);  /* 0.2384 ppm/LSB */

    if (integerOffset < 0)
    {
        integerOffset += 64;
    }

    return RV3032_WriteRegister(RV3032_EEPROM_OFFSET, (uint8_t)integerOffset);
}

float RV3032_GetCalibrationOffset(void)
{
    int8_t value = RV3032_ReadRegister(RV3032_EEPROM_OFFSET);
    value &= 0b00111111;

    if (value > 32)
    {
        value -= 64;
    }

    return value * 0.2384f;
}

/* ==================== EVI CONFIGURATION ==================== */

bool RV3032_SetEVICalibration(bool enable)
{
    return RV3032_WriteBit(RV3032_CONTROL2, CONTROL2_STOP, enable);
}

bool RV3032_SetEVIDebounceTime(uint8_t debounceTime)
{
    return RV3032_WriteTwoBits(RV3032_EVI_CONTROL, EVI_CONTROL_ET, debounceTime);
}

bool RV3032_SetEVIEdgeDetection(bool risingEdge)
{
    return RV3032_WriteBit(RV3032_EVI_CONTROL, EVI_CONTROL_EHL, risingEdge);
}

bool RV3032_SetEVIEventCapture(bool enable)
{
    /* This enables/disables the timestamp feature on EVI pin events */
    return RV3032_WriteBit(RV3032_EVI_CONTROL, EVI_CONTROL_ESYN, enable);
}

bool RV3032_SetTSOverwrite(bool overwrite)
{
    return RV3032_WriteBit(RV3032_TS_CONTROL, TS_CONTROL_EVOW, overwrite);
}

bool RV3032_GetEVICalibration(void)
{
    return RV3032_ReadBit(RV3032_EVI_CONTROL, EVI_CONTROL_ESYN);
}

uint8_t RV3032_GetEVIDebounceTime(void)
{
    return RV3032_ReadTwoBits(RV3032_EVI_CONTROL, EVI_CONTROL_ET);
}

bool RV3032_GetEVIEdgeDetection(void)
{
    return RV3032_ReadBit(RV3032_EVI_CONTROL, EVI_CONTROL_EHL);
}

/* ==================== COUNTDOWN TIMER ==================== */

bool RV3032_SetCountdownTimerEnable(bool enable)
{
    return RV3032_WriteBit(RV3032_CONTROL1, CONTROL1_TE, enable);
}

bool RV3032_SetCountdownTimerFrequency(uint8_t freq)
{
    return RV3032_WriteTwoBits(RV3032_CONTROL1, CONTROL1_TD, freq);
}

bool RV3032_SetCountdownTimerClockTicks(uint16_t clockTicks)
{
    /* Write high byte first (preserving other bits) */
    uint8_t value = RV3032_ReadRegister(RV3032_TIMER_1);
    value &= 0xF0;  /* Clear lower nibble */
    value |= (clockTicks >> 8) & 0x0F;
    bool returnValue = RV3032_WriteRegister(RV3032_TIMER_1, value);

    /* Write low byte */
    returnValue &= RV3032_WriteRegister(RV3032_TIMER_0, clockTicks & 0xFF);

    return returnValue;
}

bool RV3032_GetCountdownTimerEnable(void)
{
    return RV3032_ReadBit(RV3032_CONTROL1, CONTROL1_TE);
}

uint8_t RV3032_GetCountdownTimerFrequency(void)
{
    return RV3032_ReadTwoBits(RV3032_CONTROL1, CONTROL1_TD);
}

uint16_t RV3032_GetCountdownTimerClockTicks(void)
{
    uint16_t value = (RV3032_ReadRegister(RV3032_TIMER_1) & 0x0F) << 8;
    value |= RV3032_ReadRegister(RV3032_TIMER_0);
    return value;
}

/* ==================== CLOCK OUTPUT ==================== */

bool RV3032_SetClockOutFrequency(uint8_t freq)
{
    /* First disable, then set new frequency */
    RV3032_WriteTwoBits(RV3032_EEPROM_CLKOUT_2, EEPROM_CLKOUT2_FD, 0);
    return RV3032_WriteTwoBits(RV3032_EEPROM_CLKOUT_2, EEPROM_CLKOUT2_FD, freq);
}

uint8_t RV3032_GetClockOutFrequency(void)
{
    return RV3032_ReadTwoBits(RV3032_EEPROM_CLKOUT_2, EEPROM_CLKOUT2_FD);
}

/* ==================== PERIODIC TIME UPDATE ==================== */

bool RV3032_SetPeriodicTimeUpdateFrequency(bool minuteUpdate)
{
    return RV3032_WriteBit(RV3032_CONTROL1, CONTROL1_USEL, minuteUpdate);
}

bool RV3032_GetPeriodicTimeUpdateFrequency(void)
{
    return RV3032_ReadBit(RV3032_CONTROL1, CONTROL1_USEL);
}

/* ==================== ALARM FUNCTIONS ==================== */

void RV3032_SetAlarmItemsToMatch(bool minuteAlarm, bool hourAlarm, bool dateAlarm)
{
    /* These bits are active-low (0 = match required, 1 = don't care) */
    RV3032_WriteBit(RV3032_MINUTES_ALARM, ALARM_ENABLE, !minuteAlarm);
    RV3032_WriteBit(RV3032_HOURS_ALARM, ALARM_ENABLE, !hourAlarm);
    RV3032_WriteBit(RV3032_DATE_ALARM, ALARM_ENABLE, !dateAlarm);
}

bool RV3032_SetAlarmMinutes(uint8_t minute)
{
    uint8_t value = RV3032_ReadRegister(RV3032_MINUTES_ALARM);
    value &= (1 << ALARM_ENABLE);  /* Keep only enable bit */
    value |= RV3032_DECtoBCD(minute);
    return RV3032_WriteRegister(RV3032_MINUTES_ALARM, value);
}

bool RV3032_SetAlarmHours(uint8_t hour)
{
    uint8_t value = RV3032_ReadRegister(RV3032_HOURS_ALARM);
    value &= (1 << ALARM_ENABLE);  /* Keep only enable bit */
    value |= RV3032_DECtoBCD(hour);
    return RV3032_WriteRegister(RV3032_HOURS_ALARM, value);
}

bool RV3032_SetAlarmDate(uint8_t date)
{
    uint8_t value = RV3032_ReadRegister(RV3032_DATE_ALARM);
    value &= (1 << ALARM_ENABLE);  /* Keep only enable bit */
    value |= RV3032_DECtoBCD(date);
    return RV3032_WriteRegister(RV3032_DATE_ALARM, value);
}

uint8_t RV3032_GetAlarmMinutes(void)
{
    return RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_MINUTES_ALARM) & 0x7F);
}

uint8_t RV3032_GetAlarmHours(void)
{
    return RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_HOURS_ALARM) & 0x3F);
}

uint8_t RV3032_GetAlarmDate(void)
{
    return RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_DATE_ALARM) & 0x3F);
}

/* ==================== INTERRUPT CONTROL ==================== */

bool RV3032_EnableInterrupt(uint8_t source)
{
    uint8_t value = RV3032_ReadRegister(RV3032_CONTROL2);
    value |= (1 << source);
    return RV3032_WriteRegister(RV3032_CONTROL2, value);
}

bool RV3032_DisableInterrupt(uint8_t source)
{
    uint8_t value = RV3032_ReadRegister(RV3032_CONTROL2);
    value &= ~(1 << source);
    return RV3032_WriteRegister(RV3032_CONTROL2, value);
}

bool RV3032_DisableAllInterrupts(void)
{
    uint8_t value = RV3032_ReadRegister(RV3032_CONTROL2);
    value &= 0x01;  /* Keep only STOP bit */
    return RV3032_WriteRegister(RV3032_CONTROL2, value);
}

/* ==================== INTERRUPT FLAGS ==================== */

bool RV3032_GetInterruptFlag(uint8_t flag)
{
    uint8_t status = RV3032_ReadRegister(RV3032_STATUS);
    return (status >> flag) & 0x01;
}

bool RV3032_ClearInterruptFlag(uint8_t flag)
{
    uint8_t value = RV3032_ReadRegister(RV3032_STATUS);
    value &= ~(1 << flag);
    return RV3032_WriteRegister(RV3032_STATUS, value);
}

bool RV3032_ClearAllInterruptFlags(void)
{
    return RV3032_WriteRegister(RV3032_STATUS, 0x00);
}

/* ==================== TEMPERATURE ==================== */

float RV3032_GetTemperature(void)
{
    uint8_t tempLSB = RV3032_ReadRegister(RV3032_TEMP_LSB);
    uint8_t tempMSB = RV3032_ReadRegister(RV3032_TEMP_MSB);

    /* Temperature is stored as 12-bit signed value */
    /* MSB contains integer part (signed), LSB[7:6] contains fractional part */
    int16_t tempRaw = ((int8_t)tempMSB << 4) | ((tempLSB >> 4) & 0x0F);

    /* Resolution is 0.0625°C per LSB */
    return tempRaw * 0.0625f;
}

/* ==================== LOW-LEVEL REGISTER ACCESS ==================== */

uint8_t RV3032_ReadRegister(uint8_t addr)
{
    uint8_t value = 0;

    if (_hi2c == NULL) return 0;

    /* Write register address, then read data */
    if (HAL_I2C_Mem_Read(_hi2c, RV3032_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT,
                         &value, 1, RV3032_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    return value;
}

bool RV3032_WriteRegister(uint8_t addr, uint8_t val)
{
    if (_hi2c == NULL) return false;

    return (HAL_I2C_Mem_Write(_hi2c, RV3032_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT,
                              &val, 1, RV3032_I2C_TIMEOUT) == HAL_OK);
}

bool RV3032_ReadMultipleRegisters(uint8_t addr, uint8_t *dest, uint8_t len)
{
    if (_hi2c == NULL || dest == NULL) return false;

    return (HAL_I2C_Mem_Read(_hi2c, RV3032_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT,
                             dest, len, RV3032_I2C_TIMEOUT) == HAL_OK);
}

bool RV3032_WriteMultipleRegisters(uint8_t addr, uint8_t *values, uint8_t len)
{
    if (_hi2c == NULL || values == NULL) return false;

    return (HAL_I2C_Mem_Write(_hi2c, RV3032_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT,
                              values, len, RV3032_I2C_TIMEOUT) == HAL_OK);
}

/* ==================== UTILITY FUNCTIONS ==================== */

uint8_t RV3032_BCDtoDEC(uint8_t val)
{
    return ((val / 0x10) * 10) + (val % 0x10);
}

uint8_t RV3032_DECtoBCD(uint8_t val)
{
    return ((val / 10) * 0x10) + (val % 10);
}

/* ==================== PRIVATE BIT MANIPULATION FUNCTIONS ==================== */

static bool RV3032_ReadBit(uint8_t regAddr, uint8_t bitAddr)
{
    return (RV3032_ReadRegister(regAddr) >> bitAddr) & 0x01;
}

static uint8_t RV3032_ReadTwoBits(uint8_t regAddr, uint8_t bitAddr)
{
    return (RV3032_ReadRegister(regAddr) >> bitAddr) & 0x03;
}

static bool RV3032_WriteBit(uint8_t regAddr, uint8_t bitAddr, bool bitToWrite)
{
    uint8_t value = RV3032_ReadRegister(regAddr);
    value &= ~(1 << bitAddr);
    value |= (bitToWrite ? 1 : 0) << bitAddr;
    return RV3032_WriteRegister(regAddr, value);
}

static bool RV3032_WriteTwoBits(uint8_t regAddr, uint8_t bitAddr, uint8_t bitsToWrite)
{
    uint8_t value = RV3032_ReadRegister(regAddr);
    value &= ~(0x03 << bitAddr);
    value |= (bitsToWrite & 0x03) << bitAddr;
    return RV3032_WriteRegister(regAddr, value);
}
