/******************************************************************************
 * rtc_rv3032.h
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

#ifndef RTC_RV3032_H
#define RTC_RV3032_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== I2C ADDRESS ==================== */
#define RV3032_ADDR                     0x51
#define RV3032_ADDR_WRITE               (RV3032_ADDR << 1)
#define RV3032_ADDR_READ                ((RV3032_ADDR << 1) | 0x01)

/* ==================== WEEKDAY DEFINITIONS ==================== */
#define RV3032_SUNDAY                   0x01
#define RV3032_MONDAY                   0x02
#define RV3032_TUESDAY                  0x04
#define RV3032_WEDNESDAY                0x08
#define RV3032_THURSDAY                 0x10
#define RV3032_FRIDAY                   0x20
#define RV3032_SATURDAY                 0x40

/* ==================== REGISTER ADDRESSES ==================== */
#define RV3032_HUNDREDTHS               0x00
#define RV3032_SECONDS                  0x01
#define RV3032_MINUTES                  0x02
#define RV3032_HOURS                    0x03
#define RV3032_WEEKDAYS                 0x04
#define RV3032_DATE                     0x05
#define RV3032_MONTHS                   0x06
#define RV3032_YEARS                    0x07
#define RV3032_MINUTES_ALARM            0x08
#define RV3032_HOURS_ALARM              0x09
#define RV3032_DATE_ALARM               0x0A
#define RV3032_TIMER_0                  0x0B
#define RV3032_TIMER_1                  0x0C
#define RV3032_STATUS                   0x0D
#define RV3032_TEMP_LSB                 0x0E
#define RV3032_TEMP_MSB                 0x0F
#define RV3032_CONTROL1                 0x10
#define RV3032_CONTROL2                 0x11
#define RV3032_CONTROL3                 0x12
#define RV3032_TS_CONTROL               0x13
#define RV3032_CLOCK_INT_MASK           0x14
#define RV3032_EVI_CONTROL              0x15
#define RV3032_HUNDREDTHS_CAPTURE       0x27
#define RV3032_SECONDS_CAPTURE          0x28
#define RV3032_MINUTES_CAPTURE          0x29
#define RV3032_HOURS_CAPTURE            0x2A
#define RV3032_EEPROM_OFFSET            0xC1
#define RV3032_EEPROM_CLKOUT_2          0xC3

/* ==================== BIT DEFINITIONS ==================== */

/* Alarm Enable Bit */
#define ALARM_ENABLE                    7

/* Status Register Bits */
#define STATUS_THF                      7  /* Temp. High Flag */
#define STATUS_TLF                      6  /* Temp Low Flag */
#define STATUS_UF                       5  /* Periodic Time Update Flag */
#define STATUS_TF                       4  /* Periodic Countdown Update Flag */
#define STATUS_AF                       3  /* Alarm Flag */
#define STATUS_EVF                      2  /* External Event Flag */
#define STATUS_PORF                     1  /* Power On Reset Flag */
#define STATUS_VLF                      0  /* Voltage Low Flag */

/* Control 1 Register Bits */
#define CONTROL1_USEL                   4  /* Update Interrupt Select */
#define CONTROL1_TE                     3  /* Periodic Countdown Timer Enable */
#define CONTROL1_EERD                   2  /* EEPROM Memory Refresh Disable */
#define CONTROL1_TD                     0  /* Timer Clock Frequency selection */

/* Control 2 Register Bits */
#define CONTROL2_CLKIE                  6  /* Interrupt Controlled Clock Output Enable */
#define CONTROL2_UIE                    5  /* Periodic Time Update Interrupt Enable */
#define CONTROL2_TIE                    4  /* Periodic Countdown Timer Interrupt Enable */
#define CONTROL2_AIE                    3  /* Alarm Interrupt Enable */
#define CONTROL2_EIE                    2  /* External Event Interrupt Enable bit */
#define CONTROL2_STOP                   0  /* Stop, used for synchronization */

/* Control 3 Register Bits */
#define CONTROL3_BSIE                   4  /* Backup Switchover Interrupt Enable */
#define CONTROL3_THE                    3  /* Temperature High Enable */
#define CONTROL3_TLE                    2  /* Temperature Low Enable */
#define CONTROL3_THIE                   1  /* Temperature High Interrupt Enable */
#define CONTROL3_TLIE                   0  /* Temperature Low Interrupt Enable */

/* TS Control Register Bits */
#define TS_CONTROL_EVR                  7  /* Time Stamp EVI Reset */
#define TS_CONTROL_EVOW                 2  /* Time Stamp EVI Overwrite */

/* EVI Control Register Bits */
#define EVI_CONTROL_EHL                 6  /* Event High/Low Level selection */
#define EVI_CONTROL_ET                  4  /* Event Filtering Time set */
#define EVI_CONTROL_ESYN                0  /* Event Filtering Time set */

/* EEPROM CLKOUT Register Bits */
#define EEPROM_CLKOUT2_OS               7  /* Oscillator Selection */
#define EEPROM_CLKOUT2_FD               5  /* CLKOUT Frequency Selection */

/* ==================== CONFIGURATION VALUES ==================== */
#define TWELVE_HOUR_MODE                true
#define TWENTYFOUR_HOUR_MODE            false

/* Countdown Timer Frequencies */
#define COUNTDOWN_TIMER_FREQ_4096_HZ    0b00
#define COUNTDOWN_TIMER_FREQ_64_HZ      0b01
#define COUNTDOWN_TIMER_FREQ_1_HZ       0b10
#define COUNTDOWN_TIMER_FREQ_1_60_HZ    0b11

/* Clock Output Frequencies */
#define CLKOUT_FREQ_32768_HZ            0b00
#define CLKOUT_FREQ_1024_HZ             0b01
#define CLKOUT_FREQ_64_HZ               0b10
#define CLKOUT_FREQ_1_HZ                0b11

/* EVI Debounce Times */
#define EVI_DEBOUNCE_NONE               0b00  /* 0ms, default */
#define EVI_DEBOUNCE_256HZ              0b01  /* ~3.9ms */
#define EVI_DEBOUNCE_64HZ               0b10  /* ~15.6ms */
#define EVI_DEBOUNCE_8HZ                0b11  /* ~125ms */

/* Edge Detection */
#define RISING_EDGE                     true
#define FALLING_EDGE                    false

/* Time Update Frequency */
#define TIME_UPDATE_1_SECOND            false
#define TIME_UPDATE_1_MINUTE            true

/* Time Array */
#define TIME_ARRAY_LENGTH               8

/* ==================== TIME STRUCTURE ==================== */
typedef struct {
    uint8_t hundredths;
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t weekday;
    uint8_t date;
    uint8_t month;
    uint16_t year;
} RV3032_Time_t;

/* ==================== FUNCTION PROTOTYPES ==================== */

/* Initialization */
bool RV3032_Init(I2C_HandleTypeDef *hi2c);
bool RV3032_IsConnected(void);

/* 12/24 Hour Mode (software-side conversion) */
void RV3032_Set12Hour(void);
void RV3032_Set24Hour(void);
bool RV3032_Is12Hour(void);
bool RV3032_IsPM(void);

/* Time/Date String Functions */
char* RV3032_StringDateUSA(void);    /* MM/DD/YYYY */
char* RV3032_StringDate(void);       /* DD/MM/YYYY */
char* RV3032_StringTime(void);       /* HH:MM:SS (with AM/PM if 12hr) */
char* RV3032_StringTimestamp(void);  /* HH:MM:SS:HH */
char* RV3032_StringTime8601(void);   /* ISO 8601: YYYY-MM-DDTHH:MM:SS */

/* Set Time Functions */
bool RV3032_SetTime(uint8_t sec, uint8_t min, uint8_t hour,
                    uint8_t weekday, uint8_t date, uint8_t month, uint16_t year);
bool RV3032_SetTimeFromStruct(RV3032_Time_t *time);
bool RV3032_SetEpoch(uint32_t epoch);
bool RV3032_SetHundredthsToZero(void);
bool RV3032_SetSeconds(uint8_t value);
bool RV3032_SetMinutes(uint8_t value);
bool RV3032_SetHours(uint8_t value);
bool RV3032_SetDate(uint8_t value);
bool RV3032_SetWeekday(uint8_t value);
bool RV3032_SetMonth(uint8_t value);
bool RV3032_SetYear(uint16_t value);

/* Get Time Functions */
bool RV3032_UpdateTime(void);  /* Call this before reading time values */
uint8_t RV3032_GetHundredths(void);
uint8_t RV3032_GetSeconds(void);
uint8_t RV3032_GetMinutes(void);
uint8_t RV3032_GetHours(void);
uint8_t RV3032_GetDate(void);
uint8_t RV3032_GetWeekday(void);
uint8_t RV3032_GetMonth(void);
uint16_t RV3032_GetYear(void);
uint32_t RV3032_GetEpoch(void);
void RV3032_GetTimeStruct(RV3032_Time_t *time);

/* Timestamp Capture (EVI pin) */
uint8_t RV3032_GetHundredthsCapture(void);
uint8_t RV3032_GetSecondsCapture(void);
uint8_t RV3032_GetMinutesCapture(void);

/* Calibration */
bool RV3032_SetCalibrationOffset(float ppm);
float RV3032_GetCalibrationOffset(void);

/* External Event Input (EVI) Configuration */
bool RV3032_SetEVICalibration(bool enable);
bool RV3032_SetEVIDebounceTime(uint8_t debounceTime);
bool RV3032_SetEVIEdgeDetection(bool risingEdge);
bool RV3032_SetEVIEventCapture(bool enable);
bool RV3032_SetTSOverwrite(bool overwrite);
bool RV3032_GetEVICalibration(void);
uint8_t RV3032_GetEVIDebounceTime(void);
bool RV3032_GetEVIEdgeDetection(void);

/* Countdown Timer */
bool RV3032_SetCountdownTimerEnable(bool enable);
bool RV3032_SetCountdownTimerClockTicks(uint16_t clockTicks);
bool RV3032_SetCountdownTimerFrequency(uint8_t freq);
bool RV3032_GetCountdownTimerEnable(void);
uint16_t RV3032_GetCountdownTimerClockTicks(void);
uint8_t RV3032_GetCountdownTimerFrequency(void);

/* Clock Output */
bool RV3032_SetClockOutFrequency(uint8_t freq);
uint8_t RV3032_GetClockOutFrequency(void);

/* Periodic Time Update */
bool RV3032_SetPeriodicTimeUpdateFrequency(bool minuteUpdate);
bool RV3032_GetPeriodicTimeUpdateFrequency(void);

/* Alarm Functions */
void RV3032_SetAlarmItemsToMatch(bool minuteAlarm, bool hourAlarm, bool dateAlarm);
bool RV3032_SetAlarmMinutes(uint8_t minute);
bool RV3032_SetAlarmHours(uint8_t hour);
bool RV3032_SetAlarmDate(uint8_t date);
uint8_t RV3032_GetAlarmMinutes(void);
uint8_t RV3032_GetAlarmHours(void);
uint8_t RV3032_GetAlarmDate(void);

/* Interrupt Control */
bool RV3032_EnableInterrupt(uint8_t source);
bool RV3032_DisableInterrupt(uint8_t source);
bool RV3032_DisableAllInterrupts(void);

/* Interrupt Flags */
bool RV3032_GetInterruptFlag(uint8_t flag);
bool RV3032_ClearInterruptFlag(uint8_t flag);
bool RV3032_ClearAllInterruptFlags(void);

/* Temperature Reading */
float RV3032_GetTemperature(void);

/* Low-Level Register Access */
uint8_t RV3032_ReadRegister(uint8_t addr);
bool RV3032_WriteRegister(uint8_t addr, uint8_t val);
bool RV3032_ReadMultipleRegisters(uint8_t addr, uint8_t *dest, uint8_t len);
bool RV3032_WriteMultipleRegisters(uint8_t addr, uint8_t *values, uint8_t len);

/* Utility Functions */
uint8_t RV3032_BCDtoDEC(uint8_t val);
uint8_t RV3032_DECtoBCD(uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* RTC_RV3032_H */
