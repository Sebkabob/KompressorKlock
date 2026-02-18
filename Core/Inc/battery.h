#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

// Initialization
bool BATTERY_Init(void);

// Call once per update cycle to refresh cached values
bool BATTERY_UpdateState(void);

// Cached battery state accessors
uint16_t BATTERY_GetVoltage(void);   // Voltage in mV
int16_t BATTERY_GetCurrent(void);    // Current in mA
uint16_t BATTERY_GetSOC(void);       // State of charge %
bool BATTERY_IsCharging(void);       // Charging status
bool BATTERY_IsFull(void);           // Full battery status
bool BATTERY_IsLow(void);           // Low battery status
bool BATTERY_IsCritical(void);       // Critical battery status

#endif // BATTERY_H
