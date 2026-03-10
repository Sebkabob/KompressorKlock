#ifndef APP_H
#define APP_H

#include "main.h"

/**
 * @brief  Initialize all application subsystems.
 *         Call once from main() after all MX_*_Init() peripherals are ready.
 * @param  hi2c  Pointer to the I2C handle used by sensors/battery/RTC.
 */
void App_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Run one iteration of the application.
 *         Call continuously from the main while(1) loop.
 */
void App_Update(void);

#endif // APP_H
