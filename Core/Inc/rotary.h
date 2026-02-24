#ifndef ROTARY_H
#define ROTARY_H

#include <stdint.h>
#include <stdbool.h>

/* ================= PUBLIC API ================= */

/**
 * @brief Initialize rotary encoder with EXTI interrupts. Call once before main loop.
 */
void Rotary_Init(void);

/**
 * @brief Poll accumulated encoder steps and switch state. Call every ~5ms from main loop.
 */
void Rotary_Update(void);

/**
 * @brief Returns true once if the switch was pressed (short press, debounced).
 *        Clears the flag on read.
 */
bool Rotary_SWPressed(void);

/**
 * @brief EXTI interrupt handler for encoder pins.
 *        Call this from EXTI0_1_IRQHandler and EXTI4_15_IRQHandler.
 */
void Rotary_EXTI_Handler(void);

/* ================= INTERACTIVE SCREEN REGISTRATION ================= */

/**
 * @brief Tell the rotary which screen index is the stopwatch.
 *        When this screen is active, press/scroll go to Stopwatch_OnPress/OnScroll.
 * @param idx Screen index returned by Screen_Register(), or -1 to disable.
 */
void Rotary_SetStopwatchScreenIndex(int idx);

/**
 * @brief Tell the rotary which screen index is the countdown timer.
 *        When this screen is active, press/scroll go to Countdown_OnPress/OnScroll.
 * @param idx Screen index returned by Screen_Register(), or -1 to disable.
 */
void Rotary_SetCountdownScreenIndex(int idx);

#endif // ROTARY_H
