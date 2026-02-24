#ifndef ROTARY_H
#define ROTARY_H

#include <stdint.h>
#include <stdbool.h>

/* ================= PUBLIC API ================= */

/**
 * @brief Initialize rotary encoder with EXTI interrupts. Call once before main loop.
 *        Configures ROT_A and ROT_B as rising+falling EXTI inputs.
 */
void Rotary_Init(void);

/**
 * @brief Poll accumulated encoder steps and switch state. Call every ~5ms from main loop.
 *        Consumes steps accumulated by the EXTI ISR and drives screen transitions.
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

#endif // ROTARY_H
