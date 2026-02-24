#ifndef ROTARY_H
#define ROTARY_H

#include <stdint.h>
#include <stdbool.h>

/* ================= PUBLIC API ================= */

/**
 * @brief Initialize rotary encoder state. Call once before the main loop.
 */
void Rotary_Init(void);

/**
 * @brief Poll encoder and switch state. Call every ~5ms from main loop.
 *        Internally drives Screen_Next / Screen_Prev on rotation,
 *        and can be extended for SW press handling.
 */
void Rotary_Update(void);

/**
 * @brief Returns true once if the switch was pressed (short press, debounced).
 *        Clears the flag on read.
 */
bool Rotary_SWPressed(void);

#endif // ROTARY_H
