#ifndef CALORIE_APP_H
#define CALORIE_APP_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CAL_STATE_DISPLAY,   /* Showing calories, scroll changes screens */
    CAL_STATE_EDITING    /* Scroll adjusts calories by +/-10 */
} CalorieState_t;

void Calorie_Init(void);
void Calorie_OnPress(void);
void Calorie_OnScroll(int direction);
void Calorie_OnLongPress(void);
CalorieState_t Calorie_GetState(void);
uint16_t Calorie_GetCount(void);
bool Calorie_NeedsRedraw(void);
bool Calorie_IsFlashing(void);

#endif // CALORIE_APP_H
