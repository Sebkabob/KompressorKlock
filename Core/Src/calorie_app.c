#include "calorie_app.h"
#include "buzzer.h"
#include "main.h"
#include <stdbool.h>

#define CAL_FLASH_MS  100

static CalorieState_t cal_state = CAL_STATE_DISPLAY;
static uint16_t cal_count = 0;
static bool cal_dirty = true;
static uint32_t cal_flash_start = 0;

void Calorie_Init(void)
{
    cal_state = CAL_STATE_DISPLAY;
    cal_count = 0;
    cal_dirty = true;
    cal_flash_start = 0;
}

void Calorie_OnPress(void)
{
    if (cal_state == CAL_STATE_DISPLAY) {
        cal_state = CAL_STATE_EDITING;
        Buzzer_BeepShort();
    } else {
        cal_state = CAL_STATE_DISPLAY;
        Buzzer_BeepDouble();
    }
    cal_flash_start = HAL_GetTick();
    cal_dirty = true;
}

void Calorie_OnScroll(int direction)
{
    if (cal_state != CAL_STATE_EDITING) return;

    if (direction > 0) {
        if (cal_count <= 9990) {
            cal_count += 10;
        }
    } else {
        if (cal_count >= 10) {
            cal_count -= 10;
        } else {
            cal_count = 0;
        }
    }
    cal_dirty = true;
}

void Calorie_OnLongPress(void)
{
    cal_count = 0;
    cal_state = CAL_STATE_DISPLAY;
    cal_dirty = true;
    Buzzer_BeepShort();
}

CalorieState_t Calorie_GetState(void)
{
    return cal_state;
}

uint16_t Calorie_GetCount(void)
{
    return cal_count;
}

bool Calorie_NeedsRedraw(void)
{
    bool d = cal_dirty;
    cal_dirty = false;
    return d;
}

bool Calorie_IsFlashing(void)
{
    return (HAL_GetTick() - cal_flash_start) < CAL_FLASH_MS;
}
