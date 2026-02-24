#ifndef MATRIX_H
#define MATRIX_H

#include "main.h"
#include <stdint.h>

#define NUM_ROWS 7
#define NUM_COLS 84
#define TOTAL_BYTES 11

extern const uint8_t kompressor_logo[NUM_ROWS][TOTAL_BYTES];
extern const uint8_t buy_a_wd[NUM_ROWS][TOTAL_BYTES];

void Matrix_Init(void);
void Matrix_Task(void);

void Matrix_Clear(void);
void Matrix_Fill(void);
void Matrix_SetPixel(int r, int c, uint8_t state);

void Matrix_DrawChar(int row, int col, char c);
void Matrix_DrawText(int row, int col, const char *text);
void Matrix_ScrollText(const char *text, int offset);

void Matrix_DrawBitmap(const uint8_t bitmap[NUM_ROWS][TOTAL_BYTES]);

// Load a pre-rendered buffer directly into the display buffer
void Matrix_LoadBuffer(const uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

// Brightness control (0 = off, 1 = dimmest, 255 = full brightness)
void Matrix_SetBrightness(uint8_t brightness);
uint8_t Matrix_GetBrightness(void);

// Draw into an arbitrary buffer (for offscreen rendering by screens.c)
void Matrix_DrawChar_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row, int col, char c);
void Matrix_DrawText_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row, int col, const char *text);
void Matrix_DrawTextCentered_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row, const char *text);
void Matrix_DrawTextRight_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row, const char *text);
void Matrix_DrawBitmap_Buf(uint8_t dest[NUM_ROWS][TOTAL_BYTES], const uint8_t src[NUM_ROWS][TOTAL_BYTES]);

void Matrix_ScrollText_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row,
                           const char *text, int *scroll_offset,
                           uint32_t speed_ms, uint32_t *last_scroll_tick);

void Matrix_TIM3_IRQHandler(void);

#endif
