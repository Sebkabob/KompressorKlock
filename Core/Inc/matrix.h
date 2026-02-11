#ifndef MATRIX_H
#define MATRIX_H

#include "main.h"
#include <stdint.h>

#define NUM_ROWS 7
#define NUM_COLS 84
#define TOTAL_BYTES 11

void Matrix_Init(void);
void Matrix_Task(void);

void Matrix_Clear(void);
void Matrix_SetPixel(int r,int c,uint8_t state);

void Matrix_DrawChar(int row,int col,char c);
void Matrix_DrawText(int row,int col,const char *text);
void Matrix_ScrollText(const char *text,int offset);

#endif
