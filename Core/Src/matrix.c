#include "matrix.h"
#include <string.h>

/* ================= FRAMEBUFFER ================= */
static uint8_t display_buffer[NUM_ROWS][TOTAL_BYTES];
static uint8_t current_row = 0;

/* ================= BRIGHTNESS ================= */
/*
 * Brightness is controlled via TIM3 Output Compare Channel 1.
 *
 * TIM3 Update interrupt (counter=0): shift out data, turn row ON.
 * TIM3 OC1 interrupt (counter=compare): turn row OFF.
 *
 * brightness_compare controls when the row turns off within each period.
 * Lower value = row is on for less time = dimmer.
 * Value of 0 = row is never turned on (display off).
 * Value >= period = row stays on the full cycle (max brightness).
 */
static volatile uint16_t brightness_compare = 259; /* default: full brightness */

/* ================= ROW PINS ================= */
const uint16_t ROW_PINS[NUM_ROWS] =
{
    A1_Pin, A2_Pin, A3_Pin, A4_Pin,
    A5_Pin, A6_Pin, A7_Pin
};

/* Precomputed combined bitmask of all row pins for fast blanking */
#define ALL_ROW_PINS (A1_Pin | A2_Pin | A3_Pin | A4_Pin | A5_Pin | A6_Pin | A7_Pin)

/* ================= FONT ================= */
static const uint8_t font5x7[128][5] =
{
[' ']={0,0,0,0,0},

[0x01] = {0x38, 0x44, 0x43, 0x44, 0x38}, //water drop
[0x02] = {0x08, 0x6c, 0x3e, 0x1b, 0x08}, //bolt
[0x03] = {0x07, 0x05, 0x07, 0x00, 0x00}, //degree

[':'] = {0x00, 0x36, 0x36, 0x00, 0x00},
[';'] = {0x00, 0x56, 0x36, 0x00, 0x00},
[','] = {0x00, 0x40, 0xE0, 0x00, 0x00},
['.'] = {0x00, 0x60, 0x60, 0x00, 0x00},
['"'] = {0x00, 0x07, 0x07, 0x00, 0x00},
['\''] = {0x00, 0x07, 0x00, 0x00, 0x00},
['-'] = {0x08, 0x08, 0x08, 0x08, 0x08},
['+'] = {0x08, 0x08, 0x3E, 0x08, 0x08},
['*'] = {0x14, 0x08, 0x3E, 0x08, 0x14},
['('] = {0x00, 0x1C, 0x22, 0x41, 0x00},
[')'] = {0x00, 0x41, 0x22, 0x1C, 0x00},
['$'] = {0x24, 0x2A, 0x7F, 0x2A, 0x12},
['#'] = {0x14, 0x7F, 0x14, 0x7F, 0x14},
['@'] = {0x3E, 0x45, 0x4D, 0x55, 0x3E},
['!'] = {0x00, 0x00, 0x4F, 0x00, 0x00},
['?'] = {0x02, 0x01, 0x51, 0x09, 0x06},
['^'] = {0x04, 0x02, 0x01, 0x02, 0x04},
['&'] = {0x32, 0x49, 0x49, 0x26, 0x50},
['<'] = {0x08, 0x14, 0x22, 0x41, 0x00},
['>'] = {0x00, 0x41, 0x22, 0x14, 0x08},
['%'] = {0x23, 0x13, 0x08, 0x64, 0x62},

['0']={0x3E,0x51,0x49,0x45,0x3E},
['1']={0x00,0x42,0x7F,0x40,0x00},
['2']={0x62,0x51,0x49,0x49,0x46},
['3']={0x22,0x49,0x49,0x49,0x36},
['4']={0x18,0x14,0x12,0x7F,0x10},
['5']={0x2F,0x49,0x49,0x49,0x31},
['6']={0x3E,0x49,0x49,0x49,0x32},
['7']={0x01,0x71,0x09,0x05,0x03},
['8']={0x36,0x49,0x49,0x49,0x36},
['9']={0x26,0x49,0x49,0x49,0x3E},

['A']={0x7E,0x11,0x11,0x11,0x7E},
['B']={0x7F,0x49,0x49,0x49,0x36},
['C']={0x3E,0x41,0x41,0x41,0x22},
['D']={0x7F,0x41,0x41,0x22,0x1C},
['E']={0x7F,0x49,0x49,0x49,0x41},
['F']={0x7F,0x09,0x09,0x09,0x01},
['G']={0x3E,0x41,0x49,0x49,0x7A},
['H']={0x7F,0x08,0x08,0x08,0x7F},
['I']={0x00,0x41,0x7F,0x41,0x00},
['J']={0x20,0x40,0x41,0x3F,0x01},
['K']={0x7F,0x08,0x14,0x22,0x41},
['L']={0x7F,0x40,0x40,0x40,0x40},
['M']={0x7F,0x02,0x0C,0x02,0x7F},
['N']={0x7F,0x04,0x08,0x10,0x7F},
['O']={0x3E,0x41,0x41,0x41,0x3E},
['P']={0x7F,0x09,0x09,0x09,0x06},
['Q']={0x3E,0x41,0x51,0x21,0x5E},
['R']={0x7F,0x09,0x19,0x29,0x46},
['S']={0x46,0x49,0x49,0x49,0x31},
['T']={0x01,0x01,0x7F,0x01,0x01},
['U']={0x3F,0x40,0x40,0x40,0x3F},
['V']={0x1F,0x20,0x40,0x20,0x1F},
['W']={0x7F,0x20,0x18,0x20,0x7F},
['X']={0x63,0x14,0x08,0x14,0x63},
['Y']={0x03,0x04,0x78,0x04,0x03},
['Z']={0x61,0x51,0x49,0x45,0x43},

['a']={0x20,0x54,0x54,0x54,0x78},
['b']={0x7F,0x48,0x44,0x44,0x38},
['c']={0x38,0x44,0x44,0x44,0x20},
['d']={0x38,0x44,0x44,0x48,0x7F},
['e']={0x38,0x54,0x54,0x54,0x18},
['f']={0x08,0x7E,0x09,0x01,0x02},
['g']={0x0C,0x52,0x52,0x52,0x3E},
['h']={0x7F,0x08,0x04,0x04,0x78},
['i']={0x00,0x44,0x7D,0x40,0x00},
['j']={0x20,0x40,0x44,0x3D,0x00},
['k']={0x7F,0x10,0x28,0x44,0x00},
['l']={0x00,0x41,0x7F,0x40,0x00},
['m']={0x7C,0x04,0x18,0x04,0x78},
['n']={0x7C,0x08,0x04,0x04,0x78},
['o']={0x38,0x44,0x44,0x44,0x38},
['p']={0x7C,0x14,0x14,0x14,0x08},
['q']={0x08,0x14,0x14,0x18,0x7C},
['r']={0x7C,0x08,0x04,0x04,0x08},
['s']={0x48,0x54,0x54,0x54,0x20},
['t']={0x04,0x3F,0x44,0x40,0x20},
['u']={0x3C,0x40,0x40,0x20,0x7C},
['v']={0x1C,0x20,0x40,0x20,0x1C},
['w']={0x3C,0x40,0x30,0x40,0x3C},
['x']={0x44,0x28,0x10,0x28,0x44},
['y']={0x0C,0x50,0x50,0x50,0x3C},
['z']={0x44,0x64,0x54,0x4C,0x44}
};

const uint8_t kompressor_logo[NUM_ROWS][TOTAL_BYTES] = {
    {0x01, 0x2e, 0xfb, 0xbd, 0xdd, 0xdd, 0xe0, 0x94, 0x77, 0x48, 0x00},
    {0x01, 0x2a, 0xaa, 0xa5, 0x11, 0x15, 0x20, 0x94, 0x54, 0x48, 0x00},
    {0x01, 0x6a, 0xaa, 0xa5, 0x11, 0x15, 0x20, 0xb4, 0x54, 0x58, 0x00},
    {0x01, 0xca, 0xab, 0xbd, 0xdd, 0xd5, 0xe0, 0xe4, 0x54, 0x70, 0x00},
    {0x01, 0x6a, 0x8a, 0x29, 0x04, 0x55, 0x40, 0xb4, 0x54, 0x58, 0x00},
    {0x01, 0x2a, 0x8a, 0x2d, 0x04, 0x55, 0x60, 0x94, 0x54, 0x48, 0x00},
    {0x01, 0x2e, 0x8a, 0x25, 0xdd, 0xdd, 0x20, 0x97, 0x77, 0x48, 0x00},
};

const uint8_t buy_a_wd[NUM_ROWS][TOTAL_BYTES] = {
		0xe4, 0xa4, 0x18, 0x22, 0x67, 0x75, 0x71, 0x8e, 0x81, 0x0c, 0x60, 0x94, 0xa4, 0x24, 0x22, 0x92,
		0x45, 0x4a, 0x50, 0x83, 0x92, 0x90, 0x94, 0xa4, 0x24, 0x22, 0x92, 0x45, 0x4a, 0x50, 0x82, 0x02,
		0x90, 0xe4, 0x9c, 0x3c, 0x22, 0xd2, 0x47, 0x4a, 0x56, 0x83, 0x84, 0x70, 0x94, 0x84, 0x24, 0x2a,
		0xb2, 0x45, 0x4a, 0x52, 0x80, 0x88, 0x10, 0x94, 0xa4, 0x24, 0x2a, 0x92, 0x45, 0x4a, 0x52, 0x03,
		0x90, 0x10, 0xe7, 0x98, 0x24, 0x14, 0x92, 0x75, 0x71, 0x8e, 0x81, 0x1e, 0x10
	};

/* ================= INTERNAL HELPERS ================= */

static uint8_t reverse_byte(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

static void set_pixel_buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int r, int c, uint8_t state)
{
    if (r < 0 || r >= NUM_ROWS || c < 0 || c >= NUM_COLS) return;
    int byte = c / 8;
    int bit = c % 8;
    if (state)
        buf[r][byte] |= (1 << bit);
    else
        buf[r][byte] &= ~(1 << bit);
}

/* ================= PROPORTIONAL FONT HELPERS ================= */

static int char_width(char c)
{
    if (c == ' ') return 2;
    if (c == '1') return 6;

    const uint8_t *glyph = font5x7[(int)(unsigned char)c];

    int first = -1, last = -1;
    for (int x = 0; x < 5; x++) {
        if (glyph[x] != 0) {
            if (first < 0) first = x;
            last = x;
        }
    }

    if (first < 0) return 2;
    return (last - first + 1) + 1;
}

static int text_pixel_width(const char *text)
{
    int w = 0;
    while (*text) {
        w += char_width(*text);
        text++;
    }
    return w;
}

/* ================= OPTIMIZED SHIFT ================= */

static inline void ShiftOutRow(const uint8_t *row_data)
{
    GPIO_TypeDef *port = GPIOA;
    const uint32_t data_pin = DATA_Pin;
    const uint32_t clk_pin  = SRCLK_Pin;

    #define SHIFT_BIT(byte_val, bit_n)          \
        if ((byte_val) & (1u << (bit_n)))       \
            port->BRR  = data_pin;              \
        else                                    \
            port->BSRR = data_pin;              \
        port->BSRR = clk_pin;                   \
        port->BRR  = clk_pin;

    for (int b = 0; b < 10; b++)
    {
        uint8_t v = row_data[b];
        SHIFT_BIT(v, 0)
        SHIFT_BIT(v, 1)
        SHIFT_BIT(v, 2)
        SHIFT_BIT(v, 3)
        SHIFT_BIT(v, 4)
        SHIFT_BIT(v, 5)
        SHIFT_BIT(v, 6)
        SHIFT_BIT(v, 7)
    }

    {
        uint8_t v = row_data[10];
        SHIFT_BIT(v, 0)
        SHIFT_BIT(v, 1)
        SHIFT_BIT(v, 2)
        SHIFT_BIT(v, 3)
    }

    #undef SHIFT_BIT

    port->BSRR = RCLK_Pin;
    port->BRR  = RCLK_Pin;
}

/* ================= BRIGHTNESS CONTROL ================= */

/**
 * @brief Set display brightness
 * @param brightness 0 = display off, 1 = dimmest, 255 = full brightness
 *
 * This works by adjusting the TIM3 Channel 1 Output Compare value.
 * The Update interrupt (counter overflow) turns the row ON.
 * The OC1 interrupt turns the row OFF.
 * A smaller compare value means the row is on for less time = dimmer.
 *
 * Uses a quadratic curve for perceptually linear brightness.
 */
void Matrix_SetBrightness(uint8_t brightness)
{
    extern TIM_HandleTypeDef htim3;
    uint16_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);

    if (brightness == 0) {
        brightness_compare = 0;
    } else if (brightness >= 255) {
        brightness_compare = period + 1; /* OC never fires = full on */
    } else {
        /* Quadratic curve for perceptually linear dimming */
        uint32_t b = (uint32_t)brightness;
        brightness_compare = (uint16_t)((b * b * period) / (255UL * 255UL));
        if (brightness_compare == 0) brightness_compare = 1;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, brightness_compare);
}

uint8_t Matrix_GetBrightness(void)
{
    extern TIM_HandleTypeDef htim3;
    uint16_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);

    if (brightness_compare == 0) return 0;
    if (brightness_compare > period) return 255;

    /* Reverse the quadratic mapping */
    uint32_t val = (uint32_t)brightness_compare * 255UL * 255UL / period;
    uint32_t root = 1;
    while (root * root < val) root++;
    if (root > 255) root = 255;
    return (uint8_t)root;
}

/* ================= CORE SCAN ================= */
void Matrix_Task(void)
{
    static uint32_t last_scan = 0;
    if (HAL_GetTick() - last_scan < 1) return;
    last_scan = HAL_GetTick();

    for (int r = 0; r < NUM_ROWS; r++)
        HAL_GPIO_WritePin(GPIOA, ROW_PINS[r], GPIO_PIN_SET);

    ShiftOutRow(display_buffer[current_row]);

    HAL_GPIO_WritePin(GPIOA, ROW_PINS[current_row], GPIO_PIN_RESET);

    current_row++;
    if (current_row >= NUM_ROWS) current_row = 0;
}

/* ================= DRAW (display buffer) ================= */
void Matrix_Init(void)  { Matrix_Clear(); }
void Matrix_Clear(void) { memset(display_buffer, 0, sizeof(display_buffer)); }
void Matrix_Fill(void)  { memset(display_buffer, 0xFF, sizeof(display_buffer)); }

void Matrix_SetPixel(int r, int c, uint8_t state)
{
    set_pixel_buf(display_buffer, r, c, state);
}

void Matrix_DrawChar(int row, int col, char c)
{
    Matrix_DrawChar_Buf(display_buffer, row, col, c);
}

void Matrix_DrawText(int row, int col, const char *text)
{
    Matrix_DrawText_Buf(display_buffer, row, col, text);
}

void Matrix_ScrollText(const char *text, int offset)
{
    int x = -offset;
    while (*text) {
        Matrix_DrawChar(0, x, *text);
        x += char_width(*text);
        text++;
    }
}

void Matrix_DrawBitmap(const uint8_t bitmap[NUM_ROWS][TOTAL_BYTES])
{
    Matrix_DrawBitmap_Buf(display_buffer, bitmap);
}

void Matrix_LoadBuffer(const uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    memcpy(display_buffer, buf, sizeof(display_buffer));
}

/* ================= DRAW (arbitrary buffer) ================= */

void Matrix_DrawChar_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row, int col, char c)
{
    if (c == ' ') return;

    const uint8_t *glyph = font5x7[(int)(unsigned char)c];

    if (c == '1') {
        for (int x = 0; x < 5; x++) {
            uint8_t column = glyph[x];
            for (int y = 0; y < 7; y++) {
                set_pixel_buf(buf, row + y, col + x, (column >> y) & 1);
            }
        }
        return;
    }

    int first = -1;
    for (int x = 0; x < 5; x++) {
        if (glyph[x] != 0) {
            first = x;
            break;
        }
    }
    if (first < 0) return;

    int last = first;
    for (int x = 4; x >= first; x--) {
        if (glyph[x] != 0) {
            last = x;
            break;
        }
    }

    int draw_col = col;
    for (int x = first; x <= last; x++) {
        uint8_t column = glyph[x];
        for (int y = 0; y < 7; y++) {
            set_pixel_buf(buf, row + y, draw_col, (column >> y) & 1);
        }
        draw_col++;
    }
}

void Matrix_DrawText_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row, int col, const char *text)
{
    while (*text) {
        Matrix_DrawChar_Buf(buf, row, col, *text);
        col += char_width(*text);
        text++;
    }
}

void Matrix_DrawTextCentered_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row, const char *text)
{
    int w = text_pixel_width(text);
    int col = (NUM_COLS - w) / 2;
    Matrix_DrawText_Buf(buf, row, col, text);
}

void Matrix_DrawTextRight_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row, const char *text)
{
    int w = text_pixel_width(text);
    int col = NUM_COLS - w;
    Matrix_DrawText_Buf(buf, row, col, text);
}

void Matrix_DrawBitmap_Buf(uint8_t dest[NUM_ROWS][TOTAL_BYTES], const uint8_t src[NUM_ROWS][TOTAL_BYTES])
{
    for (int r = 0; r < NUM_ROWS; r++)
        for (int b = 0; b < TOTAL_BYTES; b++)
            dest[r][b] = reverse_byte(src[r][b]);
}

/* ================= ISR CALLBACKS ================= */

/**
 * TIM3 Update interrupt: fires at counter overflow (start of each period).
 * Blanks all rows, shifts out new data, turns on the current row.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        /* Turn OFF all rows in a single register write */
        GPIOA->BSRR = ALL_ROW_PINS;

        /* Minimal dead time */
        __NOP(); __NOP(); __NOP(); __NOP();

        /* Shift out next row data */
        ShiftOutRow(display_buffer[current_row]);

        /* Only turn row on if brightness > 0 */
        if (brightness_compare > 0) {
            GPIOA->BRR = ROW_PINS[current_row];
        }

        current_row = (current_row + 1) % NUM_ROWS;
    }
}

/**
 * TIM3 OC1 interrupt: fires when counter reaches the compare value.
 * Turns off the active row, ending its on-time.
 * This is what controls brightness — earlier fire = dimmer.
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        /* Turn OFF all rows */
        GPIOA->BSRR = ALL_ROW_PINS;
    }
}

void Matrix_ScrollText_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int row,
                           const char *text, int *scroll_offset,
                           uint32_t speed_ms, uint32_t *last_scroll_tick)
{
    uint32_t now = HAL_GetTick();
    if (now - *last_scroll_tick >= speed_ms) {
        (*scroll_offset)++;
        *last_scroll_tick = now;
    }

    int text_width = text_pixel_width(text);

    int total_scroll = NUM_COLS + text_width;
    int offset = *scroll_offset % total_scroll;

    int x = NUM_COLS - offset;
    Matrix_DrawText_Buf(buf, row, x, text);
}
