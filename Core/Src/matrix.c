#include "matrix.h"
#include <string.h>
#include <stdbool.h>

/* ================= FRAMEBUFFER ================= */
static uint8_t display_buffer[NUM_ROWS][TOTAL_BYTES];
static volatile uint8_t current_row = 0;

/* ================= BRIGHTNESS ================= */
/*
 * Brightness via TIM3 Output Compare — zero CPU spinning.
 *
 * Timer config: prescaler=15, ARR=999 → 1 MHz clock, 1 ms period
 * Frame rate: 1 MHz / 1000 / 7 rows = ~143 Hz (was ~55 Hz)
 *
 * TIMING PER ROW PERIOD (1 ms):
 *   1. Update interrupt (UIF, counter=0):
 *      → All rows OFF (blanking)
 *      → Shift out new row data (~60-70 µs)
 *      → Turn row ON
 *   2. Compare interrupt (CC1IF, counter=CCR1):
 *      → Turn row OFF
 *
 * SHIFT_OVERHEAD (80 ticks = 80 µs at 1 MHz):
 *   The shift-out takes ~60-70 µs. All CCR1 values in the LUT are
 *   offset above SHIFT_OVERHEAD so the compare event ALWAYS fires
 *   AFTER the row has been turned on. This prevents the glitch where
 *   a small CCR1 would fire before the row was lit, leaving it on
 *   for the entire period (bright flash).
 *
 *   brightness=0:   row never turns on (special case in ISR)
 *   brightness=1:   CCR1=81 → on for 1 µs after shift completes
 *   brightness=255: CCR1=999 → on for full period (919 µs)
 */

/* Must match MX_TIM3_Init() values in main.c */
#define TIMER_ARR        999
#define SHIFT_OVERHEAD    80

static volatile uint16_t brightness_ccr = TIMER_ARR; /* current CCR1 value */
static volatile uint8_t  brightness_off = 0;          /* 1 = display off   */

static volatile uint16_t pending_ccr = TIMER_ARR;
static volatile uint8_t  pending_off = 0;
static volatile bool     ccr_pending = false;

/* ================= ROW PINS ================= */
const uint16_t ROW_PINS[NUM_ROWS] =
{
    A1_Pin, A2_Pin, A3_Pin, A4_Pin,
    A5_Pin, A6_Pin, A7_Pin
};

#define ALL_ROW_PINS (A1_Pin | A2_Pin | A3_Pin | A4_Pin | A5_Pin | A6_Pin | A7_Pin)

/* ================= FONT ================= */
static const uint8_t font5x7[128][5] =
{
[' ']={0,0,0,0,0},

[0x01] = {0x38, 0x44, 0x43, 0x44, 0x38}, //water drop
[0x02] = {0x08, 0x6c, 0x3e, 0x1b, 0x08}, //bolt
[0x03] = {0x07, 0x05, 0x07, 0x00, 0x00}, //degree
[0x04] = {0x00, 0x7f, 0x00, 0x7f, 0x00}, //pause

[':'] = {0x00, 0x36, 0x36, 0x00, 0x00},
[';'] = {0x00, 0x56, 0x36, 0x00, 0x00},
[','] = {0x00, 0x40, 0xE0, 0x00, 0x00},
['.'] = {0x00, 0x60, 0x60, 0x00, 0x00},
['"'] = {0x00, 0x07, 0x07, 0x00, 0x00},
['\''] = {0x00, 0x07, 0x00, 0x00, 0x00},
['/'] = {0x20, 0x10, 0x08, 0x04, 0x02},
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
['_'] = {0x40, 0x40, 0x40, 0x40, 0x40},
['|'] = {0x00, 0x00, 0x7F, 0x00, 0x00},

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
    {0xe4, 0xa4, 0x18, 0x22, 0x67, 0x75, 0x71, 0x8e, 0x81, 0x0c, 0x60},
    {0x94, 0xa4, 0x24, 0x22, 0x92, 0x45, 0x4a, 0x50, 0x83, 0x92, 0x90},
    {0x94, 0xa4, 0x24, 0x22, 0x92, 0x45, 0x4a, 0x50, 0x82, 0x02, 0x90},
    {0xe4, 0x9c, 0x3c, 0x22, 0xd2, 0x47, 0x4a, 0x56, 0x83, 0x84, 0x70},
    {0x94, 0x84, 0x24, 0x2a, 0xb2, 0x45, 0x4a, 0x52, 0x80, 0x88, 0x10},
    {0x94, 0xa4, 0x24, 0x2a, 0x92, 0x45, 0x4a, 0x52, 0x03, 0x90, 0x10},
    {0xe7, 0x98, 0x24, 0x14, 0x92, 0x75, 0x71, 0x8e, 0x81, 0x1e, 0x10},
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

/* ================= BRIGHTNESS LUT ================= */
/*
 * Gamma 1.8 curve, 256 entries.
 * Maps brightness 0-255 → TIM3 CCR1 values.
 *
 * All non-zero entries are >= SHIFT_OVERHEAD+1 to guarantee the
 * compare event fires AFTER the shift-out + row-on sequence.
 * This eliminates the bright-flash glitch at low brightness.
 *
 * Entry 0 = 0 (special: row never turns on).
 * Entry 255 = 999 = ARR (full period, maximum brightness).
 *
 * Generated with:
 *   ccr = SHIFT_OVERHEAD + 1 + (i/255)^1.8 * (ARR - SHIFT_OVERHEAD)
 *   clamped to [0, ARR]
 */
static const uint16_t brightness_lut[256] = {
       0,   81,   82,   83,   84,   85,   86,   87,   88,   89,   91,   92,   94,   95,   97,   98,
     100,  102,  103,  105,  107,  109,  110,  112,  114,  116,  118,  120,  122,  124,  126,  129,
     131,  133,  135,  137,  140,  142,  144,  147,  149,  151,  154,  156,  159,  161,  164,  166,
     169,  171,  174,  177,  179,  182,  184,  187,  190,  193,  195,  198,  201,  204,  207,  209,
     212,  215,  218,  221,  224,  227,  230,  233,  236,  239,  242,  245,  248,  251,  254,  257,
     260,  264,  267,  270,  273,  276,  279,  283,  286,  289,  293,  296,  299,  302,  306,  309,
     313,  316,  319,  323,  326,  330,  333,  336,  340,  343,  347,  350,  354,  358,  361,  365,
     368,  372,  375,  379,  383,  386,  390,  394,  397,  401,  405,  409,  412,  416,  420,  424,
     427,  431,  435,  439,  443,  446,  450,  454,  458,  462,  466,  470,  474,  478,  482,  485,
     489,  493,  497,  501,  505,  509,  513,  517,  522,  526,  530,  534,  538,  542,  546,  550,
     554,  558,  563,  567,  571,  575,  579,  584,  588,  592,  596,  601,  605,  609,  613,  618,
     622,  626,  631,  635,  639,  644,  648,  652,  657,  661,  665,  670,  674,  679,  683,  688,
     692,  696,  701,  705,  710,  714,  719,  723,  728,  732,  737,  742,  746,  751,  755,  760,
     764,  769,  774,  778,  783,  788,  792,  797,  802,  806,  811,  816,  820,  825,  830,  834,
     839,  844,  849,  853,  858,  863,  868,  873,  877,  882,  887,  892,  897,  901,  906,  911,
     916,  921,  926,  931,  936,  940,  945,  950,  955,  960,  965,  970,  975,  980,  985,  990,
};

/**
 * @brief Set display brightness
 * @param brightness 0 = display off, 1 = dimmest, 255 = full brightness
 */
void Matrix_SetBrightness(uint8_t brightness)
{
    if (brightness == 0) {
        pending_off = 1;
        pending_ccr = 0;
    } else {
        pending_off = 0;
        pending_ccr = brightness_lut[brightness];
    }
    ccr_pending = true;
}

uint8_t Matrix_GetBrightness(void)
{
    if (brightness_off) return 0;
    uint16_t val = brightness_ccr;
    for (int i = 255; i >= 0; i--) {
        if (brightness_lut[i] <= val) return (uint8_t)i;
    }
    return 0;
}

/* ================= CORE SCAN ================= */
void Matrix_Task(void)
{
    /* No longer used — display driven entirely by TIM3 ISR. */
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

/* ================= TIM3 ISR — DIRECT REGISTER ACCESS, NO HAL ================= */
/*
 * Called from TIM3_IRQHandler() in stm32g0xx_it.c (bypasses HAL).
 *
 * UIF  (update/overflow) → blank all rows, shift data, turn ON row
 * CC1IF (compare match)  → turn OFF row (brightness cutoff)
 *
 * CC1 always fires AFTER the shift completes because all LUT values
 * are >= SHIFT_OVERHEAD. At brightness=255, CCR1=ARR=999 so CC1IF
 * fires right at the overflow boundary — row stays on full period.
 */
void Matrix_TIM3_IRQHandler(void)
{
    uint32_t sr = TIM3->SR;

    /* ---- Update (overflow): advance row, shift data, turn ON ---- */
    if (sr & TIM_SR_UIF) {
        TIM3->SR = ~TIM_SR_UIF;

        /* Blank all rows immediately */
        GPIOA->BSRR = ALL_ROW_PINS;

        /* Apply pending brightness change while all rows are OFF */
        if (ccr_pending) {
            brightness_off = pending_off;
            brightness_ccr = pending_ccr;
            TIM3->CCR1 = pending_ccr;
            ccr_pending = false;
        }

        /* Shift out data for current row */
        ShiftOutRow(display_buffer[current_row]);

        /* Turn ON current row (unless brightness is zero) */
        if (!brightness_off) {
            GPIOA->BRR = ROW_PINS[current_row];
        }

        /* Advance to next row */
        current_row++;
        if (current_row >= NUM_ROWS) current_row = 0;
    }

    /* ---- Compare match: turn OFF the row ---- */
    if (sr & TIM_SR_CC1IF) {
        TIM3->SR = ~TIM_SR_CC1IF;
        GPIOA->BSRR = ALL_ROW_PINS;
    }
}

/* ================= BATTERY ICON ================= */
/*
 * 15-column sprite. Each byte = 1 column, bit 0 = row 0 (top).
 *
 *   Col  0:     0x00  spacer
 *   Col  1:     0x3E  left wall  (rows 1-5)
 *   Col  2-11:  0x22  top/bottom border (rows 1,5) — fillable
 *   Col 12:     0x3E  right wall (rows 1-5)
 *   Col 13:     0x1C  nub (rows 2-4)
 *   Col 14:     0x00  spacer
 *
 * Filling a bar = OR 0x1C (rows 2-4) onto columns 2-11.
 */
static const uint8_t battery_shell[15] = {
    0x00, 0x3E, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
    0x22, 0x22, 0x22, 0x22, 0x3E, 0x1C, 0x00
};

void Matrix_DrawBatteryIcon_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int col, uint8_t soc)
{
    /* 10 bars, thresholds: 5,15,25,35,45,55,65,75,85,95 */
    uint8_t filled_bars = 0;
    if (soc >= 5) {
        filled_bars = (uint8_t)((soc - 5) / 10) + 1;
        if (filled_bars > 10) filled_bars = 10;
    }

    for (int i = 0; i < 15; i++) {
        uint8_t col_data = battery_shell[i];

        /* Fill interior columns 2-11 */
        if (i >= 2 && i <= 11) {
            uint8_t bar_index = (uint8_t)(i - 2);
            if (bar_index < filled_bars) {
                col_data |= 0x1C;  /* rows 2,3,4 */
            }
        }

        int dst_col = col + i;
        if (dst_col < 0 || dst_col >= NUM_COLS) continue;

        for (int r = 0; r < NUM_ROWS; r++) {
            if (col_data & (1 << r)) {
                set_pixel_buf(buf, r, dst_col, 1);
            }
        }
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

int Matrix_TextPixelWidth(const char *text)
{
    return text_pixel_width(text);
}

void Matrix_DrawBatteryIcon_Blink_Buf(uint8_t buf[NUM_ROWS][TOTAL_BYTES],
                                       int col, uint8_t soc, uint8_t blink_bar)
{
    uint8_t filled_bars = 0;
    if (soc >= 5) {
        filled_bars = (uint8_t)((soc - 5) / 10) + 1;
        if (filled_bars > 10) filled_bars = 10;
    }

    for (int i = 0; i < 15; i++) {
        uint8_t col_data = battery_shell[i];

        if (i >= 2 && i <= 11) {
            uint8_t bar_index = (uint8_t)(i - 2);
            if (bar_index < filled_bars || bar_index == blink_bar) {
                col_data |= 0x1C;
            }
        }

        int dst_col = col + i;
        if (dst_col < 0 || dst_col >= NUM_COLS) continue;

        for (int r = 0; r < NUM_ROWS; r++) {
            if (col_data & (1 << r)) {
                set_pixel_buf(buf, r, dst_col, 1);
            }
        }
    }
}
