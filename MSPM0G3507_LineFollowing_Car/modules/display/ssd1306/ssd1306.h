#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stdint.h>

#define SSD1306_ADDRESS_7BIT 0x3CU
#define SSD1306_WIDTH 128U
#define SSD1306_PAGES 8U

/* 128x64 page-buffered driver over the PA10/PA11 software I2C bus.
 * Every call is fail-soft: a missing display only returns false. */
bool Ssd1306_Init(void);
void Ssd1306_ClearBuffer(void);
/* 6-pixel wide 5x7 font; column is a character cell (0..20). */
void Ssd1306_DrawText(uint8_t page, uint8_t column, const char *text);
/* Send only the pages touched since the last successful flush. */
bool Ssd1306_FlushDirty(void);
/* Send at most 16 display bytes so one call fits the 2 ms scheduler budget. */
bool Ssd1306_FlushNextChunk(void);

#endif
