#ifndef BSP_LINE_MUX_H
#define BSP_LINE_MUX_H

#include <stdbool.h>
#include <stdint.h>

void BSP_LineMux_SelectChannel(uint8_t channel);
bool BSP_LineMux_IsBlack(void);

#endif
