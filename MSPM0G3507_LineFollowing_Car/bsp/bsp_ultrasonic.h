#ifndef BSP_ULTRASONIC_H
#define BSP_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

void BSP_Ultrasonic_Init(void);
uint32_t BSP_Ultrasonic_NowUs(void);
void BSP_Ultrasonic_SetTrig(bool high);

#endif
