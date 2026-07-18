/*
 * empty.c
 *
 * MPU6050 I2C扫描测试 - 尝试两个地址
 */

#include "ti_msp_dl_config.h"
#include "usart.h"
#include "delay.h"
#include "bsp_mpu6050.h"

void PrintStr(char *str)
{
    while(*str) { USART_SendData(*str++); }
}

void PrintHex(uint8_t num)
{
    char hex[] = "0123456789ABCDEF";
    USART_SendData(hex[(num >> 4) & 0x0F]);
    USART_SendData(hex[num & 0x0F]);
}

int main(void)
{
    SYSCFG_DL_init();
    USART_Init();

    PrintStr("\r\n=== MPU6050 I2C Scan ===\r\n");

    unsigned char Re[2] = {0};

    // 尝试地址 0x68 (AD0接地)
    PrintStr("Try addr 0x68: ");
    MPU6050_ReadData(0x68, 0x75, 1, Re);
    PrintHex(Re[0]);
    if(Re[0] == 0x68)
    {
        PrintStr(" -> Found!\r\n");
    }
    else
    {
        PrintStr(" -> Not found\r\n");
    }

    // 尝试地址 0x69 (AD0接VCC)
    Re[0] = 0;
    PrintStr("Try addr 0x69: ");
    MPU6050_ReadData(0x69, 0x75, 1, Re);
    PrintHex(Re[0]);
    if(Re[0] == 0x68)
    {
        PrintStr(" -> Found!\r\n");
    }
    else
    {
        PrintStr(" -> Not found\r\n");
    }

    PrintStr("\r\nDone.\r\n");

    while(1)
    {
        delay_ms(1000);
    }
}
