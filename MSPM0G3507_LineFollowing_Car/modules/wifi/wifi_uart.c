#include "wifi_uart.h"

#include "ti_msp_dl_config.h"

#define WIFI_UART_RX_BUFFER_SIZE (32U)
#define WIFI_UART_RX_BUFFER_MASK (WIFI_UART_RX_BUFFER_SIZE - 1U)

static WifiAtProbe wifi_probe;
static volatile uint8_t rx_buffer[WIFI_UART_RX_BUFFER_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static bool tx_pending;
static uint8_t tx_index;

static void service_pending_transmit(void)
{
    static const uint8_t at_command[] = {'A', 'T', '\r', '\n'};

    while (tx_pending) {
        if (DL_UART_Main_isTXFIFOFull(Wifi_INST)) {
            return;
        }

        DL_UART_Main_transmitData(Wifi_INST, at_command[tx_index]);
        tx_index++;
        if (tx_index == sizeof(at_command)) {
            tx_pending = false;
            tx_index = 0U;
        }
    }
}

void WifiUart_Init(uint32_t now_ms)
{
    rx_head = 0U;
    rx_tail = 0U;
    tx_pending = false;
    tx_index = 0U;
    WifiAtProbe_Init(&wifi_probe, now_ms);

    DL_UART_Main_clearInterruptStatus(Wifi_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(Wifi_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(Wifi_INST_INT_IRQN);
    NVIC_EnableIRQ(Wifi_INST_INT_IRQN);
}

void WifiUart_Service(uint32_t now_ms)
{
    while (rx_tail != rx_head) {
        WifiAtProbe_Feed(&wifi_probe, rx_buffer[rx_tail & WIFI_UART_RX_BUFFER_MASK]);
        rx_tail++;
    }

    WifiAtProbe_Service(&wifi_probe, now_ms);
    if (WifiAtProbe_TakeSendRequest(&wifi_probe, now_ms)) {
        tx_pending = true;
        tx_index = 0U;
    }
    service_pending_transmit();
}

WifiAtProbeState WifiUart_GetProbeState(void)
{
    return WifiAtProbe_GetState(&wifi_probe);
}

void Wifi_INST_IRQHandler(void)
{
    uint8_t received_byte;

    if (DL_UART_getPendingInterrupt(Wifi_INST) != DL_UART_IIDX_RX) {
        return;
    }

    received_byte = DL_UART_Main_receiveData(Wifi_INST);
    if ((uint8_t)(rx_head - rx_tail) < WIFI_UART_RX_BUFFER_SIZE) {
        rx_buffer[rx_head & WIFI_UART_RX_BUFFER_MASK] = received_byte;
        rx_head++;
    }
}
