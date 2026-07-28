.syntax unified
.thumb

.extern BootTrace_Mark
.extern Default_Handler
.extern FreeRTOS_SVC_Handler
.extern FreeRTOS_vRestoreContextOfFirstTask

/* Preserve the exception return in LR and keep SP 8-byte aligned across BL. */
.global SVC_Handler
.type SVC_Handler,%function
.thumb_func
SVC_Handler:
    push {r4, r5}
    mov  r5, lr
    push {r4, r5}
    movs r0, #5
    bl   BootTrace_Mark
    pop  {r4, r5}
    mov  lr, r5
    pop  {r4, r5}
    b    FreeRTOS_SVC_Handler
.size SVC_Handler, .-SVC_Handler

/* The C port calls this wrapper; the official restore never returns. */
.global vRestoreContextOfFirstTask
.type vRestoreContextOfFirstTask,%function
.thumb_func
vRestoreContextOfFirstTask:
    push {r4, r5}
    mov  r5, lr
    push {r4, r5}
    movs r0, #6
    bl   BootTrace_Mark
    pop  {r4, r5}
    mov  lr, r5
    pop  {r4, r5}
    b    FreeRTOS_vRestoreContextOfFirstTask
.size vRestoreContextOfFirstTask, .-vRestoreContextOfFirstTask

/* TI's startup file gives every unused vector its own weak alias to the local
 * default loop.  Strong vector trampolines are therefore needed for E3 to
 * reach the application Default_Handler.  Active TIMG0, UART1, PendSV,
 * SysTick, SVC and HardFault handlers are intentionally excluded. */
.macro BOOT_DEFAULT_VECTOR name
.global \name
.type \name,%function
.thumb_func
\name:
    b Default_Handler
.size \name, .-\name
.endm

BOOT_DEFAULT_VECTOR NMI_Handler
BOOT_DEFAULT_VECTOR GROUP0_IRQHandler
BOOT_DEFAULT_VECTOR GROUP1_IRQHandler
BOOT_DEFAULT_VECTOR TIMG8_IRQHandler
BOOT_DEFAULT_VECTOR UART3_IRQHandler
BOOT_DEFAULT_VECTOR ADC0_IRQHandler
BOOT_DEFAULT_VECTOR ADC1_IRQHandler
BOOT_DEFAULT_VECTOR CANFD0_IRQHandler
BOOT_DEFAULT_VECTOR DAC0_IRQHandler
BOOT_DEFAULT_VECTOR SPI0_IRQHandler
BOOT_DEFAULT_VECTOR SPI1_IRQHandler
BOOT_DEFAULT_VECTOR UART2_IRQHandler
BOOT_DEFAULT_VECTOR UART0_IRQHandler
BOOT_DEFAULT_VECTOR TIMG6_IRQHandler
BOOT_DEFAULT_VECTOR TIMA0_IRQHandler
BOOT_DEFAULT_VECTOR TIMA1_IRQHandler
BOOT_DEFAULT_VECTOR TIMG7_IRQHandler
BOOT_DEFAULT_VECTOR TIMG12_IRQHandler
BOOT_DEFAULT_VECTOR I2C0_IRQHandler
BOOT_DEFAULT_VECTOR I2C1_IRQHandler
BOOT_DEFAULT_VECTOR AES_IRQHandler
BOOT_DEFAULT_VECTOR RTC_IRQHandler
BOOT_DEFAULT_VECTOR DMA_IRQHandler
