#ifndef __MOTOR_SAFETY_H__
#define __MOTOR_SAFETY_H__

#include <stdint.h>
#include <stdbool.h>

/* 启动阶段：1000 ms 内分 10 级，将输出上限从 0 提升到 30%。 */
#define MOTOR_SAFETY_RAMP_MS               (1000U)
#define MOTOR_SAFETY_RAMP_STEPS            (10U)
#define MOTOR_SAFETY_INITIAL_LIMIT_PERCENT (30U)
/* 启动完成后每 100 ms 再放宽 10%，直至允许完整目标速度。 */
#define MOTOR_SAFETY_POST_STEP_MS           (100U)
#define MOTOR_SAFETY_POST_STEP_PERCENT      (10U)
#define MOTOR_SAFETY_WATCHDOG_MS            (200U)
/* Each wheel must stay at zero before changing its applied direction. */
#define MOTOR_SAFETY_DIRECTION_CHANGE_PAUSE_MS (120U)

typedef enum {
    MOTOR_SAFETY_FAULT_NONE = 0,
    MOTOR_SAFETY_FAULT_UART_TIMEOUT,
    MOTOR_SAFETY_FAULT_WATCHDOG
} MotorSafetyFaultReason;

typedef struct {
    int16_t left_applied;
    int16_t right_applied;
    bool direction_wait;
    MotorSafetyFaultReason fault_reason;
} MotorSafetyDiagnostics;

void Motor_Safety_Init(void);
void Motor_Safety_Arm(void);
void Motor_Safety_Disarm(void);
void Motor_Safety_RequestSpeed(int motor1, int motor2, int motor3, int motor4);
void Motor_Safety_Service(void);
void Motor_Safety_Tick1ms(void);
uint8_t Motor_Safety_IsFaultLatched(void);
void Motor_Safety_GetDiagnostics(MotorSafetyDiagnostics *out);

#endif
