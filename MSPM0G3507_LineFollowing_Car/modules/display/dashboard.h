#ifndef APPLICATION_DIAGNOSTICS_DASHBOARD_H
#define APPLICATION_DIAGNOSTICS_DASHBOARD_H

#include <stdbool.h>
#include <stdint.h>

/* One actionable code per class of trouble. The first three are
 * recoverable observations; the last three are latched by SafetyTask
 * and stop the car until reset. */
typedef enum {
    APP_FAULT_NONE = 0,
    APP_FAULT_CORNER_SEARCH,
    APP_FAULT_LINE_LOST,
    APP_FAULT_OLED_I2C,
    APP_FAULT_MOTOR_UART,
    APP_FAULT_CONTROL_HEARTBEAT,
    APP_FAULT_SENSOR_HEARTBEAT
} AppFaultCode;

/* Fixed diagnostic page rendered by the display task. All fields are
 * plain snapshots; the dashboard never talks back to control. */
typedef struct {
    uint8_t black_bits;
    uint8_t pattern_type;
    int8_t stable_position;
    int8_t candidate_position;
    int16_t left_command;
    int16_t right_command;
    uint8_t safety_state;
    uint8_t recovery_state;
    uint8_t mpu_state;
    uint8_t fault_code;
} AppDiagnostics;

void Dashboard_Render(const AppDiagnostics *data);

#endif
