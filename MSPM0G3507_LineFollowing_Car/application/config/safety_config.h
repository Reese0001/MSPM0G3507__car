#ifndef SAFETY_CONFIG_H
#define SAFETY_CONFIG_H

#define MOTION_REQUEST_MAX_AGE_MS (50U)
#define SAFETY_ULTRASONIC_STALE_MS (120U)
#define SAFETY_IMU_STALE_MS (50U)
#define SAFETY_VISION_STALE_MS (300U)

#define SAFETY_OBSTACLE_STOP_MM (200U)
#define SAFETY_OBSTACLE_LIMIT_MM (350U)
#define SAFETY_OBSTACLE_CLEAR_MM (400U)
#define SAFETY_CLEAR_SAMPLE_COUNT (5U)

#define SAFETY_RUNNING_SPEED_LIMIT (450)
#define SAFETY_LIMITED_SPEED_LIMIT (180)

/* Motor UART frames leave at most once per period; an immediate zero
 * (stop or unapproved) command bypasses the limit. */
#define MOTOR_UART_MIN_PERIOD_MS (5U)

#endif
