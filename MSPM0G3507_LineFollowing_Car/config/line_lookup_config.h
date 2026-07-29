#ifndef LINE_LOOKUP_CONFIG_H
#define LINE_LOOKUP_CONFIG_H

/*
 * Constants for the 15-position lookup line controller.
 * Task 2 (line_position): decode and debounce settings.
 * Task 3 (line_lookup_control): speed/differential table entries.
 */

/* A position jump larger than this needs debouncing before acceptance. */
#define LINE_POSITION_ADJACENT_STEP (1)

/* Consecutive identical candidate frames required to accept a jump. */
#define LINE_POSITION_JUMP_ACCEPT_FRAMES (2U)

/* Hard clamp for the lookup feedforward command. The downstream motor
 * safety layer remains responsible for its independent 450 limit. */
#define LINE_LOOKUP_COMMAND_LIMIT (140)

/* Base speed and differential magnitude for positions 0..7; negative
 * positions mirror the same entries. Order: position 0 first. */
#define LINE_LOOKUP_TABLE_ENTRIES                                       \
    {140, 0}, {140, 0}, {130, 12}, {115, 28},                           \
    {95, 45}, {80, 60}, {70, 65}, {60, 60}

/* YbImu 仅抑制转向角速度；数据无效时修正量必须为零。 */
#define LINE_YAW_RATE_DEADBAND_DPS (2.0f)
#define LINE_YAW_DAMPING_GAIN      (0.18f)
#define LINE_YAW_DAMPING_LIMIT     (24)
#define LINE_NOISE_HOLD_MS         (20U)

/* SafetyTask latches D1 when a task heartbeat goes silent this long. */
#define APP_CONTROL_HEARTBEAT_TIMEOUT_MS (30U)
#define APP_SENSOR_HEARTBEAT_TIMEOUT_MS (20U)

#endif
