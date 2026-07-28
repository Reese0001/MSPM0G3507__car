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

/* Hard clamp for every wheel command; must never exceed the motor
 * safety layer limit of +/-450. */
#define LINE_LOOKUP_COMMAND_LIMIT (450)

/* Base speed and differential magnitude for positions 0..7; negative
 * positions mirror the same entries. Order: position 0 first. */
#define LINE_LOOKUP_TABLE_ENTRIES                                       \
    {420, 0}, {400, 30}, {370, 65}, {330, 100},                         \
    {285, 135}, {235, 165}, {175, 195}, {120, 220}

/* Yaw rate above which a sharp corner is already turning fast enough
 * that the table differential is reduced to 3/4. */
#define LINE_LOOKUP_HIGH_YAW_DPS (95.0f)

/* Yaw limiting only applies near the edges: |position| >= this. */
#define LINE_LOOKUP_YAW_LIMIT_MIN_POSITION (5)

/* Without a fresh IMU sample, every wheel command is capped here. */
#define LINE_LOOKUP_IMU_DEGRADED_LIMIT (280)

/* SafetyTask latches D1 when a task heartbeat goes silent this long. */
#define APP_CONTROL_HEARTBEAT_TIMEOUT_MS (30U)
#define APP_SENSOR_HEARTBEAT_TIMEOUT_MS (20U)

#endif
