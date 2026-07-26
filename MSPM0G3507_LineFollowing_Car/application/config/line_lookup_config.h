#ifndef LINE_LOOKUP_CONFIG_H
#define LINE_LOOKUP_CONFIG_H

/*
 * Constants for the 15-position lookup line controller.
 * Task 2 (line_position): decode and debounce settings.
 * Task 3 (line_lookup_control) will extend this file with the
 * speed/differential table entries.
 */

/* A position jump larger than this needs debouncing before acceptance. */
#define LINE_POSITION_ADJACENT_STEP (1)

/* Consecutive identical candidate frames required to accept a jump. */
#define LINE_POSITION_JUMP_ACCEPT_FRAMES (2U)

#endif
