#ifndef LINE_FOLLOWING_PROFILE_H
#define LINE_FOLLOWING_PROFILE_H

/*
 * Current burn profile: eight-channel line sensor, MPU6050 and two motors.
 * Change these switches when the corresponding modules are physically fitted.
 */
#define LINE_FOLLOWING_POWER_QUALIFIED (1)
#define LINE_FOLLOWING_USE_ULTRASONIC (0)
#define LINE_FOLLOWING_USE_IMU (1)
#define LINE_FOLLOWING_REQUIRE_IMU (0)
#define LINE_FOLLOWING_IMU_STARTUP_TIMEOUT_MS (2600U)
#define LINE_FOLLOWING_IMU_DEGRADED_LIMIT (180)
#define LINE_FOLLOWING_USE_VISION (0)
#define LINE_FOLLOWING_USE_LEGACY_ODOMETRY (0)

#endif
