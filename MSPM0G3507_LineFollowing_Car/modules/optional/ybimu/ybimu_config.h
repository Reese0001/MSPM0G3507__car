#ifndef YBIMU_CONFIG_H
#define YBIMU_CONFIG_H

#define YBIMU_SAMPLE_PERIOD_MS          (10U)
#define YBIMU_STALE_TIMEOUT_MS          (50U)
#define YBIMU_GYRO_ONLY                 (1)
#define YBIMU_MAX_CONSECUTIVE_ERRORS    (3U)
#define YBIMU_CAL_POLL_PERIOD_MS         (100U)
#define YBIMU_CAL_IMU_TIMEOUT_MS         (7000U)
#define YBIMU_CAL_MAG_TIMEOUT_MS         (60000U)

#define YBIMU_GYRO_SCALE_RAD_S \
    ((2000.0f / 32767.0f) * (3.1415926f / 180.0f))
#define YBIMU_MAG_SCALE_UT              (800.0f / 32767.0f)
#define YBIMU_RAD_TO_DEG                 (57.2957795f)

#define YBIMU_MAG_MIN_UT                 (20.0f)
#define YBIMU_MAG_MAX_UT                 (100.0f)
#define YBIMU_MAG_NORM_SQ_DELTA_MAX      (900.0f)

/* Identity mapping until the installed module orientation is bench verified. */
#define YBIMU_BODY_X_SOURCE             (0U)
#define YBIMU_BODY_Y_SOURCE             (1U)
#define YBIMU_BODY_Z_SOURCE             (2U)
#define YBIMU_BODY_X_SIGN               (1.0f)
#define YBIMU_BODY_Y_SIGN               (1.0f)
#define YBIMU_BODY_Z_SIGN               (1.0f)

#endif
