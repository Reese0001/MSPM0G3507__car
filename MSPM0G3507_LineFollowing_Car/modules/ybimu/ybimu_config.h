#ifndef YBIMU_CONFIG_H
#define YBIMU_CONFIG_H

#define YBIMU_SAMPLE_PERIOD_MS          (10U)
#define YBIMU_STALE_TIMEOUT_MS          (50U)
#define YBIMU_MAX_CONSECUTIVE_ERRORS    (3U)

#define YBIMU_GYRO_SCALE_RAD_S \
    ((2000.0f / 32767.0f) * (3.1415926f / 180.0f))
#define YBIMU_MAG_SCALE_UT              (800.0f / 32767.0f)
#define YBIMU_RAD_TO_DEG                 (57.2957795f)

/* Identity mapping until the installed module orientation is bench verified. */
#define YBIMU_BODY_X_SOURCE             (0U)
#define YBIMU_BODY_Y_SOURCE             (1U)
#define YBIMU_BODY_Z_SOURCE             (2U)
#define YBIMU_BODY_X_SIGN               (1.0f)
#define YBIMU_BODY_Y_SIGN               (1.0f)
#define YBIMU_BODY_Z_SIGN               (1.0f)

#endif
