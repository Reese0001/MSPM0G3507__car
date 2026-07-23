#ifndef LINE_TRACKING_CONFIG_H
#define LINE_TRACKING_CONFIG_H

/* Initial oscilloscope-tuning value; keep in one place for bench adjustment. */
#define LINE_MUX_SETTLE_US (10U)
#define LINE_SENSOR_STALE_MS (20U)
#define LINE_PREDICTION_HORIZON_S (0.020f)

#endif
