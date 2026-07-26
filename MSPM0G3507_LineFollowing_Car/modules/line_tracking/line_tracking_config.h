#ifndef LINE_TRACKING_CONFIG_H
#define LINE_TRACKING_CONFIG_H

/* Official AD0/AD1/AD2/OUT example waits 50 us after changing address. */
#define LINE_MUX_SETTLE_US (50U)
/* Official line-following example defines ACTIVE_LEVEL=1 for a black line. */
#define LINE_SENSOR_BLACK_ACTIVE_LEVEL (1U)
#define LINE_SENSOR_STALE_MS (20U)
/* A blocking eight-channel frame scan must finish inside this budget. */
#define LINE_SCAN_FRAME_BUDGET_US (600U)
#define LINE_PREDICTION_HORIZON_S (0.020f)

#endif
