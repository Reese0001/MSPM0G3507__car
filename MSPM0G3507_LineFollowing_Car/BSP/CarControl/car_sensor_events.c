#include "car_sensor_events.h"

#include <stddef.h>

#include "../Eight_Tracking/app_irtracking.h"

#define CAR_SENSOR_EDGE_ERROR_LIMIT       (3.0f)
#define CAR_SENSOR_CROSS_MIN_ACTIVE       (4U)
#define CAR_SENSOR_ENDPOINTS_MASK         (0x81U)

static uint8_t CarSensor_CountBlackChannels(uint8_t sensor_bits)
{
    uint8_t count = 0U;

    for (uint8_t channel = 0U; channel < 8U; channel++) {
        if ((sensor_bits & (uint8_t)(1U << channel)) != 0U) {
            count++;
        }
    }

    return count;
}

bool CarSensor_ReadFrame(CarSensorFrame *frame)
{
    uint8_t x1;
    uint8_t x2;
    uint8_t x3;
    uint8_t x4;
    uint8_t x5;
    uint8_t x6;
    uint8_t x7;
    uint8_t x8;

    if (frame == NULL) {
        return false;
    }

    Gray_ReadAll(&x1, &x2, &x3, &x4, &x5, &x6, &x7, &x8);
    frame->sensor_bits = Tracking_PackBlackSensors(x1, x2, x3, x4,
                                                    x5, x6, x7, x8);
    frame->line_valid = Tracking_ComputeWeightedError(frame->sensor_bits,
                                                       &frame->weighted_error);
    frame->event = CarSensor_Classify(frame);

    return true;
}

CarLineEvent CarSensor_Classify(const CarSensorFrame *frame)
{
    uint8_t active_count;

    if (frame == NULL || !frame->line_valid) {
        return CAR_LINE_NONE;
    }

    if (frame->sensor_bits == 0xFFU) {
        return CAR_LINE_STOP_MARKER;
    }

    active_count = CarSensor_CountBlackChannels(frame->sensor_bits);
    if (((frame->sensor_bits & CAR_SENSOR_ENDPOINTS_MASK) ==
         CAR_SENSOR_ENDPOINTS_MASK) ||
        (active_count >= CAR_SENSOR_CROSS_MIN_ACTIVE)) {
        return CAR_LINE_CROSS;
    }

    if (frame->weighted_error <= -CAR_SENSOR_EDGE_ERROR_LIMIT) {
        return CAR_LINE_LEFT_EDGE;
    }

    if (frame->weighted_error >= CAR_SENSOR_EDGE_ERROR_LIMIT) {
        return CAR_LINE_RIGHT_EDGE;
    }

    return CAR_LINE_CENTER;
}
