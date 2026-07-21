#ifndef CAR_SENSOR_EVENTS_H
#define CAR_SENSOR_EVENTS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAR_LINE_NONE = 0,
    CAR_LINE_CENTER,
    CAR_LINE_LEFT_EDGE,
    CAR_LINE_RIGHT_EDGE,
    CAR_LINE_CROSS,
    CAR_LINE_STOP_MARKER
} CarLineEvent;

typedef struct {
    uint8_t sensor_bits;
    float weighted_error;
    bool line_valid;
    CarLineEvent event;
} CarSensorFrame;

bool CarSensor_ReadFrame(CarSensorFrame *frame);
CarLineEvent CarSensor_Classify(const CarSensorFrame *frame);

#endif
