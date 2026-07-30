#include "differential_controller.h"

#include "../../line_tracking/line_tracking_config.h"

#define DIFFERENTIAL_INTEGRAL_LIMIT (300.0f)

static float clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void DifferentialController_Init(DifferentialController *controller)
{
    if (controller != 0) {
        *controller = (DifferentialController){0};
    }
}

void DifferentialController_Update(DifferentialController *controller,
                                   float target_left,
                                   float target_right,
                                   float actual_left,
                                   float actual_right,
                                   float dt_s,
                                   float *output_left,
                                   float *output_right)
{
    float target_average = (target_left + target_right) * 0.5f;
    float target_difference = target_right - target_left;
    float actual_average = (actual_left + actual_right) * 0.5f;
    float actual_difference = actual_right - actual_left;
    float average_error = target_average - actual_average;
    float difference_error = target_difference - actual_difference;
    float average_trim;
    float difference_trim;

    if (controller == 0 || output_left == 0 || output_right == 0) {
        return;
    }
    if (dt_s > 0.0f && dt_s <= 0.2f) {
        controller->average_integral = clamp(
            controller->average_integral + average_error * dt_s,
            -DIFFERENTIAL_INTEGRAL_LIMIT, DIFFERENTIAL_INTEGRAL_LIMIT);
        controller->difference_integral = clamp(
            controller->difference_integral + difference_error * dt_s,
            -DIFFERENTIAL_INTEGRAL_LIMIT, DIFFERENTIAL_INTEGRAL_LIMIT);
    }
    average_trim = DRIVE_SPEED_KP * average_error +
                   DRIVE_SPEED_KI * controller->average_integral;
    difference_trim = DRIVE_DIFF_KP * difference_error +
                      DRIVE_DIFF_KI * controller->difference_integral;
    *output_left = target_left + average_trim - difference_trim;
    *output_right = target_right + average_trim + difference_trim;
}
