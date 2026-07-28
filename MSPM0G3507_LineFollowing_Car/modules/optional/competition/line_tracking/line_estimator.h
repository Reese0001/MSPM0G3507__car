#ifndef LINE_ESTIMATOR_H
#define LINE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "../../../line_tracking/line_model.h"
#include "line_features.h"

void LineEstimator_Init(void);
bool LineEstimator_Update(const LineFeatures *features, uint32_t now_ms);
bool LineEstimator_Get(LineEstimate *out);

#endif
