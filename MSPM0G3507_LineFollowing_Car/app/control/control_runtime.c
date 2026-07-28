#include "control_runtime.h"

#include "../line/line_motion.h"
#include "../mailbox/app_mailbox.h"

static uint16_t last_sequence;

void ControlRuntime_Init(void)
{
    last_sequence = 0U;
}

bool ControlRuntime_RunOnce(uint32_t now_ms)
{
    AppLineSample sample;
    MotionRequest request;

    if (!AppMailbox_ReadLineSample(&sample) ||
        sample.sequence == last_sequence) {
        return false;
    }
    last_sequence = sample.sequence;
    if (!AppLineMotion_BuildRequest(&sample, now_ms, &request)) {
        return false;
    }
    AppMailbox_PublishMotionRequest(&request);
    return true;
}
