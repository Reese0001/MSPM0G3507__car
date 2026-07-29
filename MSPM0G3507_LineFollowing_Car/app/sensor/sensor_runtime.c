#include "sensor_runtime.h"

#include "../line/line_motion.h"
#include "../mailbox/app_mailbox.h"
#include "../../modules/line_tracking/decoder/line_position.h"
#include "../../modules/line_tracking/scanner/line_scanner.h"

static uint16_t sequence;

void SensorRuntime_Init(void)
{
    sequence = 0U;
    LineScanner_Init();
}

bool SensorRuntime_Step(uint32_t now_ms)
{
    LineSensorSnapshot snapshot;
    bool frame_ready = false;

    if (LineScanner_ReadFrame(now_ms, &snapshot)) {
        AppLineSample sample;

        sample.position = LinePosition_Update(snapshot.black_bits);
        sequence++;
        if (sequence == 0U) {
            sequence = 1U;
        }
        sample.sequence = sequence;
        sample.timestamp_ms = now_ms;
        AppMailbox_PublishLineSample(&sample);
        frame_ready = true;
    }

    AppLineMotion_ServiceImu(now_ms);
    return frame_ready;
}
