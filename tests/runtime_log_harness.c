#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "modules/display/runtime_log.h"

static char drawn[8][22];

void Ssd1306_DrawText(uint8_t page, uint8_t column, const char *text)
{
    (void)column;
    if (page < 8U) {
        (void)snprintf(drawn[page], sizeof(drawn[page]), "%s", text);
    }
}

bool Ssd1306_FlushDirty(void)
{
    return true;
}

int main(void)
{
    char snapshot[RUNTIME_LOG_CAPACITY][RUNTIME_LOG_LINE_BUFFER] = {{0}};
    uint8_t count;
    uint8_t index;

    RuntimeLog_Init();
    for (index = 0U; index < 8U; ++index) {
        char event[4];
        (void)snprintf(event, sizeof(event), "E%u", (unsigned int)index);
        assert(RuntimeLog_Push(index, event));
    }
    assert(RuntimeLog_Push(0U, "BOOT"));

    count = RuntimeLog_Snapshot(snapshot);
    assert(count == RUNTIME_LOG_CAPACITY);
    assert(strcmp(snapshot[0], "0001 E1") == 0);
    assert(strcmp(snapshot[6], "0007 E7") == 0);
    assert(strcmp(snapshot[7], "0000 BOOT") == 0);
    for (index = 0U; index < RUNTIME_LOG_CAPACITY; ++index) {
        puts(snapshot[index]);
    }

    RuntimeLog_Init();
    assert(RuntimeLog_Push(10U, "CFG OK"));
    assert(!RuntimeLog_Push(11U, "CFG OK"));
    assert(RuntimeLog_Snapshot(snapshot) == 1U);
    assert(strcmp(snapshot[0], "0010 CFG OK") == 0);
    puts("DEDUP_OK");

    RuntimeLog_Init();
    assert(RuntimeLog_Push(12U, "123456789012345678"));
    assert(!RuntimeLog_Push(13U, "123456789012345678"));
    puts("LONG_DEDUP_OK");

    RuntimeLog_Init();
    assert(RuntimeLog_PushMotor(0U, -1000, 1000));
    assert(RuntimeLog_Snapshot(snapshot) == 1U);
    assert(strcmp(snapshot[0], "0000 TX L-999 R999") == 0);
    RuntimeLog_Draw();
    assert(strcmp(drawn[0], "0000 TX L-999 R999") == 0);
    return 0;
}
