#include "key.h"
#include "timer.h"

static Key_t key1 = {
    .GPIOx = KEY_PORT,
    .GPIO_Pin = KEY_K1_PIN,
    .state = KEY_STATE_RELEASED,
    .pressTime = 0,
    .debounceTime = 0
};

#define DEBOUNCE_DELAY 20

KeyEvent Key_Scan(Key_t* key, uint32_t currentTime, uint32_t longPressThreshold)
{
    uint8_t isPressed = 0;

    if (DL_GPIO_readPins(key->GPIOx, key->GPIO_Pin) == 0) {
        isPressed = 1;
    }

    switch (key->state) {
        case KEY_STATE_RELEASED:
            if (isPressed) {
                key->state = KEY_STATE_DEBOUNCE;
                key->debounceTime = currentTime;
            }
            break;
        case KEY_STATE_DEBOUNCE:
            if (currentTime - key->debounceTime >= DEBOUNCE_DELAY) {
                if (isPressed) {
                    key->state = KEY_STATE_PRESSED;
                    key->pressTime = currentTime;
                } else {
                    key->state = KEY_STATE_RELEASED;
                }
            }
            break;
        case KEY_STATE_PRESSED:
            if (!isPressed) {
                key->state = KEY_STATE_RELEASED;
                return KEY_EVENT_SHORT;
            }
            if (currentTime - key->pressTime >= longPressThreshold) {
                key->state = KEY_STATE_LONG;
                return KEY_EVENT_LONG;
            }
            break;
        case KEY_STATE_LONG:
            if (!isPressed) {
                key->state = KEY_STATE_RELEASED;
            }
            break;
    }

    return KEY_EVENT_NONE;
}

KeyEvent Key_PollEvent(void)
{
    static uint32_t lastTick = 0;
    uint32_t currentTick = Get_Time();

    if (currentTick - lastTick >= 10) {
        lastTick = currentTick;
        return Key_Scan(&key1, currentTick, 700);
    }

    return KEY_EVENT_NONE;
}
