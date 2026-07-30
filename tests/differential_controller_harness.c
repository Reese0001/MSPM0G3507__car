#include <assert.h>

#include "modules/motor/feedback/differential_controller.h"

int main(void)
{
    DifferentialController controller;
    float left;
    float right;

    DifferentialController_Init(&controller);
    DifferentialController_Update(&controller, 300.0f, 300.0f,
                                  200.0f, 260.0f, 0.01f, &left, &right);
    assert(left > 300.0f);
    assert(left > right);
    return 0;
}
