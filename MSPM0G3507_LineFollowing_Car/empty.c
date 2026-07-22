#include "ti_msp_dl_config.h"
#include "application/app_main.h"

int main(void)
{
    SYSCFG_DL_init();
    App_Main_Init();

    while (1)
    {
        App_Main_RunOnce();
    }
}
