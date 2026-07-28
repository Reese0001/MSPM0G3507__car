/* Keep the diagnostic symbol renames identical in CLI and CCS builds. */
#define SVC_Handler FreeRTOS_SVC_Handler
#define vRestoreContextOfFirstTask FreeRTOS_vRestoreContextOfFirstTask
#include "portasm.c"
