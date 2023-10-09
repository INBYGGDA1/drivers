#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

//=============================================================================
void vWorker(uint32_t runTime) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TickType_t tickIN = xTaskGetTickCount();
  TickType_t xRunTime = pdMS_TO_TICKS(runTime);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  while (1) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    TickType_t currentTickTime = xTaskGetTickCount();
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Keep working until the specified amount of systemticks
    // has occured
    if (currentTickTime - tickIN >= xRunTime) {
      break;
    }
  }
}
