/*
 * CF128x128x16_ST7735S.h
 *
 *  Created on: 6 Sep 2021
 *      Author: User
 */

#ifndef DRIVERS_CF128X128X16_ST7735S_H_
#define DRIVERS_CF128X128X16_ST7735S_H_

#ifndef DRIVERS_SYSTICKDELAY_H_
#define DRIVERS_SYSTICKDELAY_H_

#include <stdint.h>

// notes:
// ticks/second (tps) table for a 32 bit unsigned integer
// 1000tps      (1ms resolution)    -> 50 days to rollover
// 10000tps     (100µs resolution)  -> 5 days to rollover
// 100000tps    (10µs resolution)   -> 12 hours to rollover
// 1000000tps   (1µs resolution)    -> bit over an hour to rollover
// 10000000tps  (100ns resolution)  -> 7 minutes
// 100000000tps (10ns resolution)   -> 43 seconds, and an unusable cpu
// At this point the micro would be trying to handle an interrupt almost every clock cycle.


void SysTickDelayMs(uint32_t milliseconds);

uint32_t SysTickGetTicks(void);

void InitSysTickDelay(uint32_t coreSystemClock, uint32_t ticksPerSecond);

void SysTickDelayStop(void);
void SysTickDelayStart(void);

#endif /* DRIVERS_SYSTICKDELAY_H_ */

#ifdef __cplusplus
extern "C"
{
#endif

#include "grlib/grlib.h"

typedef enum _LCD_Orientation
{
    LCD_ORIENTATION_UP      = 0,
    LCD_ORIENTATION_LEFT    = 1,
    LCD_ORIENTATION_DOWN    = 2,
    LCD_ORIENTATION_RIGHT   = 3
}LCD_Orientation;

extern const tDisplay g_sCF128x128x16_ST7735S;

void CF128x128x16_ST7735SInit(uint32_t sysClock);
void CF128x128x16_ST7735SSetOrientation(LCD_Orientation orientation);
void CF128x128x16_ST7735SClear(uint32_t colour);


#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_CF128X128X16_ST7735S_H_ */
