#ifndef HELPER_H
#define HELPER_H
#include <stdint.h>
#include "grlib/grlib.h"
extern void ConfigureUART();
extern void ADC_newSequence(uint32_t ui32base, uint32_t ui32SequenceNum,
                            uint32_t ui32ADC_Channel, uint32_t ui32Samples);
extern void sampleData(uint32_t ui32base, uint32_t ui32SequenceNum,
                       uint32_t ui32ADC_Channel, uint32_t ui32Samples,
                       uint32_t *ui32SamplesRead, void *buffer,
                       uint32_t newSequence);
extern void calculate_average(uint32_t ui32samples, uint32_t *buffer,
                              uint32_t *averageReturned);
extern void SENSOR_enable(uint32_t enablePeripheral);
extern void ADC_init(uint32_t ui32Base);
extern void GPIOPort_init(uint32_t ui32GPIOx);
extern uint32_t abs_diff(uint32_t a, uint32_t b);
extern void clear_text(tContext *sContext, int x, int y, int width, int height);
#endif // !__HELPER__
