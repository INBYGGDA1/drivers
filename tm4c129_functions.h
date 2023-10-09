#ifndef HELPER_H
#define HELPER_H
#include <stdint.h>
#include <sys/types.h>
#include "grlib/grlib.h"
#define MICROPHONE (1 << 0)
#define JOYSTICK (1 << 1)
#define ACCELEROMETER (1 << 2)
#define BUZZER (1 << 3)
#define BUTTON_UP (1 << 4)
#define BUTTON_DOWN (1 << 5)

#define GPIO_PORTE_SENSORS (1 << 0)
#define GPIO_PORTF_SENSORS (1 << 1)
#define GPIO_PORTL_SENSORS (1 << 2)

// Linked list used in snake
struct node {
  uint32_t data;
  struct node *next;
};

extern void ConfigureUART();
extern void ConfigureSystemClock(uint32_t frequency, uint32_t *systemClock);
extern void UARTClearScreen();
extern void ADC_newSequence(uint32_t ui32base, uint32_t ui32SequenceNum,
                            uint32_t ui32ADC_Channel, uint32_t ui32Samples);
extern void sampleData(uint32_t ui32base, uint32_t ui32SequenceNum,
                       uint32_t ui32ADC_Channel, uint32_t ui32Samples,
                       uint32_t *ui32SamplesRead, void *buffer,
                       uint32_t newSequence);
extern void calculate_average(uint32_t ui32samples, uint32_t *buffer,
                              uint32_t *averageReturned);
extern void SENSOR_enable(uint32_t enablePeripheral);
extern void PERIPH_init(uint32_t ui32Base);
extern uint32_t abs_diff(uint32_t a, uint32_t b);
extern void clear_text(tContext *sContext, int x, int y, int width, int height);
#endif // !__HELPER__
