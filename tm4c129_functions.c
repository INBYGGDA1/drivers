/*
 * ================================================================
 * File: tm4c129_functons.c
 * Author: Pontus Svensson
 * Date: 2023-10-07
 * Description: Program to help implement various functions.
 *
 * License: This code is distributed under the MIT License. visit
 * https://opensource.org/licenses/MIT for more information.
 * ================================================================
 */

/*================================================================*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
//=============================================================================
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
//=============================================================================
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
//=============================================================================
#include "CF128x128x16_ST7735S.h"
//=============================================================================
#include "grlib/grlib.h"
#include "utils/uartstdio.h"

//=============================================================================
#define MICROPHONE (1 << 0)
#define JOYSTICK (1 << 1)
#define ACCELEROMETER (1 << 2)

//=============================================================================
// The error routine that is called if the driver library
// encounters an error.
//=============================================================================
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
  while (1)
    ;
}
#endif

//=============================================================================
// Configure the UART.
//=============================================================================
void ConfigureUART() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
}

//=============================================================================
// Helper function to reinitialize the ADC to sample on another channel
//=============================================================================
void ADC_newSequence(uint32_t ui32base, uint32_t ui32SequenceNum,
                     uint32_t ui32ADC_Channel, uint32_t ui32Samples) {
  int i;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable interrupts for the specified ADC_BASE
  // and sequence number
  ADCIntEnable(ui32base, ui32SequenceNum);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Clear any interrupts that might occur
  ADCIntClear(ui32base, ui32SequenceNum);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Disable the sequencer before making any changes
  ADCSequenceDisable(ui32base, ui32SequenceNum);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Set the trigger for the sampling sequence to be manual
  ADCSequenceConfigure(ui32base, ui32SequenceNum, ADC_TRIGGER_PROCESSOR, 0);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Configure each step in the sampling process. Here we could add set another
  // channel to sample from in another step.
  for (i = 0; i < ui32Samples; i++) {
    ADCSequenceStepConfigure(ui32base, ui32SequenceNum, i, ui32ADC_Channel);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // At the last step of the sequence, indicate that
  // it is the last and create an interrupt
  ADCSequenceStepConfigure(ui32base, ui32SequenceNum, ui32Samples - 1,
                           ADC_CTL_END | ADC_CTL_IE | ui32ADC_Channel);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Lastly we need to enable the sequence
  ADCSequenceEnable(ui32base, ui32SequenceNum);
}

//=============================================================================
// Helper function to sample the data
//=============================================================================
void sampleData(uint32_t ui32base, uint32_t ui32SequenceNum,
                uint32_t ui32ADC_Channel, uint32_t ui32Samples,
                uint32_t *ui32SamplesRead, void *buffer, uint32_t newSequence) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Should the sequence be reinitialized. If not we just trigger the sequence
  if (newSequence == 1) {
    ADC_newSequence(ui32base, ui32SequenceNum, ui32ADC_Channel, ui32Samples);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Trigger the sampling sequence
  ADCProcessorTrigger(ui32base, ui32SequenceNum);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Wait for the ADC to throw an interrupt indicating that the last sample has
  // been captured
  while (!ADCIntStatus(ui32base, ui32SequenceNum, false)) {
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Place the ADC values into the buffer and return the number of samples
  // collected
  *ui32SamplesRead += ADCSequenceDataGet(ui32base, ui32SequenceNum, buffer);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Clear the interrupt status of the ADC to let other sequences sample their
  // sequences
  ADCIntClear(ui32base, ui32SequenceNum);
}

//=============================================================================
// Helper function to calculate the average of the collected ADC samples
//=============================================================================
void calculate_average(uint32_t ui32samples, uint32_t *buffer,
                       uint32_t *averageReturned) {
  uint32_t averageTemp = 0;
  int i = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Sum all the samples in the array
  for (i = 0; i < ui32samples; i++) {
    averageTemp += buffer[i];
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Then I divide with number of samples in the array to get the average
  *averageReturned = averageTemp / ui32samples;
}

//=============================================================================
// Helper function to enable any sensor
//=============================================================================
void SENSOR_enable(uint32_t enablePeripheral) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ADC Pin Configuration
  //
  // Component    Axis     Pin    GPIO       Channel       AIN
  // --------------------------------------------------------
  // Microphone   N/A      PE5    GPIO_PIN_5  ADC_CTL_CH8  AIN8
  //
  // Joystick     X-Axis   PE4    GPIO_PIN_4  ADC_CTL_CH9  AIN9
  //              Y-Axis   PE3    GPIO_PIN_3  ADC_CTL_CH0  AIN0
  //
  // Gyroscope    X-Axis   PE0    GPIO_PIN_0  ADC_CTL_CH3  AIN3
  //              Y-Axis   PE1    GPIO_PIN_1  ADC_CTL_CH2  AIN2
  //              Z-Axis   PE2    GPIO_PIN_2  ADC_CTL_CH1  AIN1
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  uint32_t gpio_pins_to_enable = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // If the microphone should be enabled set the correct pins
  if (enablePeripheral & MICROPHONE) {
    gpio_pins_to_enable |= GPIO_PIN_5;
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Should the Joystick be enabled set the corresponding pins
  if (enablePeripheral & JOYSTICK) {
    gpio_pins_to_enable |= GPIO_PIN_3 | GPIO_PIN_4;
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Should the accelerometer be used set the corresponding pins
  if (enablePeripheral & ACCELEROMETER) {
    gpio_pins_to_enable |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable all pins that should be set as ADC pins
  GPIOPinTypeADC(GPIO_PORTE_BASE, gpio_pins_to_enable);
}

//=============================================================================
// Helper function to initialize any ADC module
//=============================================================================
void ADC_init(uint32_t ui32Base) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable the ADC module
  SysCtlPeripheralEnable(ui32Base);
  while (!SysCtlPeripheralReady(ui32Base)) {
  }
}

//=============================================================================
// Helper function to initalize any GPIO_PORTx_BASE
//=============================================================================
void GPIOPort_init(uint32_t ui32GPIOx) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable the specified GPIO_PORTx_BASE
  SysCtlPeripheralEnable(ui32GPIOx);
  while (!SysCtlPeripheralReady(ui32GPIOx)) {
  }
}

//=============================================================================
// Helper function to calculate absolute difference for uint32_t
//=============================================================================
uint32_t abs_diff(uint32_t a, uint32_t b) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Since the variables are defined as unsigned integers, I cant subtract a - b
  // or vice versa since that could lead to a overflow issue. In this function i
  // check if a > b, then return a - b, else if b > a I return b - a. In order
  // to avoid taking the difference of a smaller unsigned integer with a greater
  // one.
  return (a > b) ? (a - b) : (b - a);
}

//=============================================================================
// Helper function to clear the text
//=============================================================================
void clear_text(tContext *sContext, int x, int y, int width, int height) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // When updating the text on the screen, the values could have changed to be
  // << than the previous value and thus will not draw over the text furthest to
  // the right on the LCD. To circumvent this I draw a white rectangle across
  // the LCD screen with the height of the current fontSize.
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  tRectangle rect = {x, y, x + width, y + height};
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Since the background is white, I change the rectangle to white and fills
  // the are to avoid any overlap when drawing.
  GrContextForegroundSet(sContext, ClrWhite);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Draw the rectangle in the specified coordinates
  GrRectFill(sContext, &rect);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Set the foreground back to black since I now am going to draw the text.
  GrContextForegroundSet(sContext, ClrBlack);
}
