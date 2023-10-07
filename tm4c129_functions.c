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
#include "drivers/buttons.h"
#include "inc/hw_memmap.h"
//=============================================================================
#include "CF128x128x16_ST7735S.h"
//=============================================================================
#include "grlib/grlib.h"
#include "utils/uartstdio.h"
#include "tm4c129_functions.h"

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
// clang-format off
/*
 * Function: ConfigureUART
 * ------------------------------
 *  Autmatically configure the UART using the TivWare peripheral driver library 
 *
 *  Settings:
 *    Baud-Rate: 115200
 *    Clock freq: 16000000
 *
 *  returns:
 *    N/A
 */
// clang-format on
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
// clang-format off
/*
 * Function: ADC_newSequence
 * ---------------------------
 * Reinitializes the ADC to sample from a different channel.
 *
 * parameters:
 *   ui32base - The ADC base address, defined in the TivaWare peripheral library.
 *   ui32SequenceNum - Sequence number for ADC sampling. Valid values are 0, 1, 2, 3.
 *                     Refer to datasheet p.1180 for details.
 *   ui32ADC_Channel - The analog input channel to sample from.
 *                     Refer to datasheet p.1179 for available channels.
 *   ui32Samples - Number of samples to read in the sequence.
 *                 Value is dependent on SequenceNum. See datasheet p.1180 for specifics.
 *
 * returns:
 *   None. The function modifies hardware registers to change ADC settings.
 */
// clang-format on
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
// clang-format off
/*
 * Function: sampleData
 * ---------------------------
 *  Trigger the sampling sequence for the specified sequence
 *
 *  parameters:
 *    ui32base - The ADC base address
 *    ui32SequenceNum - Sequence number for ADC sampling. Valid values are 0, 1, 2, 3. 
 *                      Refer to datasheet p.1180 for details.
 *    ui32ADC_Channel - The analog input channel to sample from.
 *                      Refer to datasheet p.1179 for available channels
 *    ui32samples - Number of samples to initialize the sequence to.
 *                  The values is dependent on SequenceNum.
 *                  Refer to datasheet p.1180 for details.
 *    *ui32SamplesRead - Assigns the samples read after the sampling sequence has been triggered.
 *    *buffer - Pointer to the location where the ADC will store the values samples.
 *    newSequence - Is a flag to indicate if a new sequence should be configured before sampling.
 *
 *    returns:
 *      N/A
 */
// clang-format on
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
// clang-format off
/*
 * Function: calculate_average
 * ---------------------------------------
 *  Helper function to calculate the average of the collected ADC samples
 *  parameters:
 *    ui32samples - The number of samples in the buffer.
 *    *buffer - pointer to the array storing the samples.
 *    *averageReturned - Pointer to store the average value of the buffer
 *
 *  returns:
 *    N/A
 */
// clang-format on
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
// clang-format off
/*
 * Function: SENSOR_enable
 * ---------------------------
 *  Helper function to enable any sensor on the MKII 
 *
 *  parameters:
 *    enablePeripheral - Valid inputs: MICROPHONE | JOYSTICK | ACCELEROMETER | BUZZER | BUTTON_UP 
 *                        | BUTTON_DOWN
 *   
 *  returns:
 *  N/A
 */
// clang-format on
//=============================================================================
void SENSOR_enable(uint32_t enablePeripheral) {
  // Pin Config MKII
  // Comp  Axis  Pin  GPIO  Port  Ch    AIN
  // ---------------------------------------
  // Mic   N/A   PE5  P5    PE    CH8   A8
  // Joy   X     PE4  P4    PE    CH9   A9
  //       Y     PE3  P3    PE    CH0   A0
  // Gyro  X     PE0  P0    PE    CH3   A3
  //       Y     PE1  P1    PE    CH2   A2
  //       Z     PE2  P2    PE    CH1   A1
  // Buz   N/A   PF1  P1    PF    N/A   N/A
  // Btn1  N/A   PL1  P1    PL    N/A   N/A
  // Btn2  N/A   PL2  P2    PL    N/A   N/A
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  uint32_t gpio_porte_pins_to_enable = 0;
  uint32_t gpio_portf_pins_to_enable = 0;
  uint32_t gpio_portl_pins_to_enable = 0;
  uint32_t gpio_ports_to_enable = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // If the microphone should be enabled set the correct pins
  if (enablePeripheral & MICROPHONE) {
    gpio_porte_pins_to_enable |= GPIO_PIN_5;
    gpio_ports_to_enable |= GPIO_PORTE_BASE;
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Should the Joystick be enabled set the corresponding pins
  if (enablePeripheral & JOYSTICK) {
    gpio_porte_pins_to_enable |= GPIO_PIN_3 | GPIO_PIN_4;
    gpio_ports_to_enable |= GPIO_PORTE_BASE;
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Should the accelerometer be used set the corresponding pins
  if (enablePeripheral & ACCELEROMETER) {
    gpio_porte_pins_to_enable |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    gpio_ports_to_enable |= GPIO_PORTE_BASE;
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (enablePeripheral & BUZZER) {
    gpio_ports_to_enable |= GPIO_PORTF_BASE;
    gpio_portf_pins_to_enable |= GPIO_PIN_1;
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (enablePeripheral & BUTTON_UP) {
    gpio_ports_to_enable |= GPIO_PORTL_BASE;
    gpio_portl_pins_to_enable |= GPIO_PIN_1;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Set the buttons to act as input
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (enablePeripheral & BUTTON_DOWN) {
    gpio_ports_to_enable |= GPIO_PORTL_BASE;
    gpio_portl_pins_to_enable |= GPIO_PIN_2;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Set the buttons to act as input
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Check which ports should be enabled
  if (gpio_ports_to_enable & GPIO_PORTE_BASE) {

    GPIOPort_init(SYSCTL_PERIPH_GPIOE);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Enable all pins that should be set as ADC pins
    GPIOPinTypeADC(GPIO_PORTE_BASE, gpio_porte_pins_to_enable);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (gpio_ports_to_enable & GPIO_PORTF_BASE) {
    GPIOPort_init(SYSCTL_PERIPH_GPIOF);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Specify the buzzer to act as output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (gpio_ports_to_enable & GPIO_PORTL_BASE) {

    GPIOPort_init(SYSCTL_PERIPH_GPIOL);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Set the buttons to act as input
    GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, gpio_portl_pins_to_enable);
  }
}

//=============================================================================
// clang-format off
/*
 * Function: ADC_init
 * -------------------------
 *  Helper function to initialize any ADC module
 *
 *  parameters:
 *    ui32base - The ADC base address
 *
 *  returns:
 *    N/A
 */
// clang-format on
//=============================================================================
void ADC_init(uint32_t ui32Base) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable the ADC module
  SysCtlPeripheralEnable(ui32Base);
  while (!SysCtlPeripheralReady(ui32Base)) {
  }
}

//=============================================================================
// clang-format off
/*
 * Function: GPIOPort_init
 * ---------------------------------
 *  Helper function to initialize any GPIO_PORTx_BASE
 *
 *  parameters:
 *    ui32GPIOx - The GPIO_PORTx_BASE to be enabled
 *
 *  returns:
 *    N/A
 */
// clang-format on
//=============================================================================
void GPIOPort_init(uint32_t ui32GPIOx) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable the specified GPIO_PORTx_BASE
  SysCtlPeripheralEnable(ui32GPIOx);
  while (!SysCtlPeripheralReady(ui32GPIOx)) {
  }
}

//=============================================================================
// clang-format off
/*
 * Function: abs_diff
 * --------------------------------
 *  Helper function to calculate the absolute differecen for an unsigned integer 
 *  uint32_t.
 *  parameters:
 *    a - unsigned integer
 *    b - unsigned integer
 *
 *  returns:
 *    The absolute difference between a & b.
 */
// clang-format on
//=============================================================================
uint32_t abs_diff(uint32_t a, uint32_t b) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Since the variables are defined as unsigned integers, I cant subtract a - b
  // or vice versa since that could lead to a underflow issue. In this function
  // i check if a > b, then return a - b, else if b > a I return b - a. In order
  // to avoid taking the difference of a smaller unsigned integer with a greater
  // one.
  return (a > b) ? (a - b) : (b - a);
}

//=============================================================================
// clang-format off
/*
 * Function: clear_text
 * -------------------------------
 *  Helper function to clear the text on the LCD
 *
 *  parameters:
 *    *sContext - A pointer to context currently being used.
 *    x - The x coordinate on the LCD screen.
 *    y - The y coordinate on the LCD screen.
 *    width - The width of the rectagle to be filled.
 *    height - The heigh of the rectangle to be filled.
 */
// clang-format on
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
