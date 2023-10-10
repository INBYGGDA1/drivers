/*================================================================*/
#include <stdbool.h>
//=============================================================================
#include "CF128x128x16_ST7735S.h"

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
void clear_text(tContext *sContext, int xCoordinate, int yCoordinate,
                int widthX, int heightY) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // When updating the text on the screen, the values could have changed to be
  // << than the previous value and thus will not draw over the text furthest to
  // the right on the LCD. To circumvent this I draw a white rectangle across
  // the LCD screen with the height of the current fontSize.
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  tRectangle rect = {xCoordinate, yCoordinate, xCoordinate + widthX,
                     yCoordinate + heightY};
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Since the background is white, change the rectangle to white and fill
  // the are to avoid any overlap when drawing.
  GrContextForegroundSet(sContext, ClrWhite);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Draw the rectangle in the specified coordinates
  GrRectFill(sContext, &rect);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Set the foreground back to black.
  GrContextForegroundSet(sContext, ClrBlack);
}
