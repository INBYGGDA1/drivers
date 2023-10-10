#ifndef tm4c129_LCD_H
#define tm4c129_LCD_H
#include "CF128x128x16_ST7735S.h"

// Linked list used in snake
struct node {
  uint32_t data;
  struct node *next;
};

void clear_text(tContext *sContext, int xCoordinate, int yCoordinate,
                int widthX, int heightY);

#endif // !tm4c129_LCD_H
