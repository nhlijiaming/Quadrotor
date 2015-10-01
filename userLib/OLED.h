//---------------------------------------------------------
/*	file name:fun.h
 *  describe :declaration myself function
 *  created time: 8ÔÂ5ÈÕ
 *
 */
#ifndef __OLED_H_
#define __OLED_H_

#include "board.h"

typedef   char    byte;

#define X_WIDTH 128
#define Y_WIDTH 64

#define  OLED_D0_L      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0)
#define  OLED_D0_H      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3)

#define  OLED_D1_L      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0)
#define  OLED_D1_H      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2)

#define  OLED_RST_L      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0)
#define  OLED_RST_H      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1)

#define  OLED_DS_L      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0)
#define  OLED_DS_H      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0)

void OLED_WrDat(unsigned char data);

void OLED_WrCmd(unsigned char cmd);

void OLED_Set_Pos(byte x, byte y);

void OLED_Fill(byte bmp_data);

void OLED_Clear(void);

void OLED_Init(void);

void OLED_PutPixel(byte x,byte y);

void OLED_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);

void OLED_P8x16Str(byte x,byte y,byte ch[]);

void OLED_P14x16Str(byte x,byte y,byte ch[]);

void OLED_Print(byte x, byte y, byte ch[]);

#endif
