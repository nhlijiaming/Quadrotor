/*
 * EEPROM.h
 *
 *  Created on: 2014��7��26��
 *      Author: Ljm
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "board.h"

#define Data_Len 30		//length of the data to be stored or read

void EEPROM_Init();

void EEPROM_Write();

void EEPROM_Read();

#endif /* EEPROM_H_ */
