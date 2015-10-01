/*
 * Debugger.h
 *
 *  Created on: 2014Äê7ÔÂ23ÈÕ
 *      Author: Ljm
 */

#ifndef DEBUGGER_H_
#define DEBUGGER_H_

#include "board.h"

#define TXFIFO_LENGTH 400

extern uint8_t Debugger_receiveData, Debugger_lastData;
extern uint8_t Debugger_RxFIFO[];
extern uint8_t Debugger_RxFIFOCount;

extern uint8_t UARTTxFIFO[TXFIFO_LENGTH+2];
extern uint8_t UARTTxFIFO_Head, UARTTxFIFO_Tail;

extern bool Debugger_SendPIDDataFlag;
extern bool Debugger_ParserFlag;

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

void Debugger_SendRCData();

void Debugger_SendStatusData();

void Debugger_SendSensorData();

void Debugger_SendVoltageData();

void Debugger_SendMotorData();

void Debugger_SendPIDData();

void Debugger_SendAttOffset();

void Debugger_SendCheck(uint16_t check);

void Debugger_SendSeriesData(uint8_t n, float data);

void Debugger_Parser(uint8_t n);

#endif /* DEBUGGER_H_ */
