#pragma once

#include "stm32f4xx.h"

//Definicje czasów w mikrosekundach
#define NEC_PULSE 560
#define NEC_PAUSE_0 560
#define NEC_PAUSE_1 1680
#define NEC_START_PULSE 9000
#define NEC_START_PAUSE 4500

void ir_sender_init(void);

void NEC_SendCommand(uint8_t command);

void NEC_SendBit(uint8_t bit);

void delay_us (uint16_t delay);
