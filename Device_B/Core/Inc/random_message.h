/*
 * random_message.h
 *
 *  Created on: 18 lis 2024
 *      Author: Igor
 */

#ifndef INC_RANDOM_MESSAGE_H_
#define INC_RANDOM_MESSAGE_H_

//#include "stdint.h"
#include "main.h"

uint8_t RNG_GetRandomByte(void);
void GenerateRandomMessage(uint8_t *message, uint8_t length);
void Random_message_init(void);
#endif /* INC_RANDOM_MESSAGE_H_ */
