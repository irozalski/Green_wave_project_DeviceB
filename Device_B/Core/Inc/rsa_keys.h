/*
 * rsa_keys.h
 *
 *  Created on: 7 lis 2024
 *      Author: Igor
 */

#ifndef INC_RSA_KEYS_H_
#define INC_RSA_KEYS_H_

#include <stdint.h>


#define RSA_KEY_SET_SIZE 512
#define RSA_KEY_SETS_COUNT 50

// Funkcja do pobrania wskaźnika na wybrany zestaw kluczy w tablicy
const uint8_t* get_rsa_key_set(uint8_t index);

#endif /* INC_RSA_KEYS_H_ */
