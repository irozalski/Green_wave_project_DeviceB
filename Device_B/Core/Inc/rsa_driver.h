/*
 * rsa_driver.h
 *
 *  Created on: 5 lis 2024
 *      Author: Igor
 */

#ifndef INC_RSA_DRIVER_H_
#define INC_RSA_DRIVER_H_

#include "main.h"
#include "crypto.h"


typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;


int32_t RSA_Encrypt(RSApubKey_stt *P_pPubKey,
                    const uint8_t *P_pInputMessage,
                    int32_t P_InputSize,
                    uint8_t *P_pOutput);

int32_t RSA_Decrypt(RSAprivKey_stt * P_pPrivKey,
                    const uint8_t * P_pInputMessage,
                    uint8_t *P_pOutput,
                    int32_t *P_OutputSize);

TestStatus Buffercmp(const uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength);



void Encrypt_And_Add_Random_Message(uint8_t key_number);

void RSA_Driver_Get_Random_Message(void);

uint8_t Decrypt_And_Check_RSA_Message(uint8_t key_number, uint8_t *received_message);

#endif /* INC_RSA_DRIVER_H_ */
