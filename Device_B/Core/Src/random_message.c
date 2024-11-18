/*
 * random_message.c
 *
 *  Created on: 18 lis 2024
 *      Author: Igor
 */
#include "random_message.h"

// Seed for the pseudo-random number generator
static uint32_t pseudoRandomSeed = 0;

// Initialize the timer-based seed
void Random_message_init() {
    // Ensure that the SysTick timer or any other timer is running
    pseudoRandomSeed = HAL_GetTick(); // Use the current millisecond tick as the seed
}

// Simple Linear Congruential Generator (LCG) for pseudo-random numbers
uint8_t RNG_GetRandomByte(void) {
    pseudoRandomSeed = (pseudoRandomSeed * 1664525 + 1013904223) & 0xFFFFFFFF; // LCG formula
    return (uint8_t)(pseudoRandomSeed & 0xFF); // Return the least significant byte
}

// Generate a random message of `length` bytes
void GenerateRandomMessage(uint8_t *message, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        message[i] = RNG_GetRandomByte();
    }
}
