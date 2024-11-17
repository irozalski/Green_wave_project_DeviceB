#pragma once

#include "stm32f4xx.h"

#define IR_CODE_ONOFF 0xd

// Procedura obsługi przerwania
void ir_tim_interrupt(void);

// Inicjalizacja modułu
void ir_receiver_init(void);

// Funkcja odczytująca dane
// return - kod klawisza lub -1
int ir_read(void);
