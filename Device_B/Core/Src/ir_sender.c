#include "ir_sender.h"
#include "tim.h"

void ir_sender_init(void){
    //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Nie potrzebne bo start jest przy wysyłaniu komendy
    HAL_TIM_Base_Start(&htim2);
}


void delay_us (uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while (__HAL_TIM_GET_COUNTER(&htim2)<delay);
}


// Funkcja do wysyłania całej komendy NEC
void NEC_SendCommand(uint8_t command) {
    // Wyślij ramkę startową
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Start nośnej
    delay_us(NEC_START_PULSE);         // Impuls 9ms
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);   // Stop nośnej
    delay_us(NEC_START_PAUSE);         // Pauza 4.5ms


    //Adres i negacja adresu
    for (int i = 0; i < 8; i++) {
    	// Zawsze najpierw krótki impuls 560µs
    	    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Włącz nośną
    	    delay_us(NEC_PULSE);
    	    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);   // Wyłącz nośną
    	    delay_us(NEC_PAUSE_0);
    }

    for (int i = 0; i < 8; i++) {
        	// Zawsze najpierw krótki impuls 560µs
        	    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Włącz nośną
        	    delay_us(NEC_PULSE);
        	    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);   // Wyłącz nośną
        	    delay_us(NEC_PAUSE_1);
        }

    // Wyślij bity komendy (zakładamy 8-bitową komendę)
    for (int i = 0; i < 8; i++) {
        NEC_SendBit((command >> i) & 0x01);
    }

    // Wyślij bity zanegowanej komendy (zakładamy 8-bitową komendę)
        for (int i = 0; i < 8; i++) {
            NEC_SendBit(~(command >> i) & 0x01);
        }

        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Włącz nośną
                	    delay_us(NEC_PULSE);
                	    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);   // Wyłącz nośną
                	    delay_us(NEC_PAUSE_1);

    // Opcjonalnie dodaj pauzę końcową
    delay_us(560);  // Koniec transmisji
}

// Funkcja do wysyłania pojedynczego bitu
void NEC_SendBit(uint8_t bit) {
    // Zawsze najpierw krótki impuls 560µs
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Włącz nośną
    delay_us(NEC_PULSE);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);   // Wyłącz nośną

    // Pauza zależna od wartości bitu (bit 0 - 560µs, bit 1 - 1680µs)
    if (bit == 0) {
    	delay_us(NEC_PAUSE_0);
    } else {
    	delay_us(NEC_PAUSE_1);
    }
}
