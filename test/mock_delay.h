#ifndef MOCK_DELAY_H
#define MOCK_DELAY_H

#include <stdint.h>

// Variable zum Speichern des letzten Delay-Werts
extern uint32_t mock_delay_value;

// Deklaration der Mock-Funktion für delay
void delay(uint32_t ms);

#endif // MOCK_DELAY_H
