#include "mock_delay.h"

// Initialisiere die Variable
uint32_t mock_delay_value = 0;

// Mock-Implementierung von delay()
void delay(uint32_t ms) {
    // Speichere den übergebenen Wert
    mock_delay_value = ms;
}
