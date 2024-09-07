#include "mock_millis.h"

uint32_t mock_millis_value = 0;

uint32_t millis(void) {
    return ++mock_millis_value;
}
