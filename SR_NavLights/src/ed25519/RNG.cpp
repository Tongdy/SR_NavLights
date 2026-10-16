// Minimal stand-in for the Crypto library RNG (see RNG.h).
#include "RNG.h"
#if defined(ESP32)
#include "esp_random.h"
void RNGClass::rand(uint8_t *data, size_t len) { esp_fill_random(data, len); }
#else
#include <stdlib.h>
void RNGClass::rand(uint8_t *data, size_t len) { for (size_t i = 0; i < len; i++) data[i] = (uint8_t)::rand(); }
#endif
RNGClass RNG;
