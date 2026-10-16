// Minimal stand-in for the Crypto library RNG (SR Nav Lights only VERIFIES
// signatures; key generation is never used on the board).
#ifndef CRYPTO_RNG_H
#define CRYPTO_RNG_H
#include <inttypes.h>
#include <stddef.h>
class RNGClass {
public:
    void rand(uint8_t *data, size_t len);
};
extern RNGClass RNG;
#endif
