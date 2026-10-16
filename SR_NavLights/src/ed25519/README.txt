Ed25519 signature check, taken from the Crypto library by Rhys Weatherley
(https://github.com/rweather/arduinolibs, MIT licence, headers kept).
Only the files needed for Ed25519::verify are included. RNG.h/.cpp are a
small stand-in because the board never creates keys.
