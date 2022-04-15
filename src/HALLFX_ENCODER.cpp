#include "HALLFX_ENCODER.h"

HALLFX_ENCODER::HALLFX_ENCODER(PinName enc_in) : _enc_in(enc_in) {
    _enc_in.mode(PullUp);
    // Invoke interrupt on both falling and rising edges
    _enc_in.fall(this, &HALLFX_ENCODER::callback);
    _enc_in.rise(this, &HALLFX_ENCODER::callback);
}

long HALLFX_ENCODER::read() {
    return count;
}

void HALLFX_ENCODER::reset() {
    count = 0;
}

void HALLFX_ENCODER::callback() {
    count++;
}