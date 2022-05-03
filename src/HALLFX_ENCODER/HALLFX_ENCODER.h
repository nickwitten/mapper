#ifndef HALLFX_ENCODER_H
#define HALLFX_ENCODER_H

/*
    Basic Encoder Library for Sparkfun's Wheel Encoder Kit
    Part# ROB-12629.
*/

#include "mbed.h"

class HALLFX_ENCODER {
public:
    /*
        Constructor for Encoder objects
        @param enc_in    The mBed pin connected to encoder output
    */
    HALLFX_ENCODER(PinName enc_in);
    /*
        read() returns total number of counts of the encoder.
        Count can be +/- and indicates the overall direction,
        (+): CW (-): CCW
        @return     The toltal number of counts of the encoder.
    */
    long read();
    /*
        reset() clears the counter to 0.
    */
    void reset();
private:
    long count;         // Total number of counts since start.
    InterruptIn _enc_in;// Encoder Input/Interrupt Pin
    /*
        Increments/Decrements count on interrrupt.
    */
    void callback();    // Interrupt callback function
};
#endif