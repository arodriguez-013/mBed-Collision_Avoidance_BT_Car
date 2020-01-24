#include "rtos.h"

class Speaker
{
public:
    Speaker(PinName pin) : _pin(pin) {
// _pin(pin) means pass pin to the Speaker Constructor
// precompute 32 sample points on one sine wave cycle
// used for continuous sine wave output later
        for(int k=0; k<32; k++) {
            Analog_out_data[k] = int (65536.0 * ((1.0 + sin((float(k)/32.0*6.28318530717959)))/2.0));
            // scale the sine wave to 16-bits - as needed for AnalogOut write_u16 arg
        }

    }
// class method to play a note based on AnalogOut class
    void PlayNote(float frequency, float duration, float volume) {
        // scale samples using current volume level arg
        for(int k=0; k<32; k++) {
            Analog_scaled_data[k] = Analog_out_data[k] * volume;
        }
        // reset to start of sample array
        i=0;
        // turn on timer interrupts to start sine wave output
        Sample_Period.attach(this, &Speaker::Sample_timer_interrupt, 1.0/(frequency*32.0));
        // play note for specified time
        Thread::wait(duration);
        // turns off timer interrupts
        Sample_Period.detach();
        // sets output to mid range - analog zero
        this->_pin.write_u16(32768);

    }
private:
// sets up specified pin for analog using AnalogOut class
    AnalogOut _pin;
    // set up a timer to be used for sample rate interrupts
    Ticker Sample_Period;

    //variables used by interrupt routine and PlayNote
    volatile int i;
    short unsigned Analog_out_data[32];
    short unsigned Analog_scaled_data[32];

// Interrupt routine
// used to output next analog sample whenever a timer interrupt occurs
    void Sample_timer_interrupt(void) {
        // send next analog sample out to D to A
        this->_pin.write_u16(Analog_scaled_data[i]);
        // increment pointer and wrap around back to 0 at 32
        i = (i+1) & 0x01F;
    }
};

