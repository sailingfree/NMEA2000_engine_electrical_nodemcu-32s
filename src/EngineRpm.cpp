/////////////////////////////////////////////////////////////////////////////////////////////
// RPM code.
// The engine RPM is measured using the W wire from the alternator.
// The W wire is used to trigger an ISR which measures the number of 1uS pulses between each
// +ve going transition. this is the alternator W wire frequency
// The frequency is then adjusted using the alternator and crank pulley ratio
// and the number of poles on the alternator.
// The calibration value is calculated using user configured options so it
// can be used with other engines/alternatiors
//
// A timer ISR regulaly samples the frequency, applies a low pass 


// and makes the RPM value available for external functions to read.
// Makes use of code from https://github.com/AK-Homberger/NMEA2000-Data-Sender
/////////////////////////////////////////////////////////////////////////////////////////////

/*
Copyright (c) 2022 Peter Martin www.naiadhome.com

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Arduino.h>
#include <GwPrefs.h>
#include <freq_adc_fft .h>

extern Stream* Console;

// select the method of reading RPM
#define RPM_TYPE_INTERRUPT 1    // use an interrupt handler 
#define RPM_TYPE_ADC_FFT 2      // Use the ADC and FFT method

#define RPM_TYPE RPM_TYPE_ADC_FFT

// RPM data. Generator RPM is measured on connector "W"
static double poles;
static double main_dia;    // mm
static double alt_dia;     // mm
static double belt_depth;  // mm

#define Engine_RPM_Pin 33  // Engine RPM is measured as interrupt on GPIO 33

volatile uint64_t StartValue;   // First interrupt value
volatile uint64_t PeriodCount;  // period in counts of 0.000001 of a second
unsigned long Last_int_time = 0;
hw_timer_t* timer = NULL;                         // pointer to a variable of type hw_timer_t
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // synchs between main code and interrupt

// RPM Event Interrupt
// Enters on falling edge
uint32_t filter_depth = 1;
//=======================================
void IRAM_ATTR handleInterrupt() {
    portENTER_CRITICAL_ISR(&mux);
    uint64_t TempVal = timerRead(timer);  // value of timer at interrupt
    PeriodCount = TempVal - StartValue;   // period count between rising edges in 0.000001 of a second
    StartValue = TempVal;                 // puts latest reading as start for next calculation
    portEXIT_CRITICAL_ISR(&mux);
    Last_int_time = millis();
}

void InitRPM() {
    GwSetVal(ENGINEDIA, "96");
    GwSetVal(ALTDIA, "67");

    // Get the calibration data
    poles = GwGetVal(ALTPOLES, "12").toDouble();
    main_dia = GwGetVal(ENGINEDIA, "96").toDouble();
    alt_dia = GwGetVal(ALTDIA, "67").toDouble();
    belt_depth = 0;

    Serial.printf("InitRPM: %f %f %f\n", poles, main_dia, alt_dia);
    pinMode(Engine_RPM_Pin, INPUT_PULLUP);                                            // sets pin high

    // attaches pin to interrupt. We trigger on rising and falling edges
    attachInterrupt(digitalPinToInterrupt(Engine_RPM_Pin), handleInterrupt, HIGH);

    // 0 = first timer
    // 80 is prescaler so 80MHZ divided by 80 = 1MHZ signal ie 0.000001 of a second
    // true - counts up
    timer = timerBegin(0, 80, true);  // this returns a pointer to the hw_timer_t global variable

    timerStart(timer);  // start the timer
}

// Calculate engine RPM from number of interupts per time
double ReadRPM() {
    double EngineRPM = 0.0;

    // The pulley ratio determines how fast the alternator turns WRT the engine crank.
    // I use the pitch circle diameter (PCD) to get the best result.
    // The PCD is roughly the diameter where the middle of the belt touches the pulley.
    double pulley_ratio = (main_dia - belt_depth) / (alt_dia - belt_depth);

    // The W wire from the alternator outputs one pulse for every two poles.
    // So my Volvo alternator which has 12 poles will output 6 pulses per rev.
    double rpm_calib = poles / 2.0 * pulley_ratio;
    uint64_t frequency;
    unsigned long now;
    unsigned long period;

    now = millis();  // Current sample time
#if RPM_TYPE == RPM_TYPE_INTERRUPT
    portENTER_CRITICAL(&mux);
    if (PeriodCount > 0) {
        frequency = 1000000 / PeriodCount;  // PeriodCount in 0.000001 of a second
    }
    else {
        frequency = 0LL;
    }
    portEXIT_CRITICAL(&mux);
#else
    frequency = (uint64_t)readFreqAdcFft();
#endif

    // Convert the frequency of the alternator output to alternator revolutions
    EngineRPM = frequency / (poles / 2);

    // Adjust for the difference in pulley diameters
    EngineRPM /= pulley_ratio;

    // Convert frequency in pulses per second to revolutions per minute
    EngineRPM *= 60;

    // Apply a low pass filter to remove noise?
    static double rpm = 0;
    double w = 6.0;
    rpm = ((rpm * (w - 1.0)) + (EngineRPM)) / w;
    //Serial.printf("RPM %f\n", rpm);
    
    return (EngineRPM);
}
