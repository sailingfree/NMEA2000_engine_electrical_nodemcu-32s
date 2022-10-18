// RPM code.
// The engine RPM is measured using the W wire from the alternator.
// The W wire is used to trigger an ISR which measures the number of 1uS pulses between each
// +ve going transition. this is the alternator W wire frequency
// The frequency is then adjusted using the alternator and crank pulley ratio
// and the number of poles on the alternator.
//
// A timer ISR regulaly samples the frequency, applies a low pass filter
// and makes the RPM value available for external functions to read.

#include <Arduino.h>
#include <GwPrefs.h>

extern Stream * Console;

// RPM data. Generator RPM is measured on connector "W"
static double poles;
static double main_dia;    // mm
static double alt_dia;     // mm
static double belt_depth;  // mm

#define Engine_RPM_Pin 22  // Engine RPM is measured as interrupt on GPIO 22

volatile uint64_t StartValue;   // First interrupt value
volatile uint64_t PeriodCount;  // period in counts of 0.000001 of a second
unsigned long Last_int_time = 0;
hw_timer_t* timer = NULL;                         // pointer to a variable of type hw_timer_t
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // synchs between main code and interrupt?
hw_timer_t * sample_timer = NULL;

// RPM Event Interrupt
// Enters on falling edge
uint32_t filter_depth = 64;
//=======================================
void IRAM_ATTR handleInterrupt() {
    portENTER_CRITICAL_ISR(&mux);
    uint64_t TempVal = timerRead(timer);  // value of timer at interrupt
    uint64_t TmpPeriodCount = TempVal - StartValue;   // period count between rising edges in 0.000001 of a second
    StartValue = TempVal;                 // puts latest reading as start for next calculation
    PeriodCount =  ((PeriodCount * (filter_depth - 1)) + (TmpPeriodCount)) / filter_depth;  // This implements a low pass filter to eliminate spike for RPM measurements
    portEXIT_CRITICAL_ISR(&mux);
    Last_int_time = millis();
}

void IRAM_ATTR handleSampleTimer() 
{
    portENTER_CRITICAL_ISR(&mux);
digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    portEXIT_CRITICAL_ISR(&mux);
}
void InitRPM() {
    GwSetVal(ENGINEDIA, "96");
    GwSetVal(ALTDIA, "67");

    // Get the calibration data
    poles = GwGetVal(ALTPOLES, "12").toDouble();
    main_dia = GwGetVal(ENGINEDIA, "96").toDouble();
    alt_dia = GwGetVal(ALTDIA, "67").toDouble();
    belt_depth = 8;

    Serial.printf("InitRPM: %f %f %f\n", poles, main_dia, alt_dia);
    attachInterrupt(digitalPinToInterrupt(Engine_RPM_Pin), handleInterrupt, FALLING);  // attaches pin to interrupt on Falling Edge
    timer = timerBegin(0, 80, true);                                                   // this returns a pointer to the hw_timer_t global variable
    // 0 = first timer
    // 80 is prescaler so 80MHZ divided by 80 = 1MHZ signal ie 0.000001 of a second
    // true - counts up
    timerStart(timer);  // starts the timer

    // The sample timer
    sample_timer = timerBegin(1, 80, true);   // Second timer, 1usec
    timerAttachInterrupt(sample_timer, handleSampleTimer, true);
    timerAlarmWrite(sample_timer, 100000, true);   // Alarm every 1/10 second
}

// Calculate engine RPM from number of interupts per time
static uint32_t EngineRPM = 0;

double ReadRPM() {
    //  return 1234.0;
    uint64_t RPM = 0;

   // uint32_t filter_depth = 3;  // Low pass filter. Noot too big as the RPM is sampled every second or so.

    // The pulley ratio determines how fast the alternator turns WRT the engine crank.
    // I use the pitch circle diameter to get the best result.
    // The PCD is roughly the diameter where the middle of the belt touches the pully.
    double pulley_ratio = (main_dia - (belt_depth)) / (alt_dia - (belt_depth));

    // The W wire from the alternator outputs one pulse for every two poles.
    // So my Volvo alternator which has 12 poles will output 6 pulses per rev.
    double rpm_calib = poles / 2.0 * pulley_ratio;
    uint64_t frequency;
    unsigned long now;
    unsigned long period;
    volatile uint64_t old_period_count;
    now = millis();
    portENTER_CRITICAL(&mux);
    old_period_count = PeriodCount;
    if (PeriodCount > 0) {
        frequency = 1000000 / PeriodCount;  // PeriodCount in 0.000001 of a second
    } else {
        frequency = 0LL;
    }
    portEXIT_CRITICAL(&mux);

    if (now > Last_int_time + 200) {
        RPM = 0;  // No signals RPM=0;
    } else {
        period = (now - Last_int_time) * 1000;  // uSecs taken for PeriodCount.
    }

   // EngineRPM = ((EngineRPM * (filter_depth - 1)) + (frequency * rpm_calib)) / filter_depth;  // This implements a low pass filter to eliminate spike for RPM measurements
    EngineRPM = frequency * rpm_calib;

   // Console->printf("F %llu Calib %f\n", frequency, rpm_calib);
    return (EngineRPM);
}