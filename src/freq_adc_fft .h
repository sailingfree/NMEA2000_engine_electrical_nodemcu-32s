#include <Arduino.h>

// Number of samples to read from the ADC
// Must be a power of 2.
#define N_SAMPLES   4096  

// The ESP ADC can sample at around 8500 samples per second when run in a simple loop
// Thats 117uS per sample
// 2048 works for sane values of alternator frequencies
// Larger values will give greater precision at the cost of time.
// Don't forget in this application we are measuring an engine alternator frequency
// and we only need the RPM to be to the nearest 5 or 10 RPM or so.
// My Volvo MD2010 idles at ~850RPM and under load a maximum of ~3000 RPM 
// It has a 12 pole alternator which gives 6 puleses per revolution
// The engine/alternator pulley ratio is 1.432 (96mm/67mm)
//
// 3000 RPM = 50 RPS * 6 * 1.432 = 429.85Hz
// 850 RPM = 14 RPS * 6 = 121.7Hz
//
// 10 RPM change = 1.432Hz
// 20 RPM change = 2.864Hz
// So with a max frequency of 1/2 the sample frequency ~ 4250Hz 
// the number of FFT bins using N_SAMPLES of 2048 gives us a resolution of just under 2Hz per bin
// So a close enough approximation I think.


// Time taken for the ADC loop and FFT for different N_SAMPLES
// using -O3 optimisation
// N_SAMPLES | Time mS | 
//  128      |  16
//  256      |  30
//  512      |  62
// 1024      | 125
// 2048      | 250
// 4096      | 580

// define the pin we are going to read the analogue input
#define ANALOGUE_PIN    (36)


// Read some ADC samples and run an FFT to get the fundamental frequency
float readFreqAdcFft(void);

// For debug and test return the time taken for the ADC in uS
uint64_t getAdcTime();

// For debug and test return the time taken for the FFT in uS
uint64_t getFftTime();

// For debug and test print the stats
void printAdcFFTStats();