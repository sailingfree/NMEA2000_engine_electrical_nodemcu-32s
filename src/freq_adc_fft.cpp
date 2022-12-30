#include <Arduino.h>
#include <rtc.h>
#include <fft.h>
#include <driver/i2s.h>
#include <freq_adc_fft .h>

// From here https://www.esp32.com/viewtopic.php?t=1215

static uint32_t t[N_SAMPLES];
static volatile uint32_t val[N_SAMPLES];
static uint64_t adcTime;
static uint64_t fftTime;
static float lastFreq;

// Read some ADC samples and run an FFT to get the fundamental frequency
float readFreqAdcFft(void) {

  uint64_t t1, t2, t3, t4;  // For start and end time to measure elapsed time
  // Define the FFT parameters. Let the fft provide the buffers
  float* fft_input = NULL; //[N_SAMPLES];
  float* fft_output = NULL; //[N_SAMPLES];

  fft_config_t* real_fft_plan = fft_init(N_SAMPLES, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

  t1 = esp_timer_get_time();
  for (uint32_t i = 0; i < N_SAMPLES; i++) {
      real_fft_plan->input[i] = (float) analogRead(ANALOGUE_PIN);
  }
  t2 = esp_timer_get_time();

  

  // Total elapsed time to get N_SAMPLES samples in uS
  adcTime = t2 - t1;
  float TOTAL_TIME = adcTime / 1000000.0;
  float max_magnitude = 0.0;
  float fundamental_freq = 0.0;

  // Execute transformation
  t3 = esp_timer_get_time();      // get start time for measurements
  fft_execute(real_fft_plan);

  // Get the fundamental frequency
  for (int k = 1; k < real_fft_plan->size / 2; k++)
  {
    /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
    float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2)) / 1;
    float freq = k * 1.0 / TOTAL_TIME;
    if (mag > max_magnitude)
    {
      max_magnitude = mag;
      fundamental_freq = freq;
    }
  }

  // Clean up at the end to free the memory allocated
  fft_destroy(real_fft_plan);
  t4 = esp_timer_get_time();
  fftTime = t4 - t3;

  lastFreq = fundamental_freq;
  
  return fundamental_freq;
}

// For debug and test return the time taken for the ADC in uS
uint64_t getAdcTime(){
  return adcTime;
}

// For debug and test return the time taken for the FFT in uS
uint64_t getFftTime(){
  return fftTime;
}

// print some stats
void printAdcFFTStats() {
  uint64_t t_adc = getAdcTime();
  uint64_t t_fft = getFftTime();
  float samplesPerSec = 1000000.0 / ((float)t_adc / (float)N_SAMPLES);
  Serial.printf("Freq (Hz) %f N_SAMPLES %d ADC time (uS) %llu FFT Time (uS) %llu Total time (mS) %llu ADC samples/sec %f      \r",
    lastFreq, N_SAMPLES, adcTime, fftTime, (adcTime + fftTime) / 1000LLU, samplesPerSec);
}