/*
 * iq_modem.c
 *
 *  Created on: 2018/06/27
 */
#include "arm_math.h"
# include "iq_modem.h"
# include "chirp.h"
# include "main.h"

/**
 * Finite Impulse Response of sinc function via Hanning window
 * fe: 0.03 Hz(3000 Hz), delta: 0.12 Hz(12000 Hz)
 */

#define NUM_TAPS 27

float32_t b[27] = {0.01560757, 0.02043850, 0.02535792, 0.03027307, 0.03508888, 0.03971022, 0.04404423, 0.04800257, 0.05150362, 0.05447453, 0.05685299, 0.05858884, 0.05964532, 0.06000000, 0.05964532, 0.05858884, 0.05685299, 0.05447453, 0.05150362, 0.04800257, 0.04404423, 0.03971022, 0.03508888, 0.03027307, 0.02535792, 0.02043850, 0.01560757};

// Carrier waves
float32_t carrier_cos[PCM_SAMPLES];
float32_t carrier_sin[PCM_SAMPLES];

float32_t i_state[NUM_TAPS + PCM_SAMPLES];
float32_t q_state[NUM_TAPS + PCM_SAMPLES];

// FIR instances
arm_fir_instance_f32 S_I;
arm_fir_instance_f32 S_Q;

/**
 * Initialize IQ demodulator
 */
void init_iq_modem(float32_t sampling_rate) {

  float32_t theta;
  float32_t delta_t = TIME_FRAME/(TIME_FRAME * sampling_rate);
  float32_t t = 0.0f;

  for (uint32_t i = 0; i < PCM_SAMPLES; i++) {
    theta = 360.0 * CARRIER * t;
    arm_sin_cos_f32(theta, &carrier_sin[i], &carrier_cos[i]);
    t = t + delta_t;
  }

  // Finite Impulse Response initialization
  arm_fir_init_f32(&S_I, NUM_TAPS, b, i_state, PCM_SAMPLES);
  arm_fir_init_f32(&S_Q, NUM_TAPS, b, q_state, PCM_SAMPLES);

}

/**
 * Demodulate received tone into I and Q.
 */
void iq_demodulation(float32_t *pInOut) {

  uint32_t re, im;

  // Received tone * carrier --> I/Q w/ high 2*frequency terms
  arm_mult_f32(pInOut, carrier_sin, &pInOut[PCM_SAMPLES], PCM_SAMPLES);
  arm_mult_f32(pInOut, carrier_cos, pInOut, PCM_SAMPLES);

  // FIR/LPF to remove 2*frequency terms
  arm_fir_f32(&S_I, pInOut, pInOut, PCM_SAMPLES);
  arm_fir_f32(&S_Q, &pInOut[PCM_SAMPLES], &pInOut[PCM_SAMPLES], PCM_SAMPLES);

  /*
  for (uint32_t i = 0; i < PCM_SAMPLES; i++) {
    re = i + 1;
    im = PCM_SAMPLES + i;
    re_v = pInOut[re];
    pInOut[re] = pInOut[im];
  }
  */
}

