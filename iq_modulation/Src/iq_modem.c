/*
 * iq_modem.c
 *
 *  Created on: 2018/06/27
 */
#include "arm_math.h"
# include "iq_modem.h"
# include "chirp.h"
# include "main.h"

// fe: 0.03 Hz(3000 Hz), delta: 0.12 Hz(12000 Hz)
uint32_t numTaps = 27;
float32_t b[27] = {0.01560757, 0.02043850, 0.02535792, 0.03027307, 0.03508888, 0.03971022, 0.04404423, 0.04800257, 0.05150362, 0.05447453, 0.05685299, 0.05858884, 0.05964532, 0.06000000, 0.05964532, 0.05858884, 0.05685299, 0.05447453, 0.05150362, 0.04800257, 0.04404423, 0.03971022, 0.03508888, 0.03027307, 0.02535792, 0.02043850, 0.01560757};
float32_t inout[PCM_SAMPLES];

float32_t carrier_cos[PCM_SAMPLES];
float32_t carrier_sin[PCM_SAMPLES];
float32_t i_temp[PCM_SAMPLES];
float32_t q_temp[PCM_SAMPLES];

arm_fir_instance_f32 S_I;
arm_fir_instance_f32 S_Q;

void init_iq_modem(float32_t sampling_rate) {

  float32_t theta;
  float32_t delta_t = TIME_FRAME/(TIME_FRAME * sampling_rate);
  float32_t t = 0.0f;

  for (uint32_t i = 0; i < PCM_SAMPLES; i++) {
    theta = 360.0 * CARRIER * t;
    arm_sin_cos_f32(theta, &carrier_sin[i], &carrier_cos[i]);
    t = t + delta_t;
  }

  arm_fir_init_f32(&S_I, numTaps, b, i_temp, PCM_SAMPLES);
  arm_fir_init_f32(&S_Q, numTaps, b, q_temp, PCM_SAMPLES);

}

void iq_demodulation(float32_t *pInOut) {
  arm_mult_f32(pInOut, carrier_cos, i_temp, PCM_SAMPLES * 2);
  arm_mult_f32(pInOut, carrier_sin, q_temp, PCM_SAMPLES * 2);
  // FIR/LPF


}

