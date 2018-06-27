/*
 * iq_modem.c
 *
 *  Created on: 2018/06/27
 */
#include "arm_math.h"
# include "iq_modem.h"
# include "chirp.h"
# include "main.h"

float32_t carrier_sin[PCM_SAMPLES];
float32_t carrier_cos[PCM_SAMPLES];

void init_iq_modem(float32_t sampling_rate) {

  float32_t theta;
  float32_t delta_t = TIME_FRAME/(TIME_FRAME * sampling_rate);
  float32_t t = 0.0f;

  for (uint32_t i = 0; i < PCM_SAMPLES; i++) {
    theta = 360.0 * CARRIER * t;
    arm_sin_cos_f32(theta, &carrier_sin[i], &carrier_cos[i]);
    t = t + delta_t;
  }
}
