/*
 * chirp.h
 *
 *  Created on: 2018/06/07
 */

#ifndef CHIRP_H_
#define CHIRP_H_

#include "stdbool.h"

#define TIME_FRAME 0.0205f
#define AMPLITUDE 1.0f;
#define F0 16000
#define F1 19000

void init_ref_chirp(float sampling_rate);
void mult_ref_chirp(float32_t *pInOut);
void mult_ref_chirp_sim(float32_t *pInOut);

#endif /* CHIRP_H_ */
