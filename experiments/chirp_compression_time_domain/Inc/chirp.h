/*
 * chirp.h
 *
 *  Created on: 2018/06/07
 */

#ifndef CHIRP_H_
#define CHIRP_H_

// Amplitude of reference chirp
#define AMPLITUDE 1.0f;

// Chirp sweep range
#define F1 17000.0f
#define F2 18000.0f

void init_ref_chirp(float sampling_rate);
void compress_chirp(float32_t *pInOut);
void compress_ref_chirp(float32_t *pInOut);
void get_ref_chirp(float32_t *pInOut);

#endif /* CHIRP_H_ */
