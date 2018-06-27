/*
 * iq_modem.h
 *
 *  Created on: 2018/06/27
 */

#ifndef IQ_MODEM_H_
#define IQ_MODEM_H_

#define CARRIER 18000.0f

void init_iq_modem(float32_t sampling_rate);
void iq_demodulation(float32_t *pInOut);

#endif /* IQ_MODEM_H_ */
