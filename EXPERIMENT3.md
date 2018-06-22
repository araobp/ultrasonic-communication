## Ultrasonic communications experiment (Chirp modulation with compression)

This time, I employ some compression technique, and chirp sweep time corresponds to the length of time within time frame, unlike the previous experiment.

This material is great: https://www.ittc.ku.edu/workshops/Summer2004Lectures/Radar_Pulse_Compression.pdf

### The first chirp compression experiment on STM32L4 DSP (June 10, 2018)

I tried Chirp compression in frequency domain (real upchirp * complex down-chip). I transmitted very weak chirp signals (real upchirp) to STM32L4 DSP with MEMS mic. It worked! But I observed two peaks most of time, since the FFT calculation was performed on a chirp signal split into two within the time frame, since the time frame was not in sync between the transmitter and the receiver.

![upchirp_downchirp](./doc/FFT_upXdown.jpg)

#### How much time does it take to compute complex FFT of 2048 samples?

The measured value is 3msec for each complex FFT of 2048 samples at 80MHz system clock. 2048 samples correspond to 20.5msec at 100kHz sampling rate, so 3msec is short enough compared to 20.5msec.

### Simulation of chirp compression on Jupyter Notebook (June 20, 2018)

I made [chirp compression simulator](./simulation/ChirpSimulation.ipynb) on Jupyter Notebook.

The chirp signal is under white noise:

![upchirp_with_noise](./doc/Simulation_upchirp_with_noise.jpg)

I made FFT [upchirp * upchirp], then the chirp signal became identifiable at around zero Hz:

![upchirp_upchirp](./doc/Simulation_upchirp_upchirp.jpg)

I also tried IFFT [FFT[upchirp] * FFT[downchirp]] to simulate Chirp compression in time domain. Again, the chirp signal is under noise level, but the chirp signal is identifiable around zero Hz after compression. The good thing of this technique is that it does not require a larger buffer (i.e., larger memory) for compression processing, and it can also detect phase shift of chirp signal by measuring the shift of peak to the left or to the right:

![upchirp_downchirp](./doc/Simulation_upchirp_downchirp.jpg)

### The second chirp compression experiment on STM32L4 DSP (June 22, 2018)

The technique of chirp compression in time domain resulted in bad compression in a real experiment on STM32L4 w/ MEMS mic.

On the contrary, an experiment of FFT [upchirp * upchirp] on STM32L4 w/ MEMS mic showed a very good result:

![upchirp_upchirp](./doc/Experiment_upchirp_upchirp.jpg)

However, I got that this techinique is not so good, since I observed a peak at zero Hz sometimes, even when chirp signal was not transmitted to the receiver.
