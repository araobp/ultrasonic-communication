## Ultrasonic communications experiment (FSK modulation)

My first idea was FSK modulation of 18 frequencies (18 symboles) to send data on ultrasonic wave. The method worked very well in a silent room, but did not work in a noisy environment such as a meeting room. I had to come up with another approach, such as spread spectrum.

![](./doc/18symbols.jpg)

==> [Experiment](EXPERIMENT.md)

==> [Test code](./ultracom)

## Ultrasonic communications experiment (Chirp modulation)

I tested Chirp modulation as one of spread spectrum techniques. It worked! But the noise problem still remained, and the result was useless in a noisy room. I had to come up with a noise suppression technique such as Chirp compression.

![](./doc/Chirp.jpg)

==> [Experiment2](EXPERIMENT2.md)

==> [Test code](./chirp)

## Ultrasonic communications experiment (Chirp modulation with compression)

I tested various Chirp compression techniques by simulating chirp compression and also by implementing test code on STM32L4 DSP.

Test code of FFT[real up-chirp * complex down-chirp] on STM32L4 DSP showed the best result as follows:

![](./doc/FFT_upXdown.jpg)

==> [Experiment3](EXPERIMENT3.md)

### Conclusion

|Techinique                               | peak magnitude/amplitude              |Compression    |
|-----------------------------------------|---------------------------------------|---------------|
|FFT[Real upchirp * complex downchirp]    | peaks at around chirp frequency * 2 Hz|Very Good, sinc5 filter improves SNR|
|IFFT[FFT[real upchirp]*FFT[real upchirp]]| compressed wave in time domain        |Not good       |
|FFT[Real upchirp * Real upchirp]          | peaks at around zero Hz              |Disturbed by noises around zero Hz|
