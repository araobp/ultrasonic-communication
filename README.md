# Ultrasonic communications (STM32L4 DSP w/ MEMS mic)

![ChirpFrame](./doc/ChirpFrame.jpg)

STMicro gave me STM32L4 evaluation board and a pair of MEMS microphones for free at [a trade show held in Makuhari city](https://www.st.com/content/st_com/en/about/events/events.html/techno-frontier-2018.html), Chiba, Japan. Thanks a lot to STMicro! As an IoT hobyyist, I am becoming interested in developing an IoT demo using DSP with MEMS mic.

## Preparation: STM32L4 platform and FFT test code on MEMS mic

This project uses STM32L476RG as MCU/DSP and MP34ST01-M as MEMS microphone:

![platform](./doc/MEMSMIC_expansion_board.jpg)

The system architecture is as follows:

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=2268&h=567)

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

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

==> [Experiment3](EXPERIMENT3.md)

### Conclusion

|Techinique                               | peak magnitude/amplitude              |Compression    |
|-----------------------------------------|---------------------------------------|---------------|
|FFT[Real upchirp * complex downchirp]    | peaks at around chirp frequency * 2 Hz|Very Good, sinc5 filter improves SNR|
|IFFT[FFT[real upchirp]*FFT[real upchirp]]| compressed wave in time domain        |Not good       |
|FFT[Real upchirp * Real upchirp]          | peaks at around zero Hz              |Disturbed by noises|

### Next steps

#### Frame synchronization problems

The phase difference between chirp from the transmitter and chirp from the receiver results in two peaks.

Minimize the phase difference for synchronization (to maximize correlation).

![](./doc/Simulation_upchirp_downchirp_shift.jpg)

I have come up with the following method:

![](https://docs.google.com/drawings/d/e/2PACX-1vT9da0oKUWgUHHTmYUO8Y0Rix6ORT5aeQxAz8Ihjoxc4vWMvFLudPTet1UHLMConm5RDk9kFaXTXnj8/pub?w=960&h=720)

#### sinc filter optimization (moving average)

sinc5 filter improves SNR at around frequencies of peaks.

## My original MEMS mic shield

I have bought [this MEMS mic](http://akizukidenshi.com/catalog/g/gM-05577/): Knowles SPM0405HD4H. The spec is similar to the mic on the expansion board from STMicro. Although this one does not support ultrasonic, it should be OK.

![Knowles](./doc/Knowles.jpg)

I am going to make my original shield with Knowles MEMS mic:

- Knowles MEMS mic
- LCDs
- LEDs
- Tactile switches
- CAN tranceiver
