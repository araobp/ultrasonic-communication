# Ultrasonic communication by STM32L4 and MEMS microphone

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=2268&h=567)

## Preparation: STM32L4 platform and FFT test code on MEMS mic

This project uses STM32L476RG as MCU and MP34ST01-M as MEMS microphone.

![platform](./doc/MEMSMIC_expansion_board.jpg)

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

## Ultrasonic communications experiment

==> [Experiment](EXPERIMENT.md)

==> [Test code](./ultracom)

Conclusion: the method (sort of FSK modulation) work very well in a silent room, but did not work in a noisy environment such as a meeting room. I have to come up with another approach, such as spread spectrum.

## PDM mic

I have bought [this MEMS mic](http://akizukidenshi.com/catalog/g/gM-05577/): Knowles SPM0405HD4H. The spec is similar to the mic on the expansion board from STMicro.

## Chirp modulation experiment

### Two kinds of noises

- Constant noises at specific frequencies: noises from motors/inverters???
- Bursty noises in a short period: cough, folding paper etc.

I think Chirp modulation might be suitable for ultrasonic communications in a noisy environment.

### Chirp modulation

![Chirp](./doc/Chirp.jpg)

![Chirp_Spectrogram](./doc/Chirp_Spectrogram.jpg)

### Chirp demodulation

Since all the frequency appears in one TQ(Time Quantum), I use SFFT to accumulate power of each frequency.

### Frame (tentative)

"Start of frame" is to detect the beginning of transmission and also for frame synchronization with the transmitter.

```
Segment length: TQ[msec] = 10msec

Start of frame: 5TQ length
Bit: 3TQ length
End of frame: 5TQ length

Frame (290msec)
<- SOF    -><- Bit 0  ->   <- Bit 7  -><- EOF    ->
[S][S][S][S][B0][B0][B0]...[B7][B7][B7][E][E][E][E]
    40msec    30msec         30msec       40msec

 ----------  ----------                 ----------
                            ----------
                            
S or E
1: Chirp

Bit value
0: No chirp
1: Chirp

Void
0: No chirp
```

