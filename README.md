# Ultrasonic communication by STM32L4 and MEMS microphone

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=2268&h=567)

## Preparation: STM32L4 platform and FFT test code on MEMS mic

This project uses STM32L476RG as MCU and MP34ST01-M as MEMS microphone.

![platform](./doc/MEMSMIC_expansion_board.jpg)

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

## Ultrasonic communication

### Hex number in 8bit length

I assign the following frequencies to each hexa-decimal number of 8 bit data length. I am going to increase the frequencies to ultrasonic range later on.

|Hz   |Hex|
|-----|---|
|17000| 0 |
|17200| 1 |
|17400| 2 |
|17600| 3 |
|17800| 4 |
|18000| 5 |
|18200| 6 |
|18400| 7 |
|18600| 8 |
|18800| 9 |
|19000| A |
|19200| B |
|19400| C |
|19600| D |
|19800| E |
|20000| F |

### DFSDM setting

|Parameter    |Value/setting|
|-------------|-----|
|System clock |80MHz|
|Clock divider|25   |
|Decimation   |32   |
|Filter       |sinc3|

The resulting sampling rate is 100kHz (Nyquist frequency is 50kHz). If the length of FFT input data is 2048, the data length corresponds 20msec.

```
    [5/A][A][A][A][A][A/B][B][B]..

```
If the FFT peak is same and its magnitude is larger than a specific value in three times in a row (60msec), the receiver recognize it as a meaningful hex data.

### Data frame

This frame format is tentative: I am going to refer to CAN frame to support Standard Identifier, DLC and CRC.

```
    [Start-of-frame][D0(8bit)]...[Dn(8bit)][Enf-of-frame]
       18600 Hz                               18800Hz
       100msec                                100msec
```

### Result of first ultrasonic communication test performed on May 20, 2018

Although the MEMS mic is only 50 cm away from tone generator on my PC, this is my first experience in ultrasonic communications.

The first of the first step.

```
MEMS mic: M1
Frequency at max magnitude: 16552.7, Max magnitude: 22883.681641
Hex data: START OF FRAME

MEMS mic: M1
Frequency at max magnitude: 16748.0, Max magnitude: 16412.267578
Hex data: END OF FRAME

MEMS mic: M2A
Frequency at max magnitude: 16943.4, Max magnitude: 35989.476562
Hex data: 0

MEMS mic: M1
Frequency at max magnitude: 17138.7, Max magnitude: 38641.460938
Hex data: 1

MEMS mic: M1
Frequency at max magnitude: 17334.0, Max magnitude: 28021.398438
Hex data: 2

MEMS mic: M2A
Frequency at max magnitude: 17578.1, Max magnitude: 13951.361328
Hex data: 3

MEMS mic: M1
Frequency at max magnitude: 17773.4, Max magnitude: 21634.185547
Hex data: 4

MEMS mic: M2A
Frequency at max magnitude: 17968.8, Max magnitude: 11070.100586
Hex data: 5

MEMS mic: M2A
Frequency at max magnitude: 18164.1, Max magnitude: 37401.156250
Hex data: 6

MEMS mic: M1
Frequency at max magnitude: 18359.4, Max magnitude: 24178.134766
Hex data: 7

MEMS mic: M2A
Frequency at max magnitude: 18554.7, Max magnitude: 30851.169922
Hex data: 8

MEMS mic: M2A
Frequency at max magnitude: 18750.0, Max magnitude: 18402.484375
Hex data: 9

MEMS mic: M2A
Frequency at max magnitude: 18945.3, Max magnitude: 16483.308594
Hex data: A

MEMS mic: M2A
Frequency at max magnitude: 19140.6, Max magnitude: 11588.148438
Hex data: B

MEMS mic: M2A
Frequency at max magnitude: 19335.9, Max magnitude: 11355.094727
Hex data: C

MEMS mic: M1
Frequency at max magnitude: 19531.2, Max magnitude: 16408.939453
Hex data: D

MEMS mic: M1
Frequency at max magnitude: 19726.6, Max magnitude: 17636.755859
Hex data: E

MEMS mic: M1
Frequency at max magnitude: 19970.7, Max magnitude: 12264.990234
Hex data: F
```

## Next

- Develop tone generator on Win10.
- Add a character LDC to the board.
