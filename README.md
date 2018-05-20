# Ultrasonic communication by STM32L4 and MEMS microphone

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=480&h=189)

## Platform and test code

This project uses STM32L476RG as MCU and MP34ST01-M as MEMS microphone.

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

## Hex number in 8bit length

I assign the following frequencies to each hexa-decimal number of 8 bit data length:

|Hz   |Hex|
|-----|---|
|18000| 0 |
|18200| 1 |
|18400| 2 |
|18600| 3 |
|18800| 4 |
|19000| 5 |
|19200| 6 |
|19400| 7 |
|19600| 8 |
|19800| 9 |
|20000| A |
|20200| B |
|20400| C |
|20600| D |
|20800| E |
|20000| F |

## DFSDM setting

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

## Data frame

```
    [Start-of-frame][Identifier(4bit)][DLC(4bit)][D0(8bit)]...[Dn(8bit)][Enf-of-frame]
       18600 Hz                                                           18800Hz
       100msec                                                            100msec
```

## Tone generator

I use [Audacity](https://www.audacityteam.org/) to generate tones.


