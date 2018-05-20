# Ultrasonic communication by STM32L4 and MEMS microphone

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=480&h=189)

## Platform and test code

This project uses STM32L476RG as MCU and MP34ST01-M as MEMS microphone.

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

## Hex number in 8bit length

Although this is not ultrasonic, but I assign the following frequencies to each hexa-decimal number of 8 bit data length:

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
       16600 Hz                                                           16800Hz
       100msec                                                            100msec
```


