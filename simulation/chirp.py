# author: <https://github.com/araobp>
#
# I don't like Object Oriented Programming so much
#

from numpy import zeros, append, linspace, sin, cos, exp, pi, int16, abs, real, imag, random, sqrt, log10
from scipy.io.wavfile import write
from scipy.signal import spectrogram, buttord, butter, lfilter
from scipy.fftpack import fft, ifft, fftfreq, fftshift
import matplotlib.pyplot as plt
import peakutils
import pandas
from IPython.display import display, Audio

import matplotlib
matplotlib.rcParams['figure.figsize'] = [10, 4]

Fs = 44100  # Hz
TIME_FRAME = 1.0 # Time frame in sec
AMPLITUDE = 20000 # 16bit PCM max amplitude

# Chirp sweep range in Hz
F0 = 880
F1 = 1760

WAVE_FILE="./ChirpTone.wav"

# Complex version of chirp tone generator
def chirp(f0=F0, f1=F1, T=TIME_FRAME, amp=AMPLITUDE, updown="up",phase=-pi/2.0):
    t = linspace(0, T, int(T * Fs))
    k = float(f1 - f0)/float(T)
    if (updown == "up"):
        f = f0 + k * t / 2.0
    elif (updown == "down"):
        f = f1 - k * t / 2.0
    arg = (2.0 * pi * f * t) + phase
    return exp(1j * arg) * amp

# Real version of chirp tone generator
def chirp_cos(f0=F0, f1=F1, T=TIME_FRAME, amp=AMPLITUDE, updown="up", phase=-pi/2.0):
    t = linspace(0, T, int(T * Fs))
    k = float(f1 - f0)/float(T)
    if (updown == "up"):
        f = f0 + k * t / 2.0
    elif (updown == "down"):
        f = f1 - k * t / 2.0
    arg = (2.0 * pi * f * t) + phase
    return cos(arg) * amp

# Real version of chirp tone generator
def chirp_sin(f0=F0, f1=F1, T=TIME_FRAME, amp=AMPLITUDE, updown="up", phase=-pi/2.0):
    t = linspace(0, T, int(T * Fs))
    k = float(f1 - f0)/float(T)
    if (updown == "up"):
        f = f0 + k * t / 2.0
    elif (updown == "down"):
        f = f1 - k * t / 2.0
    arg = (2.0 * pi * f * t) + phase
    return sin(arg) * amp

# White noise generator
def white_noise(T=TIME_FRAME, amp=AMPLITUDE):
    a = random.random(int(T * Fs)) * 2 * amp - amp
    b = random.random(int(T * Fs)) * 2 * amp - amp
    return a + 1j * b

# Constant noise generator
def constant_noise(f=0, T=TIME_FRAME, amp=AMPLITUDE, phase=-pi/2.0):
    t = linspace(0, self.T, int(T * Fs))
    arg = (2 * pi * f * t) + phase
    return cos(arg) * amp

# Plot FFT frequency domain
def plot_fft(wave, thres=0.95, logscale=False):
    y = fftshift(fft(wave))
    freq = fftshift(fftfreq(len(y), 1/Fs))

    a = abs(y)
    if logscale:
        plt.plot(freq, 10 * log10(a))
        plt.ylabel("Magnitude(dB)")
    else:
        plt.plot(freq, a)
        plt.ylabel("Magnitude")
    plt.xlabel("Frequency(Hz)")
    plt.title("Frequency domain")

    freq = fftshift(fftfreq(len(y), 1/Fs))
    print("Frequencies at peaks: {} Hz".format(freq[peakutils.indexes(a, thres=thres)]))

# Plot spectrogram
def plot_spectrogram(wave, nperseg, band):
    f, t, Sxx = spectrogram(real(wave), nperseg=nperseg, fs=Fs)
    plt.pcolormesh(t, f[:band], Sxx[:band])
    plt.ylabel('Frequency [Hz]')
    plt.xlabel('Time [sec]')
    plt.title("Spectrogram")

# Plot wave
def plot_wave(wave, real_only=True, logscale=False):
    t = linspace(0, len(wave)/Fs, len(wave))

    re = real(wave)
    im = imag(wave)
    if logscale:
        re = 10 * log10(re)
        im = 10 * log10(im)
        plt.ylabel('Amplitude(dB)')
    else:
        plt.ylabel('Amplitude')

    if real_only:
        plt.plot(t, re)
    else:
        plt.plot(t, re)
        plt.plot(t, im)

    plt.xlabel('Time [sec]')
    plt.title("Time domain")

# Add time delay to chirp signal
def add_delay(chirp, delay_rate=0.0):
    l = len(chirp)
    la = int(l * delay_rate)
    a = zeros(la)
    b = zeros(2 * l - (l + la))
    return append(append(a, chirp), b)

# Playback chirp signal as tone
def play(wave):
    write(WAVE_FILE, Fs, real(wave).astype(int16))
    display(Audio('./' + WAVE_FILE))

# Low pass filter
def lpf(f, cutoff):
    WP = float(cutoff)/float(Fs/2)
    WS = 1.3 * WP
    N, Wn = buttord(wp=WP, ws=WS, gpass=2, gstop=30, analog=0)
    b, a = butter(N, Wn, btype='low', analog=0, output='ba')
    g = lfilter(b, a, f)
    return g