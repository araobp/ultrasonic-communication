# author: <https://github.com/araobp>
#
# I like Object Oriented Programming so much
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

WAVE_FILE="./ChirpTone.wav"

class Signal:

    def __init__(self, fs, f0, f1, T, A):
        self.fs = fs  # sampling rate
        self.f0 = f0  # start of sweep range
        self.f1 = f1  # end of sweep range
        self.T = T    # period
        self.A = A    # amplitude

    def chirp(self, updown="up", phase=-pi/2.0):
        t = linspace(0, self.T, int(self.T * self.fs))
        k = float(self.f1 - self.f0)/float(self.T)
        if (updown == "up"):
            f = self.f0 + k * t / 2.0
        elif (updown == "down"):
            f = self.f1 - k * t / 2.0
        arg = (2.0 * pi * f * t) + phase
        return exp(1j * arg) * self.A

    def chirp_cos(self, updown="up", phase=-pi/2.0):
        return real(self.chirp(updown, phase))

    def chirp_sin(self, updown="up", phase=-pi/2.0):
        return imag(self.chirp(updown, phase))
    
    def chirp_orth(self, updown="up", phase=-pi/2.0):
        t = linspace(0, self.T, int(self.T * self.fs))
        k = float(self.f1 - self.f0)/float(self.T)
        if (updown == "up"):
            f = self.f0 + k * t / 2.0
        elif (updown == "down"):
            f = self.f1 - k * t / 2.0
        arg = (2.0 * pi * f * t) + phase
        return ( cos(arg) + sin(arg) ) * self.A
    
    def silence(self):
        return zeros(int(self.T * self.fs))

    def white_noise(self, A):
        if A == None:
            A = self.A
        a = random.random(int(self.T * self.fs)) * 2 * A - A
        b = random.random(int(self.T * self.fs)) * 2 * A - A
        return a + 1j * b

    def constant_noise(self, f, A):
        if A == None:
            A = self.A
        t = linspace(0, self.T, int(self.T * self.fs))
        arg = 2 * pi * f * t
        return cos(arg) * A

    def plot_fft(self, wave, thres=0.95, logscale=False):
        y = fftshift(fft(wave))
        freq = fftshift(fftfreq(len(y), 1/self.fs))

        a = abs(y)
        if logscale:
            plt.plot(freq, 10 * log10(a))
            plt.ylabel("Magnitude(dB)")
        else:
            plt.plot(freq, a)
            plt.ylabel("Magnitude")
        plt.xlabel("Frequency(Hz)")
        plt.title("Frequency domain")

        print("Frequencies at peaks: {} Hz".format(freq[peakutils.indexes(a, thres=thres)]))

    def plot_spectrogram(self, wave, band, nperseg=256):
        f, t, Sxx = spectrogram(real(wave), nperseg=nperseg, fs=self.fs)
        plt.pcolormesh(t, f[:band], Sxx[:band])
        plt.ylabel('Frequency [Hz]')
        plt.xlabel('Time [sec]')
        plt.title("Spectrogram")

    def plot_wave(self, wave, real_only=False, logscale=False):
        t = linspace(0, len(wave)/self.fs, len(wave))

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

    def lpf(self, f, cutoff):
        WP = float(cutoff)/float(self.fs/2)
        WS = 1.3 * WP
        N, Wn = buttord(wp=WP, ws=WS, gpass=2, gstop=30, analog=0)
        b, a = butter(N, Wn, btype='low', analog=0, output='ba')
        g = lfilter(b, a, f)
        return g

    def play(self, wave, wave_file=WAVE_FILE):
        write(wave_file, self.fs, real(wave).astype(int16))
        display(Audio('./' + wave_file))

def add_delay(chirp, delay_rate=0.0):
    l = len(chirp)
    la = int(l * delay_rate)
    a = zeros(la)
    b = zeros(2 * l - (l + la))
    return append(append(a, chirp), b)

def time_shift(chirp, shift_rate=0.0):
    l = len(chirp)
    t = int(l * shift_rate)
    return append(chirp[t:], chirp[:t])