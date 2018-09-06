# DSP library

import matplotlib
matplotlib.rcParams['figure.figsize'] = [12, 4]
LINE_COLOR='lightslategray'
LINE_COLOR2='burlywood'

import numpy as np
import scipy.signal as sg
import matplotlib.pyplot as plt

from scipy.fftpack import fft, ifft, fftfreq, fftshift
from scipy.io.wavfile import write
import peakutils

import scipy.io.wavfile as wv
from IPython.display import display, Audio

def plot_fft(fs, wave, thres=0.95, logscale=False, color=LINE_COLOR):
    y = fftshift(fft(wave))
    freq = fftshift(fftfreq(len(y), 1/fs))

    a = np.abs(y)
    if logscale:
        plt.plot(freq, 20 * np.log10(a), color=color)
        plt.ylabel("Magnitude(dB)")
    else:
        plt.plot(freq, a, color=color)
        plt.ylabel("Magnitude")
    plt.xlabel("Frequency(Hz)")
    plt.title("Frequency domain")

    print("Frequencies at peaks: {} Hz".format(freq[peakutils.indexes(a, thres=thres)]))
        
def plot_spectrogram(fs, wave, band=1, nperseg=256, logscale=False, cmap='summer', mode='magnitude'):
    f, t, Sxx = sg.spectrogram(np.real(wave), nperseg=nperseg, fs=fs, mode=mode)
    band = int(len(f) * band)
    if logscale:
        plt.pcolormesh(t, f[:band], np.log10(Sxx[:band]), cmap=cmap)
    else:
        plt.pcolormesh(t, f[:band], Sxx[:band], cmap=cmap)
    plt.ylabel('Frequency [Hz]')
    plt.xlabel('Time [sec]')
    plt.title("Spectrogram")

def plot_wave(fs, wave, real_only=False, logscale=False, color=LINE_COLOR, color2=LINE_COLOR2):
    t = np.linspace(0, len(wave)/fs, len(wave))

    re = np.real(wave)
    im = np.imag(wave)
    if logscale:
        re = 20 * np.log10(re)
        im = 20 * np.log10(im)
        plt.ylabel('Amplitude(dB)')
    else:
        plt.ylabel('Amplitude')

    if real_only:
        plt.plot(t, re, color=color)
    else:
        plt.plot(t, re, color=color)
        plt.plot(t, im, color=color2)

    plt.xlabel('Time [sec]')
    plt.title("Time domain")

def cut(fs, wave, t0, t1):
    a = int(t0*fs)
    b = int(t1*fs)
    return(wave[a:b])

def add_delay(fs, wave, delay=0.0, transmitter=False):
    la = int(delay * fs)
    law = len(wave) + la
    a = np.zeros(la)
    b = np.zeros(law)
    if transmitter:
        w = np.append(np.append(wave, a), b)
    else:
        w = np.append(np.append(a, wave), b)
    return w

def buffer(fs, length, wave):
    l = int(fs * length)
    buf = np.zeros(l)
    buf[:len(wave)] = wave
    return buf

def time_shift(signal, shift_ratio=0.0):
    l = len(chirp)
    t = int(l * shift_rate)
    return np.append(chirp[t:], chirp[:t])

def read(filename):
    return wv.read(filename)

def play(*args, **kwargs):
    display(Audio(*args, **kwargs))

def lpf(fs, wave, cutoff):
    WP = float(cutoff)/float(fs/2)
    WS = 1.3 * WP
    N, Wn = sg.buttord(wp=WP, ws=WS, gpass=2, gstop=30, analog=0)
    b, a = sg.butter(N, Wn, btype='low', analog=0, output='ba')
    g = sg.lfilter(b, a, wave)
    return g

class Chirp:

    def __init__(self, fs, f0, f1, T=1.0, A=1.0):
        self.fs = fs
        self.f0 = f0
        self.f1 = f1
        self.T = T
        self.A = A

    def chirp(self, updown="up", phase=-np.pi/2.0):
        t = np.linspace(0, self.T, int(self.T * self.fs))
        k = float(self.f1 - self.f0)/float(self.T)
        if (updown == "up"):
            f = self.f0 + k * t / 2.0
        elif (updown == "down"):
            f = self.f1 - k * t / 2.0
        arg = (2.0 * np.pi * f * t) + phase
        return np.exp(1j * arg) * self.A

    def chirp_cos(self, updown="up", phase=-np.pi/2.0):
        return np.real(self.chirp(updown, phase))

    def chirp_sin(self, updown="up", phase=-np.pi/2.0):
        return np.imag(self.chirp(updown, phase))
    
    def silence(self):
        return np.zeros(int(self.T * self.fs))

    def white_noise(self, A):
        if A == None:
            A = self.A
        a = np.random.random(int(self.T * self.fs)) * 2 * A - A
        b = np.random.random(int(self.T * self.fs)) * 2 * A - A
        return a + 1j * b

    def constant_noise(self, f, A):
        if A == None:
            A = self.A
        t = np.linspace(0, self.T, int(self.T * self.fs))
        arg = 2 * np.pi * f * t
        return cos(arg) * A
    
    def carrier_IQ(self, iq, fc, phase=-np.pi/2.0):
        t = np.linspace(0, self.T, int(self.T * self.fs))
        if iq == 'I':
            return np.cos(2 * np.pi * fc * t + phase)
        elif iq == 'Q':
            return np.sin(2 * np.pi * fc * t + phase)

    def chirp_x_carrier(self, fc, updown="up"):
        t = np.linspace(0, self.T, int(self.T * self.fs))
        k = float(self.f1-self.f0)/float(self.T)
        if (updown == "up"):
            f = self.f0 + k * t / 2.0
        elif (updown == "down"):
            f = self.f1 - k * t / 2.0
        arg = 2 * np.pi * (fc - f) * t
        return np.cos(arg) * self.A
