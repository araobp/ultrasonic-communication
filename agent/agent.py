import serial
import time

MODE = b'2'
INTERVAL = 2.5

ser = serial.Serial('COM15', 115200)

HEADER = 'HEADER'
FFT = 'FFT'
mode = HEADER

mic = ''
sampling_rate = 0

filename_fft = ''

file_fft = None

def param(line):
    return line.split(':')[1][1:].rstrip()

def print_mode():
    print('mode = {}'.format(mode))

if __name__ == '__main__':

    print_mode()
    ser.write(MODE)

    while True:

        line = ser.readline().decode('ascii')
        #print(line, end='')

        if (mode == HEADER):
            if (line.startswith('MEMS mic')):
                mic = param(line)
                print('MEMS mic: {}'.format(mic))

            if (line.startswith('Sampling rate')):
                sampling_rate = param(line)
                print('Sampling rate: {}'.format(sampling_rate))

            if (line.startswith('index')):
                mode = FFT
                print_mode()
                file_fft = open('out.fft', 'w')
                file_fft.write(line)

        elif (mode == FFT):
            if (line == 'EOF\n'):
                mode = HEADER
                print_mode()
                file_fft.close()
                time.sleep(INTERVAL)
                ser.write(MODE)
            else:
               file_fft.write(line)
