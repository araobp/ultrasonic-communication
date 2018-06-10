import serial
import time

ser = serial.Serial('COM15', 115200)

HEADER = 'HEADER'
FILTER = 'FILTER'
CSV = 'CSV'
RAW = 'RAW'
mode = HEADER

mic = ''
sampling_rate = 0

filename_fft = ''
filename_raw = ''
filename_filter = ''

file_fft = None
file_raw = None
file_filter = None

def param(line):
    return line.split(':')[1][1:].rstrip()

def print_mode():
    print('mode = {}'.format(mode))

if __name__ == '__main__':

    print_mode()

    while True:

        line = ser.readline().decode('ascii')

        if (mode == HEADER):
            if (line.startswith('MEMS mic')):
                mic = param(line)
                print('MEMS mic: {}'.format(mic))

            if (line.startswith('Sampling rate')):
                sampling_rate = param(line)
                print('Sampling rate: {}'.format(sampling_rate))

            if (line.startswith('Frequency(Hz)')):
                mode = CSV
                print_mode()
                filename ='{}_{}_{}'.format(str(int(time.time())),
                                            sampling_rate, mic)
                filename_fft = '{}.fft'.format(filename)
                file_fft = open(filename_fft, 'w')
                filename_raw = '{}.raw'.format(filename)
                file_raw = open(filename_raw, 'w')
                file_fft.write(line)

        elif (mode == CSV):
            if (line == '\n'):
                mode = RAW
                print_mode()
            else:
                file_fft.write(line)

        elif (mode == RAW):
            if (line == 'EORAW\n'):
                mode = HEADER
                print_mode()
                file_fft.close()
                file_raw.close()
            else:
                file_raw.write(line)
