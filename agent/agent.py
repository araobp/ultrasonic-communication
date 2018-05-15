import serial
import time

ser = serial.Serial('COM15', 115200)

HEADER = 'HEADER'
CSV = 'CSV'
RAW = 'RAW'
mode = HEADER

mic = ''
sampling_rate = 0

filename_fft = ''
filename_raw = ''

file_fft = None
file_raw = None

def param(line):
    return line.split(':')[1][1:].rstrip()

if __name__ == '__main__':

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

            if (line.startswith('Frequency(Hz)')):
                mode = CSV
                filename_fft = '{}_{}_{}.fft'.format(
                    str(int(time.time())), sampling_rate, mic
                    )
                file_fft = open(filename_fft, 'w')
                filename_raw = '{}_{}_{}.raw'.format(
                    str(int(time.time())), sampling_rate, mic
                    )
                file_raw = open(filename_raw, 'w')
                
        if (mode == CSV):
            if (line == '\n'):
                mode = RAW
            else:
                file_fft.write(line)

        if (mode == RAW):
            if (line == 'EOF\n'):
                mode = HEADER
                file_fft.close()
                file_raw.close()
            else:
                file_raw.write(line)
