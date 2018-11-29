import serial
import time

CMD = b'2'
INTERVAL = 2.5

HEADE = 'header'
FFT = 'fft'

ser = serial.Serial('COM15', 115200)
mode = HEADER

file_fft = None

if __name__ == '__main__':

    ser.write(CMD)

    while True:

        line = ser.readline().decode('ascii')

        if (line.startswith('start')):
            print('{}'.format(line))
        if (line.startswith('count')):
                mode = FFT
                file_fft = open('out.csv', 'w')
                file_fft.write(line)
        elif (mode == FFT):
            if (line == 'EOF\n'):
                mode = HEADER
                file_fft.close()
                time.sleep(INTERVAL)
                ser.write(CMD)
            else:
                file_fft.write(line)
