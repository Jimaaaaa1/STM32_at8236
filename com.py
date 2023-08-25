import serial
import time
ser = serial.Serial('COM5', 115200)

while True:
    for i in range(1000):
        ser.write(b'$speed0@')
        time.sleep(0.001)
    for i in range(1000):
        ser.write(b'$speed9@')
        time.sleep(0.001)