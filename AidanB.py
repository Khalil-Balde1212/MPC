import serial
import sys
import struct
import matplotlib.pyplot as plt

x = [0]
y = [0]

if sys.platform.startswith('win'):
    ser = serial.Serial('COM3', 9600, timeout=None)
else:
    ser = serial.Serial('ttyUSB0', 9600, timeout=None)

plt.ion()

graph = plt.plot(x, y)[0]

print("Starting data collection...")
ser.write(b'255')  # Send a signal to the Arduino to start sending data
values = []


while True:
    data = ser.read(4)
    ser.flushInput()
    if len(data) == 4:
        value = struct.unpack('<f', data)[0]
        y.append(value)
        x.append(x[-1]+1)

    graph.remove()

    graph = plt.plot(x, y, color='g')[0]
    plt.xlim(x[0], x[-1])

    plt.pause(0.001)
