import serial
import sys
import struct
import matplotlib.pyplot as plt
import time

x = [0]
y = [0]

if sys.platform.startswith('win'):
    ser = serial.Serial('COM3', 9600, timeout=None)
else:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=None)

plt.ion()

time.sleep(2)  # Wait for Arduino reset
ser.reset_input_buffer()

plt.ion()
fig, ax = plt.subplots()

try:
    graph = plt.plot(x, y)[0]
    print("Starting data collection...")
    value_to_send = 255
    little_endian_bytes = value_to_send.to_bytes(length=1, byteorder='little')
    ser.write(little_endian_bytes)

    while True:
        data = ser.read(4)  # Just read, don't reset buffer!

        if len(data) == 4:
            value = struct.unpack('<f', data)[0]
            print(value)
            y.append(value)
            x.append(x[-1] + 1)

            # More efficient plotting
            graph.set_data(x, y)
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.001)
        else:
            print(f"Warning: got {len(data)} bytes instead of 4")

except KeyboardInterrupt:
    print("\nInterrupted! Closing plot...")

finally:
    value_to_send = 0
    little_endian_bytes = value_to_send.to_bytes(length=1, byteorder='little')
    ser.write(little_endian_bytes)

    ser.close()
    plt.close('all')
    print("Cleanup complete")
