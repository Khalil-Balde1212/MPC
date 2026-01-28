import serial
import time
import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial('COM3', 9600, timeout=1)

read_from_serial = lambda: readings.append(ser.readline().decode('utf-8').strip())
readings = []
dt = 0.05  # time interval between readings


if __name__ == "__main__":
    # Collect data for a specific duration
    duration = 5  # seconds
    start_time = time.time()
    print("Starting data collection...")
    ser.write(b'\xff')

    while time.time() - start_time < duration:
        read_from_serial()
        time.sleep(dt)

    ser.write(b'\x01')
    print("Data collection complete.")
    ser.close()

    for reading in readings:
        if reading >= 5000 * 0.632:  # time constant
            print("Time constant reached at reading:", reading)
            break
        
    # Convert readings to numeric values
    try:
        values = [float(x) for x in readings if x]
        time_axis = [i * dt for i in range(len(values))]
        
        # Plot the data
        plt.figure(figsize=(10, 6))
        plt.plot(time_axis, values, label='Data')
        plt.xlabel('Time (s)')
        plt.ylabel('Reading')
        plt.title('Serial Data')
        plt.legend()
        plt.grid(True)
        plt.show()
    except ValueError:
        print("Error: Could not convert readings to numeric values")