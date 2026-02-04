import sys
import os

from libs.plant_interface import initialize_connection, send_to_serial, read_from_serial, cleanup
from time import time, sleep
import matplotlib.pyplot as plt

if __name__ == "__main__":
    initialize_connection()

    # Collect data for a specific duration
    duration = 5  # seconds
    delay = 2
    dt = 0.05  # time interval between readings
    readings = []

    start_time = time()
    print("Starting data collection...")

    last_sample = start_time
    while time() - start_time < duration:
        current = time()
        if(current - start_time) >= delay:
            send_to_serial(255)  # Step input after 2 seconds
        if current - last_sample >= dt:
            readings.append(read_from_serial(latest=False))  # Get sequential values
            last_sample += dt  # Use += to prevent drift
        sleep(0.001)  # Small sleep to avoid busy-waiting


    send_to_serial(0)
    for i in range(5):
        send_to_serial(0)
        sleep(0.1)

    print("Data collection complete.")
    cleanup()
    
    for reading in readings:
        if reading is not None:
            if reading > 5000*0.632:
                print(f"time constant is approximately: {readings.index(reading)*dt-delay} seconds")
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

        # print(readings)
    except ValueError:
        print("Error: Could not convert readings to numeric values")