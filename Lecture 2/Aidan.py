import serial
import time
import matplotlib.pyplot as plt

ser = serial.Serial('COM3', 9600, timeout=1)

read_from_serial = lambda: readings.append(ser.readline().decode('utf-8').strip())
readings = []

if __name__ == "__main__":
    # Collect data for a specific duration
    duration = 3  # seconds
    start_time = time.time()
    print("Starting data collection...")
    ser.write(b'\x00') # no clue why 0x00 starts the motor but it does

    while time.time() - start_time < duration:
        read_from_serial()
        time.sleep(0.05)

    ser.write(b'\xff') # no clue why 0xff stops the motor but it does
    print("Data collection complete.")
    ser.close()

    # Convert readings to numeric values
    try:
        values = [float(x) for x in readings if x]
        
        # Plot the data
        plt.figure(figsize=(10, 6))
        plt.plot(values)
        plt.xlabel('Sample')
        plt.ylabel('Value')
        plt.title('Serial Data')
        plt.grid(True)
        plt.show()
    except ValueError:
        print("Error: Could not convert readings to numeric values")