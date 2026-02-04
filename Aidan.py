import serial
import time
import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial('COM3', 9600, timeout=1)

read_from_serial = lambda: ser.readline().decode('utf-8').strip()

dt = 0.05  # time interval between readings

if __name__ == "__main__":
    readings = []
    duration = 5  # seconds

    start_time = time.time()
    print("Starting data collection...")
    ser.write(b'\xff')

    kp, ki, kd = 1.0, 0.0, 0.0
    perror, ierror, derror = 0.0, 0.0, 0.0
    setpoint = 1000
    while time.time() - start_time < duration:
        reading = read_from_serial()
        readings.append(reading)

        perror = setpoint - float(reading) if reading else perror
        ierror += perror * dt
        derror = (perror - derror) / dt if dt > 0 else 0.0

        control_signal = kp * perror + ki * ierror + kd * derror
        control_signal = max(0, min(255, int(control_signal)))

        ser.write(bytes([control_signal]))
        
        time.sleep(dt)

    for _ in range(5):
        ser.write(b'\x00')
        time.sleep(dt)
    print("Data collection complete.")