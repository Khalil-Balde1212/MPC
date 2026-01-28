import serial
import time
import matplotlib.pyplot as plt
import numpy as np

import simPlant

run_real = False

control_in = 1.0  # constant control input for simulation

dt = 0.02  # time interval between readings

global ser
sim = simPlant.SimPlant(kp=2, time_constant=0.5)

if (run_real):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

def read_from_serial():
    if run_real:
        return ser.readline().decode('utf-8').strip()
    else:
        return sim.step(control_in, dt)


if __name__ == "__main__":
    plt.ion()
    fig, ax = plt.subplots()
    x_data, y_data = [], []
    line, = ax.plot(x_data, y_data)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Value')
    ax.set_title('Real-time Serial Data')

    start_time = time.time()

    try:
        while True:
            data = read_from_serial()
            if data:
                try:
                    value = float(data)
                    current_time = time.time() - start_time
                    x_data.append(current_time)
                    y_data.append(value)
                    
                    line.set_xdata(x_data)
                    line.set_ydata(y_data)
                    ax.relim()
                    ax.autoscale_view()
                    
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                except ValueError:
                    pass
            
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if run_real:
            ser.close()
        plt.ioff()
        plt.show()
    