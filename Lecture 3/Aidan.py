import time
import matplotlib.pyplot as plt
from plant_interface import initialize_connection, read_from_serial, send_to_serial, cleanup, is_using_serial

if __name__ == "__main__":
    # Initialize connection
    initialize_connection()
    
    # Collect data for a specific duration
    duration = 5  # seconds
    readings = []
    dt = 0.02  # time interval between readings
    
    print("Starting data collection...")
    
    try:
        
        start_time = time.time()
        
        send_to_serial(255)
        while time.time() - start_time < duration:
            value = read_from_serial()
            if value is not None:
                readings.append(value)
            time.sleep(dt)
            
        
        # Send stop command (1 = minimal speed / 0x01)
        send_to_serial(0)
        
        print("Data collection complete.")
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        cleanup()
    
    # Check for time constant (63.2% of final value)
    if readings:
        final_value = 5000  # kp * u = 1000 * 5.0
        target = final_value * 0.632
        for i, reading in enumerate(readings):
            if reading >= target:
                print(f"Time constant reached at {i * dt:.2f} seconds, value: {reading:.1f}")
                break
    
    # Convert readings to plot
    try:
        values = [float(x) for x in readings if x]
        time_axis = [i * dt for i in range(len(values))]
        
        # Plot the data
        plt.figure(figsize=(10, 6))
        plt.plot(time_axis, values, label='Plant Response')
        plt.axhline(y=5000 * 0.632, color='r', linestyle='--', label='63.2% of steady state')
        plt.xlabel('Time (s)')
        plt.ylabel('Reading (RPM)')
        plt.title('plant data')
        plt.legend()
        plt.grid(True)
        plt.show()
    except ValueError:
        print("Error: Could not convert readings to numeric values")
    