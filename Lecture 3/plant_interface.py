import time
from simPlant import SimPlant
from multiprocessing import Process, Queue
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Try to import serial
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Serial library not available, will use simulation")

# Global state - plant and plot are separate processes
plant_process = None
plot_process = None
command_queue = None  # Send commands to plant
reading_queue = None  # Receive readings from plant (values only)
plot_queue = None  # Receive readings for plotting (timestamp, value)
last_reading = None  # Cache the last reading to avoid None returns

def _plant_process(cmd_queue, read_queue, plot_queue):
    """Background process that runs the plant and streams readings"""
    use_serial = False
    ser = None
    sim_plant = None
    last_sample_time = time.time()
    start_time = time.time()

    def try_put_latest(q, item):
        """Put item without blocking; if full, drop oldest."""
        try:
            q.put_nowait(item)
        except Exception:
            try:
                q.get_nowait()
            except Exception:
                pass
            try:
                q.put_nowait(item)
            except Exception:
                pass

    # Try to connect to real serial
    if SERIAL_AVAILABLE:
        try:
            print("Attempting to connect to serial port...")
            for port in ['COM3', '/dev/ttyACM0', '/dev/ttyACM1']:
                try:
                    ser = serial.Serial(port, 9600, timeout=0.05)
                    print(f"Connected to serial port {port}")
                    use_serial = True
                    break
                except (serial.SerialException, FileNotFoundError) as e:
                    print(f"Failed to connect to {port}: {e}")
                    continue
        except Exception as e:
            print(f"Error during serial connection: {e}")

    # Use simulated plant if serial not available
    if not use_serial:
        print("Using simulated plant")
        sim_plant = SimPlant(kp=1000, time_constant=0.85, dt=0.01)
        last_sample_time = time.time()

    def read_value():
        """Read from the plant (serial or simulated)"""
        nonlocal last_sample_time
        if use_serial:
            data = ser.readline().decode('utf-8').strip()
            if data:
                try:
                    return float(data)
                except ValueError:
                    return None
        else:
            current_time = time.time()
            elapsed = current_time - last_sample_time
            last_sample_time = current_time
            num_steps = max(1, int(elapsed / sim_plant.dt))
            value = None
            for _ in range(num_steps):
                value = sim_plant.step()
            return value
        return None

    def send_value(value):
        """Send command to the plant (serial or simulated)"""
        if use_serial:
            ser.write(bytes([int(value)]))
        else:
            control_value = (value / 255.0) * 5.0
            sim_plant.set_input(control_value)

    # Main loop
    while True:
        # Process commands from main process
        while not cmd_queue.empty():
            try:
                cmd_value = cmd_queue.get_nowait()
                send_value(cmd_value)
            except Exception:
                pass

        # Read from plant
        value = read_value()
        if value is not None:
            timestamp = time.time() - start_time
            try_put_latest(read_queue, value)
            try_put_latest(plot_queue, (timestamp, value))

        time.sleep(0.001)


def _plot_process(plot_queue):
    """Background process that only plots readings"""
    data = []
    time_data = []

    def init_plot():
        line.set_data([], [])
        return line,

    def animate(frame):
        # Drain plot queue
        while not plot_queue.empty():
            try:
                t, value = plot_queue.get_nowait()
                time_data.append(t)
                data.append(value)
            except Exception:
                break

        # Keep last 20 seconds
        while time_data and time_data[0] < time_data[-1] - 20:
            data.pop(0)
            time_data.pop(0)

        # Update plot
        line.set_data(time_data, data)
        if time_data:
            current_time = time_data[-1]
            ax.set_xlim(max(0, current_time - 20), current_time + 1)

        return line,

    # Setup plot
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-', linewidth=2)
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 6000)
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Value')
    ax.set_title('Real-time Plant Data')
    ax.grid(True)

    ani = animation.FuncAnimation(fig, animate, init_func=init_plot,
                                   blit=False, interval=10, cache_frame_data=False)
    plt.show()

def initialize_connection():
    """Initialize connection and start plant and live plot in background"""
    global plant_process, plot_process, command_queue, reading_queue, plot_queue
    
    if (plant_process is not None and plant_process.is_alive()) or (
        plot_process is not None and plot_process.is_alive()
    ):
        print("Connection already initialized")
        return
    
    # Create communication queues
    command_queue = Queue()
    reading_queue = Queue(maxsize=10)
    plot_queue = Queue(maxsize=2000)

    # Start plant process
    plant_process = Process(target=_plant_process, args=(command_queue, reading_queue, plot_queue))
    plant_process.daemon = True
    plant_process.start()

    # Start plot process
    plot_process = Process(target=_plot_process, args=(plot_queue,))
    plot_process.daemon = True
    plot_process.start()

    print(
        "Plant and live plot started in background "
        f"(plant PID: {plant_process.pid}, plot PID: {plot_process.pid})"
    )
    time.sleep(1)

def send_to_serial(value):
    """Send command to the plant
    Args:
        value: Control input (0-255)
    """
    global command_queue
    
    if command_queue is None:
        print("Error: Call initialize_connection() first")
        return
    
    command_queue.put(value)

def read_from_serial(latest=True):
    """Read current value from the plant
    Args:
        latest: If True, flush queue and get most recent value. If False, get next sequential value.
    Returns:
        Current plant output value (returns last cached value if queue empty)
    """
    global reading_queue, last_reading
    
    if reading_queue is None:
        print("Error: Call initialize_connection() first")
        return None
    
    try:
        if latest:
            # Flush queue and get most recent value (for slow sampling)
            value = None
            while not reading_queue.empty():
                value = reading_queue.get_nowait()
            if value is not None:
                last_reading = value
            return last_reading
        else:
            # Get one value from queue (for sequential reading)
            if not reading_queue.empty():
                value = reading_queue.get_nowait()
                last_reading = value
                return value
            else:
                return last_reading
    except:
        return last_reading

def cleanup():
    """Clean up and stop background processes"""
    global plant_process, plot_process, command_queue, reading_queue, plot_queue

    if plant_process and plant_process.is_alive():
        print("Stopping plant process...")
        plant_process.terminate()
        plant_process.join(timeout=2)

        if plant_process.is_alive():
            plant_process.kill()
            plant_process.join()

        plant_process = None

    if plot_process and plot_process.is_alive():
        print("Stopping plot process...")
        plot_process.terminate()
        plot_process.join(timeout=2)

        if plot_process.is_alive():
            plot_process.kill()
            plot_process.join()

        plot_process = None

    # Clear queues
    command_queue = None
    reading_queue = None
    plot_queue = None
    print("Cleanup complete")

def is_using_serial():
    """Check if using real serial (always returns False for now since we can't easily query the subprocess)"""
    return False
