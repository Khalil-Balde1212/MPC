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

# Global state - THE ONLY PLANT runs in the animation process
plot_process = None
command_queue = None  # Send commands to plant
reading_queue = None  # Receive readings from plant

def _animation_process(cmd_queue, read_queue):
    """Background process that runs THE ONLY PLANT and plots it"""
    # Local variables for this process
    use_serial = False
    ser = None
    sim_plant = None
    last_sample_time = time.time()
    
    # Try to connect to real serial
    if SERIAL_AVAILABLE:
        try:
            print("Attempting to connect to serial port...")
            for port in ['COM3', '/dev/ttyACM0', '/dev/ttyACM1']:
                try:
                    ser = serial.Serial(port, 9600, timeout=1)
                    print(f"Connected to serial port {port}")
                    use_serial = True
                    break
                except (serial.SerialException, FileNotFoundError):
                    continue
        except Exception:
            pass
    
    # Use simulated plant if serial not available
    if not use_serial:
        print("Using simulated plant")
        sim_plant = SimPlant(kp=1000, time_constant=0.85, dt=0.005)
        last_sample_time = time.time()
    
    # Data storage for plotting
    data = []
    time_data = []
    start_time = time.time()
    
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
    
    def init_plot():
        line.set_data([], [])
        return line,
    
    def animate(frame):
        # Process commands from main process
        while not cmd_queue.empty():
            try:
                cmd_value = cmd_queue.get_nowait()
                send_value(cmd_value)
            except:
                pass
        
        # Read from plant
        value = read_value()
        if value is not None:
            current_time = time.time() - start_time
            data.append(value)
            time_data.append(current_time)
            
            # Send reading back to main process
            if not read_queue.full():
                try:
                    read_queue.put_nowait(value)
                except:
                    pass
        
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
                                   blit=False, interval=50, cache_frame_data=False)
    plt.show()

def initialize_connection():
    """Initialize connection and start live plot in background"""
    global plot_process, command_queue, reading_queue
    
    if plot_process is not None and plot_process.is_alive():
        print("Connection already initialized")
        return
    
    # Create communication queues
    command_queue = Queue()
    reading_queue = Queue(maxsize=10)
    
    # Start background process with THE ONLY PLANT
    plot_process = Process(target=_animation_process, args=(command_queue, reading_queue))
    plot_process.daemon = True
    plot_process.start()
    
    print(f"Plant and live plot started in background (PID: {plot_process.pid})")
    time.sleep(0.5)  # Give it time to initialize

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

def read_from_serial():
    """Read current value from the plant
    Returns:
        Current plant output value, or None if no value available
    """
    global reading_queue
    
    if reading_queue is None:
        print("Error: Call initialize_connection() first")
        return None
    
    try:
        # Get most recent value
        value = None
        while not reading_queue.empty():
            value = reading_queue.get_nowait()
        return value
    except:
        return None

def cleanup():
    """Clean up and stop background process"""
    global plot_process
    
    if plot_process and plot_process.is_alive():
        plot_process.terminate()
        plot_process.join(timeout=1)
        plot_process = None
        print("Stopped")

def is_using_serial():
    """Check if using real serial (always returns False for now since we can't easily query the subprocess)"""
    return False
