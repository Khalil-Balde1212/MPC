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

# Global state for connection
use_serial = False
ser = None
sim_plant = SimPlant(kp=1000, time_constant=0.85, dt=0.005)
last_sample_time = 0.0
plot_process = None
command_queue = None

def initialize_connection(start_plot=True):
    """Initialize connection to either real serial or simulated plant and optionally start live plot
    Args:
        start_plot: If True, automatically start the live plot. Set to False when called from within the plot process.
    """
    global use_serial, ser, sim_plant, last_sample_time
    
    
    if SERIAL_AVAILABLE:
        try:
            print("Attempting to connect to serial port...")
            ports_to_try = ['COM3', '/dev/ttyACM0', '/dev/ttyACM1']
            for port in ports_to_try:
                try:
                    ser = serial.Serial(port, 9600, timeout=1)
                    print(f"Connected to serial port {port}")
                    use_serial = True
                    if start_plot:
                        start_live_plot()
                    return
                except (serial.SerialException, FileNotFoundError):
                    continue
            print("No serial port found, using simulated plant")
        except (serial.SerialException, FileNotFoundError):
            print("Serial connection failed, using simulated plant")
    
    # If serial failed, use simulated plant
    print("Using simulated plant")
    last_sample_time = time.time()
    
    # Start the live plot if requested
    if start_plot:
        start_live_plot()

def read_from_serial():
    """Read a value from serial (real or simulated)"""
    global last_sample_time
    
    if use_serial:
        data = ser.readline().decode('utf-8').strip()
        if data:
            try:
                return float(data)
            except ValueError:
                return None
    else:
        # For simulated plant, step forward in time since last read
        current_time = time.time()
        elapsed = current_time - last_sample_time
        last_sample_time = current_time
        
        # Run multiple small steps for accuracy
        num_steps = max(1, int(elapsed / sim_plant.dt))
        for _ in range(num_steps):
            value = sim_plant.step()
        
        return value
    return None

def send_to_serial(value):
    """Send a command to serial (real or simulated)
    Args:
        value: A number between 0-255 representing the control input
               For real serial: converted to hex byte
               For simulated: scaled to 0.0-5.0 control value
    """
    if use_serial:
        # Convert to byte and send to real serial
        ser.write(bytes([int(value)]))
    else:
        # Scale 0-255 to 0.0-5.0 for simulated plant
        control_value = (value / 255.0) * 5.0
        sim_plant.set_input(control_value)

def cleanup():
    """Clean up connections and processes"""
    global ser, sim_plant
    
    if use_serial and ser:
        ser.close()
    if sim_plant:
        sim_plant = None

def is_using_serial():
    """Check if using real serial or simulation"""
    return use_serial









def _animation_process(cmd_queue):
    """Internal function that runs the animation in a separate process"""
    # Initialize connection in this process (don't start another plot!)
    initialize_connection(start_plot=False)
    
    # Give the plant an initial input so it generates values
    send_to_serial(100)  # Start with some default input
    
    # Data storage
    data = []
    time_data = []
    start_time = time.time()
    
    def init_plot():
        """Initialize the plot"""
        line.set_data([], [])
        return line,

    def animate(frame):
        """Animation function that updates the plot"""
        nonlocal start_time
        
        # Check for commands from main process
        while not cmd_queue.empty():
            try:
                cmd_value = cmd_queue.get_nowait()
                send_to_serial(cmd_value)
                print(f"Received command: {cmd_value}")  # Debug
            except:
                pass
        
        # Read value from serial
        value = read_from_serial()
        if value is not None:
            current_time = time.time() - start_time
            data.append(value)
            time_data.append(current_time)
            if len(data) % 50 == 0:  # Debug every 50 points
                print(f"Data points: {len(data)}, Latest value: {value:.2f}")
        
        # Keep only data from last 20 seconds
        while time_data and time_data[0] < time_data[-1] - 20:
            data.pop(0)
            time_data.pop(0)
        
        # Update line data
        line.set_data(time_data, data)
        
        # Auto-scroll x-axis: show last 20 seconds
        if time_data:
            current_time = time_data[-1]
            ax.set_xlim(max(0, current_time - 10), current_time + 1)
        
        # Keep y-axis fixed (no autoscaling)
        
        return line,
    
    # Setup the plot
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-', linewidth=2)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6000)  # Plant output range: kp * u_max = 1000 * 5V = 5000
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Value')
    ax.set_title('Real-time Serial Data')
    ax.grid(True)
    
    # Create animation (blit=False needed for autoscale to work)
    ani = animation.FuncAnimation(fig, animate, init_func=init_plot, 
                                   blit=False, interval=50, cache_frame_data=False)
    
    plt.show()

def start_live_plot():
    """Start live plotting in a background process"""
    global plot_process, command_queue
    
    if plot_process is not None and plot_process.is_alive():
        print("Live plot already running")
        return
    
    command_queue = Queue()
    plot_process = Process(target=_animation_process, args=(command_queue,))
    plot_process.daemon = True
    plot_process.start()
    print(f"Live plot started in background (PID: {plot_process.pid})")

def send_to_plot_serial(value):
    """Send command to the plot process's serial connection
    Args:
        value: A number between 0-255 representing the control input
    """
    global command_queue
    
    if command_queue is None:
        print("Warning: Live plot not started. Call start_live_plot() first.")
        return
    
    command_queue.put(value)

def stop_live_plot():
    """Stop the live plotting process"""
    global plot_process
    
    if plot_process and plot_process.is_alive():
        plot_process.terminate()
        plot_process.join(timeout=1)
        plot_process = None
        print("Live plot stopped")
