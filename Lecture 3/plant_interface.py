import time
from simPlant import SimPlant

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
sim_plant = None
last_sample_time = None

def initialize_connection():
    """Initialize connection to either real serial or simulated plant"""
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
                    return
                except (serial.SerialException, FileNotFoundError):
                    continue
            print("No serial port found, using simulated plant")
        except (serial.SerialException, FileNotFoundError):
            print("Serial connection failed, using simulated plant")
    
    # If serial failed, use simulated plant
    print("Using simulated plant")
    sim_plant = SimPlant(kp=1000, time_constant=0.85, dt=0.005)
    last_sample_time = time.time()

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
