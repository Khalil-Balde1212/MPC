import time
import os
import sys
import struct

# Add libs folder to path for imports
sys.path.insert(0, os.path.dirname(__file__))

from simPlant import SimPlant

# Try to import serial
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Serial library not available, will use simulation")

# Global state
_initialized = False
_sim_plant = None
_ser = None
_use_serial = False
_last_sample_time = None
_last_reading = None


def initialize_connection():
    """Initialize connection to plant (serial or simulated)"""
    global _initialized, _sim_plant, _ser, _use_serial, _last_sample_time
    
    if _initialized:
        print("Connection already initialized")
        return
    
    # Try to connect to real serial
    if SERIAL_AVAILABLE:
        try:
            print("Attempting to connect to serial port...")
            for port in ['COM3', '/dev/ttyACM0', '/dev/ttyACM1']:
                try:
                    _ser = serial.Serial(port, 9600, timeout=1)
                    time.sleep(2)  # Wait for Arduino to reset after serial connect
                    _ser.reset_input_buffer()  # Clear any boot garbage
                    print(f"Connected to serial port {port}")
                    _use_serial = True
                    break
                except (serial.SerialException, FileNotFoundError) as e:
                    print(f"Failed to connect to {port}: {e}")
                    continue
        except Exception as e:
            print(f"Error during serial connection: {e}")
    
    if not _use_serial:
        print("Using simulated plant")
        _sim_plant = SimPlant(kp=1000, time_constant=0.85, dt=0.01)
        _last_sample_time = time.time()
    
    _initialized = True
    print("Plant initialized")


def send_to_serial(value):
    """Send command to the plant
    Args:
        value: Control input (0-255)
    """
    global _sim_plant, _ser
    
    if not _initialized:
        print("Error: Call initialize_connection() first")
        return
    
    if _use_serial:
        _ser.write(bytes([int(value)]))
        _ser.flush()
    else:
        control_value = (value / 255.0) * 5.0
        _sim_plant.set_input(control_value)


def read_from_serial(latest=True):
    """Read current value from the plant
    Args:
        latest: Ignored (kept for API compatibility)
    Returns:
        Current plant output value
    """
    global _last_sample_time, _last_reading
    
    if not _initialized:
        print("Error: Call initialize_connection() first")
        return None
    
    if _use_serial:
        try:
            # Arduino sends RPM as 4-byte float via Serial.write((byte*)&rpm, 4)
            while _ser.in_waiting >= 4:
                raw = _ser.read(4)
                _last_reading = struct.unpack('<f', raw)[0]  # little-endian float
        except (serial.SerialException, OSError, struct.error) as e:
            print(f"Serial read error: {e}")
        return _last_reading
    else:
        # Step simulation based on elapsed time
        current_time = time.time()
        elapsed = current_time - _last_sample_time
        _last_sample_time = current_time
        
        num_steps = max(1, int(elapsed / _sim_plant.dt))
        for _ in range(num_steps):
            _last_reading = _sim_plant.step()
        
        return _last_reading


def cleanup():
    """Clean up and close connections"""
    global _initialized, _ser, _sim_plant, _use_serial, _last_reading, _last_sample_time
    
    if _ser:
        _ser.close()
        _ser = None
    
    _sim_plant = None
    _use_serial = False
    _last_reading = None
    _last_sample_time = None
    _initialized = False
    
    print("Cleanup complete")


def is_using_serial():
    """Check if using real serial"""
    return _use_serial
