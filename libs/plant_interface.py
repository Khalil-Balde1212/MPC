import time
import threading
import os
import sys

# Add libs folder to path for imports
sys.path.insert(0, os.path.dirname(__file__))

import simpy
from simPlant import SimPlant
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Try to import serial
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Serial library not available, will use simulation")

# Global state
_plant_thread = None
_plot_thread = None
_running = False
_env = None
_sim_plant = None
_ser = None
_use_serial = False
_start_time = None
_last_reading = None
_pending_command = None
_command_lock = threading.Lock()
_reading_lock = threading.Lock()
_plot_data = []
_plot_lock = threading.Lock()


def _simpy_plant_process(env, plant):
    """SimPy process that steps the plant at regular intervals"""
    global _last_reading, _pending_command, _running, _start_time
    
    while _running:
        # Check for pending commands
        with _command_lock:
            if _pending_command is not None:
                control_value = (_pending_command / 255.0) * 5.0
                plant.set_input(control_value)
                _pending_command = None
        
        # Step the plant
        value = plant.step()
        timestamp = time.time() - _start_time
        
        # Update reading
        with _reading_lock:
            _last_reading = value
        
        # Update plot data
        with _plot_lock:
            _plot_data.append((timestamp, value))
            # Keep last 30 seconds of data
            while _plot_data and _plot_data[0][0] < timestamp - 30:
                _plot_data.pop(0)
        
        yield env.timeout(plant.dt)


def _serial_loop():
    """Thread loop for serial communication"""
    global _last_reading, _pending_command, _running, _start_time, _ser
    
    while _running:
        # Check for pending commands
        with _command_lock:
            if _pending_command is not None:
                _ser.write(bytes([int(_pending_command)]))
                _pending_command = None
        
        # Read from serial
        try:
            data = _ser.readline().decode('utf-8').strip()
            if data:
                value = float(data)
                timestamp = time.time() - _start_time
                
                with _reading_lock:
                    _last_reading = value
                
                with _plot_lock:
                    _plot_data.append((timestamp, value))
                    while _plot_data and _plot_data[0][0] < timestamp - 30:
                        _plot_data.pop(0)
        except (ValueError, Exception):
            pass
        
        time.sleep(0.001)


def _run_simpy_env():
    """Run the SimPy environment in a thread"""
    global _env, _sim_plant, _running
    
    _env = simpy.RealtimeEnvironment(factor=1.0, strict=False)
    _sim_plant = SimPlant(kp=1000, time_constant=0.85, dt=0.01)
    _env.process(_simpy_plant_process(_env, _sim_plant))
    
    try:
        while _running:
            _env.step()
    except simpy.core.EmptySchedule:
        pass


def _plot_loop():
    """Thread that handles live plotting"""
    
    def animate(frame):
        with _plot_lock:
            if _plot_data:
                times = [d[0] for d in _plot_data]
                values = [d[1] for d in _plot_data]
                line.set_data(times, values)
                if times:
                    current_time = times[-1]
                    ax.set_xlim(max(0, current_time - 20), current_time + 1)
        return line,

    def init_plot():
        line.set_data([], [])
        return line,

    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-', linewidth=2)
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 6000)
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Value')
    ax.set_title('Real-time Plant Data (SimPy)')
    ax.grid(True)

    ani = animation.FuncAnimation(fig, animate, init_func=init_plot,
                                   blit=False, interval=50, cache_frame_data=False)
    plt.show()

def initialize_connection():
    """Initialize connection and start plant with SimPy and live plot"""
    global _plant_thread, _plot_thread, _running, _start_time, _use_serial, _ser
    
    if _running:
        print("Connection already initialized")
        return
    
    _running = True
    _start_time = time.time()
    
    # Try to connect to real serial
    if SERIAL_AVAILABLE:
        try:
            print("Attempting to connect to serial port...")
            for port in ['COM3', '/dev/ttyACM0', '/dev/ttyACM1']:
                try:
                    _ser = serial.Serial(port, 9600, timeout=0.05)
                    print(f"Connected to serial port {port}")
                    _use_serial = True
                    break
                except (serial.SerialException, FileNotFoundError) as e:
                    print(f"Failed to connect to {port}: {e}")
                    continue
        except Exception as e:
            print(f"Error during serial connection: {e}")
    
    # Start appropriate thread based on connection type
    if _use_serial:
        print("Using serial connection")
        _plant_thread = threading.Thread(target=_serial_loop, daemon=True)
    else:
        print("Using SimPy simulated plant")
        _plant_thread = threading.Thread(target=_run_simpy_env, daemon=True)
    
    _plant_thread.start()
    
    # Start plot thread (runs matplotlib in main-compatible way)
    _plot_thread = threading.Thread(target=_plot_loop, daemon=True)
    _plot_thread.start()
    
    print("Plant and live plot started in background")
    time.sleep(0.5)

def send_to_serial(value):
    """Send command to the plant
    Args:
        value: Control input (0-255)
    """
    global _pending_command
    
    if not _running:
        print("Error: Call initialize_connection() first")
        return
    
    with _command_lock:
        _pending_command = value


def read_from_serial(latest=True):
    """Read current value from the plant
    Args:
        latest: If True, return most recent value. If False, same behavior (no queue now).
    Returns:
        Current plant output value
    """
    global _last_reading
    
    if not _running:
        print("Error: Call initialize_connection() first")
        return None
    
    with _reading_lock:
        return _last_reading


def cleanup():
    """Clean up and stop background threads"""
    global _running, _plant_thread, _plot_thread, _ser, _env, _sim_plant
    global _last_reading, _pending_command, _use_serial, _plot_data
    
    _running = False
    
    if _plant_thread and _plant_thread.is_alive():
        print("Stopping plant thread...")
        _plant_thread.join(timeout=2)
    
    if _ser:
        _ser.close()
        _ser = None
    
    _plant_thread = None
    _plot_thread = None
    _env = None
    _sim_plant = None
    _last_reading = None
    _pending_command = None
    _use_serial = False
    _plot_data.clear()
    
    print("Cleanup complete")


def is_using_serial():
    """Check if using real serial"""
    return _use_serial
