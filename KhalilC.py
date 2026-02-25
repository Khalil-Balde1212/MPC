from libs.simPlant import SimPlant
import simpy
import numpy as np


class PIDController:
    """
    PID Controller implementation as a class-based object.
    
    This class encapsulates all PID functionality including tuning,
    control execution, and output data collection.
    """
    
    def __init__(self, plant, kp=1.0, ki=0.0, kd=0.0, dt=0.01):
        """
        Initialize the PID Controller.
        
        Args:
            plant: The plant/system to control
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            dt: Sampling time
        """
        self.plant = plant
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        
        # Internal state variables
        self.perror = 0.0
        self.ierror = 0.0
        self.derror = 0.0
        self.previous_error = 0.0
        
        # Control limits
        self.output_min = 0.0
        self.output_max = 2.0
        
        # Data collection variables
        self.pid_output = []
        self.pid_control = []
        self.pid_time = []
        self.pid_error = []
        
    def run_pid(self, env, setpoint):
        """
        Run the PID control algorithm.
        
        Args:
            env: SimPy environment
            setpoint: Array of setpoint values over time
        """
        time_step = 0
        
        print("PID Controller Parameters:")
        print(f"Kp = {self.kp}, Ki = {self.ki}, Kd = {self.kd}")
        print(f"Sample Time (dt) = {self.dt}")
        print(f"Control Limits: [{self.output_min}, {self.output_max}]")
        
        while True:
            # Get current setpoint
            setpoint_idx = min(time_step, len(setpoint) - 1)
            target_setpoint = setpoint[setpoint_idx]
            
            # Get current measurement
            measurement = self.plant.y
            
            # Calculate error
            self.perror = target_setpoint - measurement
            
            
            self.ierror += self.perror * self.dt
            self.derror = (self.perror - self.previous_error) / self.dt if self.dt > 0 else 0.0
            
            # Calculate control signal
            control_signal = self.kp * self.perror + self.ki * self.ierror + self.kd * self.derror
            
            # Apply control limits
            control_signal = np.clip(control_signal, self.output_min, self.output_max)
            
            # Apply control signal to plant
            self.plant.set_input(float(control_signal))
            
            # Store previous error for derivative calculation
            self.previous_error = self.perror
            
            time_step += 1
            yield env.timeout(self.plant.dt)
    
    def collect_output(self, env, duration):
        """
        Collect PID output data for plotting and analysis.
        
        Args:
            env: SimPy environment
            duration: Duration to collect data (seconds)
        """
        while env.now < duration:
            # Ensure we always append a scalar value
            y_value = self.plant.y
            if hasattr(y_value, '__iter__') and not isinstance(y_value, str):
                y_value = float(y_value[0]) if len(y_value) > 0 else 0.0
            else:
                y_value = float(y_value)
                
            self.pid_output.append(y_value)
            self.pid_time.append(env.now)
            self.pid_control.append(float(self.plant.u))
            
            yield env.timeout(self.plant.dt)
    
    def reset(self):
        """Reset the controller state for a new simulation."""
        self.ierror = 0.0
        self.previous_error = 0.0
        self.pid_output = []
        self.pid_control = []
        self.pid_time = []
        self.pid_error = []
        self.plant.reset()
    
    def get_results(self):
        """
        Get the collected PID results.
        
        Returns:
            dict: Dictionary containing time, output, and control data
        """
        return {
            'time': self.pid_time.copy(),
            'output': self.pid_output.copy(),
            'control': self.pid_control.copy()
        }