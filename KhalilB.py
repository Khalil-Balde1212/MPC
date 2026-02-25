from libs.simPlant import SimPlant
import simpy
import numpy as np


class DynamicMatrixController:
    """
    Dynamic Matrix Controller (DMC) implementation as a class-based object.
    
    This class encapsulates all MPC functionality including data collection,
    dynamic matrix building, and MPC execution.
    """
    
    def __init__(self, plant, prediction_horizon=50, control_horizon=2, lambda_reg=0.01):
        """
        Initialize the Dynamic Matrix Controller.
        
        Args:
            plant: The plant/system to control
            prediction_horizon: Prediction horizon for the controller
            control_horizon: Number of control moves to calculate
            lambda_reg: Regularization parameter for control effort
        """
        self.plant = plant
        self.control_horizon = control_horizon
        self.lambda_reg = lambda_reg
        self.prediction_horizon = prediction_horizon  # Can be set to a fixed value if desired  
        
        # Internal state variables
        self.dynamic_matrix = None
        self.step_response = []
        self.control_moves = np.zeros(control_horizon)
        self.reference_window = np.zeros(self.prediction_horizon)
        self.delta_u = np.zeros(control_horizon)
        
        # Data collection variables
        self.mpc_output = []
        self.mpc_control = []
        self.mpc_time = []
        
    def collect_step_response(self, env):
        """
        Collect step response data from the plant for system identification.
        
        Args:
            env: SimPy environment
        """
        control_signal = 1.0  # step input of 1 volt
        self.plant.set_input(control_signal)
        
        # Collect data for longer to ensure we reach steady state
        max_time = 8.0  # Allow more time for full response
        start_time = env.now
        
        while env.now - start_time < max_time:
            self.plant.step()
            self.step_response.append(self.plant.y)
            yield env.timeout(self.plant.dt)

            # Check if we've reached steady state (98% of final value)
            if (env.now - start_time > 3.0 and  # Minimum collection time
                self.plant.y >= 0.98 * self.plant.get_steady_state()):
                print(f"Steady state reached at t={env.now:.2f}s, y={self.plant.y:.2f}")
                break
                
        self.build_dynamic_matrix()
        np.savetxt('./dynamic_matrix.csv', self.dynamic_matrix, delimiter=',')
        print("Dynamic matrix saved to './dynamic_matrix.csv'")
        print("Step response length:", len(self.step_response))
        print("Final step response value:", self.step_response[-1])

    def build_dynamic_matrix(self):
        """
        Build the dynamic matrix from collected step response data.
        """
        if not self.step_response:
            raise ValueError("No step response data available. Run collect_data first.")
            
        # Use the length of step response as prediction horizon if not specified
        pred_horizon = min(self.prediction_horizon, len(self.step_response))
        
        self.dynamic_matrix = np.zeros((pred_horizon, self.control_horizon))

        for i in range(pred_horizon):
            for j in range(self.control_horizon):
                if i - j >= 0 and i - j < len(self.step_response):
                    self.dynamic_matrix[i, j] = self.step_response[i - j]

    def run_dmc(self, env, setpoint):
        """
        Run the Dynamic Matrix Control algorithm based on the control loop simulation.
        
        Args:
            env: SimPy environment
            setpoint: Array of setpoint values for the prediction horizon
        """
        if self.dynamic_matrix is None:
            raise ValueError("Dynamic matrix is None; build it before running DMC")
            
        # Initialize variables
        time_step = 0
        alpha = 0.3  # Setpoint conditioning factor
        Min_ContAct = 0.0  # Minimum control action
        Max_ContAct = 2.0  # Maximum control action
        
        # Initialize control and prediction variables
        U = np.zeros(self.control_horizon)  # Current control moves
        Uprev = np.zeros(self.control_horizon)  # Previous control moves
        yhat = np.zeros(self.prediction_horizon)  # Predicted outputs
        conditioned_setpoint = np.zeros(self.prediction_horizon)  # Conditioned setpoint
        
        print("Control Horizon:", self.control_horizon)
        print("Prediction Horizon:", self.prediction_horizon)
        print("Dynamic matrix shape:", self.dynamic_matrix.shape)
        print("Step response final value:", self.step_response[-1] if self.step_response else "None")

        # Precompute DM matrix for efficiency: DM = (A^T*A + λI)^(-1)*A^T
        A = self.dynamic_matrix
        ATA_reg = A.T @ A + self.lambda_reg * np.eye(self.control_horizon)
        DM = np.linalg.solve(ATA_reg, A.T)
        
        while True:
            # Error Collection
            measurement = self.plant.y
            
            phi = measurement - yhat[0] if len(yhat) > 0 else 0  # The missile knows where it is because it know's where it isn't
            yhat = yhat + phi
            
            # Setpoint Conditioning
            conditioned_setpoint[0] = measurement
            
            # Get target setpoint for this time step
            setpoint_idx = min(time_step, len(setpoint) - 1)
            target_setpoint = setpoint[setpoint_idx]
            
            # Apply exponential filter to remaining setpoint elements
            for i in range(1, len(conditioned_setpoint)):
                conditioned_setpoint[i] = alpha * conditioned_setpoint[i] + (1 - alpha) * target_setpoint
            
            # Evaluate Error
            error = conditioned_setpoint - yhat
            
            # Solve for optimal addition to previous control move
            deltaU = DM @ error
            U = Uprev + deltaU
            
            # Limit Control Move
            for i in range(self.control_horizon):
                if U[i] < Min_ContAct:
                    U[i] = Min_ContAct
                if U[i] > Max_ContAct:
                    U[i] = Max_ContAct
                    
            # Re-evaluate delta U after limiting
            deltaU = U - Uprev
            
            # Apply first control move to plant
            self.plant.set_input(float(U[0]))
            
            # Evaluate Prediction: yhat = yhat + Dynamic_Matrix[:, 0] * deltaU[0]
            if len(deltaU) > 0:
                yhat = yhat + self.dynamic_matrix[:, 0] * deltaU[0]
            
            # Update Values
            Uprev = U.copy()
            
            # Move prediction forward: yhat[:-1] = yhat[1:]
            yhat[:-1] = yhat[1:]
            yhat[-1] = yhat[-2] if len(yhat) > 1 else yhat[0]  # Extend last prediction
            
            time_step += 1
            yield env.timeout(self.plant.dt)

    def collect_output(self, env, duration):
        """
        Collect MPC output data for plotting and analysis.
        
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
                
            self.mpc_output.append(y_value)
            self.mpc_time.append(env.now)
            self.mpc_control.append(float(self.plant.u))
            yield env.timeout(self.plant.dt)

    def reset(self):
        """Reset the controller state for a new simulation."""
        self.mpc_output = []
        self.mpc_control = []
        self.mpc_time = []
        self.plant.reset()

    def get_results(self):
        """
        Get the collected MPC results.
        
        Returns:
            dict: Dictionary containing time, output, and control data
        """
        return {
            'time': self.mpc_time.copy(),
            'output': self.mpc_output.copy(),
            'control': self.mpc_control.copy()
        }