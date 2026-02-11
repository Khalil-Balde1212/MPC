from libs.simPlant import SimPlant
import simpy

import numpy as np
step_response = []
dynamicMatrix = None

plant = SimPlant(kp=1000, time_constant=0.8, dt=0.01)

def collect_data(env, plant, duration):
    global step_response, dynamicMatrix
    control_signal = 1.0  # step input of 1 volt
    plant.set_input(control_signal)
    
    while env.now < duration:
        plant.step()
        step_response.append(plant.y)
        yield env.timeout(plant.dt)

        if plant.y >= 0.95 * plant.get_steady_state():
            dynamicMatrix = build_dynamic_matrix(step_response, len(step_response), 10)
            np.savetxt('./dynamic_matrix.csv', dynamicMatrix, delimiter=',')
            print("Dynamic matrix saved to './dynamic_matrix.csv'")
            print(dynamicMatrix)
            break


def build_dynamic_matrix(step_response, prediction_horizon, control_horizon):
    import numpy as np
    dynamic_matrix = np.zeros((prediction_horizon, control_horizon))

    for i in range(prediction_horizon):
        for j in range(control_horizon):
            if i - j >= 0 and i - j < len(step_response):
                dynamic_matrix[i, j] = step_response[i - j]

    return dynamic_matrix



def RunDMC(env, plant, dynamicMatrix, setpoint, lambda_reg=0.01):
    if dynamicMatrix is None:
        raise ValueError("dynamicMatrix is None; build it before running DMC")
    control_horizon = dynamicMatrix.shape[1]
    prediction_horizon = dynamicMatrix.shape[0]
    control_moves = np.zeros(control_horizon)  # Control move sequence

    reference_window = np.zeros(prediction_horizon)  # Reference trajectory for prediction horizon
    delta_u = np.zeros(control_horizon)  # Control move increments
    time_step = 0
    previous_output = 0.0

    print("control Horizon:", control_horizon)
    print("prediction Horizon:", prediction_horizon)

    A = dynamicMatrix
    
    while(True):
        # sliding setpoint window for prediction horizon
        start_idx = min(time_step, len(setpoint) - prediction_horizon)
        end_idx = min(start_idx + prediction_horizon, len(setpoint))
        
        if end_idx - start_idx == prediction_horizon:
            reference_window = setpoint[start_idx:end_idx]
        else:
            reference_window[:end_idx - start_idx] = setpoint[start_idx:end_idx]
            reference_window[end_idx - start_idx:] = setpoint[-1]


        # Model prediction: y_pred = A * u + current_output_deviation
        output_deviation = plant.y - previous_output ## phi
        prediction = A @ control_moves + output_deviation

        error = reference_window - prediction
        
        # My favourite equation :(
        ATA_reg = A.T @ A + lambda_reg * np.eye(control_horizon)
        delta_u = np.linalg.inv(ATA_reg) @ A.T @ error
        
        # Shift control moves and add new one
        control_moves[1:] = control_moves[:-1]
        control_moves += delta_u  # Apply only the first control move increment

        np.clip(control_moves, 0, 2, out=control_moves)  # Clip control moves to limits
        
        # Apply first control move to plant
        plant.set_input(float(control_moves[0]))
        
        previous_output = plant.y
        time_step += 1
        yield env.timeout(plant.dt)


if __name__ == "__main__":
    ## Build Dynamic Matrix based on simulation
    DataEnv = simpy.Environment()
    DataEnv.process(plant.run(DataEnv))
    DataEnv.process(collect_data(DataEnv, plant, 10.0))
    control_signal = 1 # in volts
    dt = plant.dt
    duration = 10.0  # seconds
    
    DataEnv.run(until=duration)

    if dynamicMatrix is None:
        dynamicMatrix = build_dynamic_matrix(step_response, len(step_response), 1)


    # MPC Time
    mpc_duration = 10.0  # seconds  
    import matplotlib.pyplot as plt

    # After the MPC simulation, collect and plot results
    mpc_output = []
    mpc_control = []
    mpc_time = []

    def collect_mpc_output(env, plant, duration):
        while env.now < duration:
            # Ensure we always append a scalar value
            y_value = plant.y
            if hasattr(y_value, '__iter__') and not isinstance(y_value, str):
                y_value = float(y_value[0]) if len(y_value) > 0 else 0.0
            else:
                y_value = float(y_value)
            mpc_output.append(y_value)
            mpc_time.append(env.now)
            mpc_control.append(float(plant.u))  # Ensure we always append a scalar value for control
            yield env.timeout(plant.dt)

    # Reset plant for MPC simulation
    plant.reset()

    # Create a proper step function: 0 for first second, then 500 for rest
    num_points = int(mpc_duration / plant.dt)
    step_time = 1.0  # Step occurs at t = 1.0 seconds
    step_index = int(step_time / plant.dt)
    
    response = np.zeros(num_points)
    response[step_index:] = 500.0  # Step from 0 to 500 at t=1s
    
    mpcEnv = simpy.Environment()
    mpcEnv.process(plant.run(mpcEnv))
    mpcEnv.process(RunDMC(mpcEnv, plant, dynamicMatrix, response, lambda_reg=0.1))
    mpcEnv.process(collect_mpc_output(mpcEnv, plant, mpc_duration))

    mpcEnv.run(until=mpc_duration)



    # Plot
    plt.figure(figsize=(10, 6))
    plt.plot(mpc_time, mpc_output, label='Plant Output')
    
    # Create time array for setpoint plotting
    setpoint_time = np.arange(0, mpc_duration, plant.dt)
    plt.plot(setpoint_time, response, label='Setpoint', linestyle='--')

    ax2 = plt.twinx()
    ax2.set_ylim([0, 2])
    ax2.plot(mpc_time, mpc_control, label='Control Signal', linestyle='--', color='orange')
    ax2.set_ylabel('Control Signal')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Output')
    plt.legend()
    plt.grid()
    plt.show()