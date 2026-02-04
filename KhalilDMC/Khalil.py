import simPlant
import simpy

step_response = []

plant = simPlant.SimPlant(kp=1000, time_constant=0.8, dt=0.01)

def collect_data(env, plant, duration):
    global step_response
    control_signal = 1.0  # step input of 1 volt
    plant.set_input(control_signal)
    
    while env.now < duration:
        plant.step()
        step_response.append(plant.y)
        yield env.timeout(plant.dt)

        if plant.y >= 0.95 * plant.get_steady_state():
            break

if __name__ == "__main__":
    env = simpy.Environment()
    env.process(plant.run(env))
    env.process(collect_data(env, plant, 10.0))
    control_signal = 1 # in volts
    dt = plant.dt
    duration = 10.0  # seconds
    
    env.run(until=duration)

    
    ## Creating our model
    import numpy as np
    # construct dynamic matrix
    predictionHorizon = len(step_response)
    controlHorizon = 10

    dynamicMatrix = np.zeros((predictionHorizon, controlHorizon))

    for i in range(predictionHorizon):
        for j in range(controlHorizon):
            if i - j >= 0:
                dynamicMatrix[i, j] = step_response[i - j]

    print("Dynamic Matrix:")
    print(dynamicMatrix)
    # Save dynamic matrix as CSV
    np.savetxt('dynamic_matrix.csv', dynamicMatrix, delimiter=',')
    print("Dynamic matrix saved to dynamic_matrix.csv")

    control_moves = np.zeros(controlHorizon)

    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot([i * dt for i in range(len(step_response))], step_response)
    plt.xlabel('Time (s)')
    plt.ylabel('Output')
    plt.title('Step Response')
    plt.grid(True)
    plt.show()