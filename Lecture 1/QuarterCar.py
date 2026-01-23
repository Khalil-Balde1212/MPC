import numpy as np
import matplotlib.pyplot as plt

class QuarterCar:
    def __init__(self, m_tire, c_tire, k_tire, m_car, c_car, k_car, delta_t):
        self.m_tire = m_tire  # Mass (kg)
        self.c_tire = c_tire  # Damping coefficient (N·s/m)
        self.k_tire = k_tire  # Spring constant (N/m)

        self.m_car = m_car
        self.c_car = c_car
        self.k_car = k_car

        # State stuff
        self.delta_t = delta_t
        self.x_tire = 0.0      # Initial position (m)
        self.v_tire = 0.0      # Initial velocity (m/s)
        self.a_tire = 0.0      # Initial acceleration (m/s²)

        self.x_car = 0.0      # Initial position (m)
        self.v_car = 0.0      # Initial velocity (m/s)
        self.a_car = 0.0      # Initial acceleration (m/s²)

    def iterate(self, U):

        # Car body ODE: m_car*x'' + c_car*(x_tire - x_car') + k_car*(x_car - x_tire) = 0
        # Isolate car x'': x'' = (- c_car*(x_car' - x_tire) - k_car*(x_car - x_tire)) / m_car
        self.a_car = ( - self.c_car * (self.v_car - self.v_tire) - self.k_car * (self.x_car - self.x_tire)) / self.m_car

        # Car tire ODE: m_tire*x'' + c_tire*x' + k_tire*x = U
        # isolate tire acceleration: x'' = (U - c_tire*x' - k_tire*x) / m_tire
        self.a_tire = (U - self.c_tire * self.v_tire - self.k_tire * self.x_tire) / self.m_tire
    
    
        # Update velocity and position using Euler integration
        self.v_tire = self.v_tire + self.delta_t * self.a_tire
        self.x_tire = self.x_tire + self.delta_t * self.v_tire
        
        self.v_car = self.v_car + self.delta_t * self.a_car
        self.x_car = self.x_car + self.delta_t * self.v_car

        return self.x_tire, self.x_car

if __name__ == "__main__":
    m_car = 250.0      # Mass (kg)
    c_car = 1000.0      # Damping coefficient (N·s/m)
    k_car = 16000.0     # Spring constant (N/m)
    
    m_tire = 40.0
    c_tire = 1000.0
    k_tire = 150000.0   # Tuned parameters
    dt = 0.001   # Time step (s)
    t_end = 5.0  # Simulation time (s)

    # setup simulation
    t = np.arange(0, t_end, dt)
    F = np.zeros_like(t)


    # Select to analyze response
    # F = np.ones_like(t) # flat input of 1 N
    # F[int(0.5 / dt)] = 1.0  # Dirac-Delta Impulse of 1 Ns at t=0.5s
    F = np.heaviside(t - 0.5, 1) * 1  # Step input of 1 N at t=0.5s
    # F = np.sin(2 * np.pi * 1.0 * t)  # 1 Hz Sine wave input force

    # noise addition
    F *= np.random.uniform(0.99, 1.01, size=F.shape)

    quarter_car = QuarterCar(m_tire, c_tire, k_tire, m_car, c_car, k_car, dt)
    X_T = []
    X_C = []

    # simulate system response
    for force in F:
        x_tire, x_car = quarter_car.iterate(force)
        X_T.append(x_tire)
        X_C.append(x_car)


    # Plot results
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(t, F, label='Applied Force (N)')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Input Force')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(t, X_T, label='tire pos (m)')
    plt.plot(t, X_C, label='car body pos (m)')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')


    plt.title('Quarter Car Response')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
