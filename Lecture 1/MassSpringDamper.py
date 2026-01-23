import numpy as np
import matplotlib.pyplot as plt

class MassSpringDamper:
    def __init__(self, m, c, k, delta_t):
        self.m = m  # Mass (kg)
        self.c = c  # Damping coefficient (N·s/m)
        self.k = k  # Spring constant (N/m)
        self.delta_t = delta_t
        self.x = 0.0      # Initial position (m)
        self.v = 0.0      # Initial velocity (m/s)

    def iterate(self, F):
        # ODE: mx'' + cx' + kx = F
        # Acceleration: x'' = (F - cx' - kx) / m
        a = (F - self.c * self.v - self.k * self.x) / self.m
        
        # Update velocity and position using Euler integration
        self.v = self.v + self.delta_t * a
        self.x = self.x + self.delta_t * self.v
        
        return self.x

if __name__ == "__main__":
    m = 250.0      # Mass (kg)
    c = 1000.0      # Damping coefficient (N·s/m)
    k = 16000.0     # Spring constant (N/m)
    
    dt = 0.001   # Time step (s)
    t_end = 5.0  # Simulation time (s)

    msd = MassSpringDamper(m, c, k, dt)

    # setup simulation
    t = np.arange(0, t_end, dt)
    F = np.zeros_like(t)


    # Select to analyze response
    # F = np.ones_like(t) # flat input of 1 N
    F[int(0.5 / dt)] = 1.0  # Dirac-Delta Impulse of 1 Ns at t=0.5s
    # F = np.heaviside(t - 0.5, 1) * 1.0  # Step input of 1 N at t=0.5s
    # F = np.sin(2 * np.pi * 1.0 * t)  # 1 Hz Sine wave input force

    # noise addition
    # F *= np.random.uniform(0.95, 1.05, size=F.shape)


    X = []

    # simulate system response
    for force in F:
        msd.iterate(force)
        X.append(msd.x)


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
    plt.plot(t, X, label='pos (m)')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Mass-Spring-Damper Response')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()