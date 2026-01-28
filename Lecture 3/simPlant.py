class SimPlant:
    def __init__(self, kp=0.0, time_constant=0.1):
        self.kp = kp
        self.tau = time_constant
        self.y = 0.0  # initial state

    def step(self, u, dt):
        # Simple linear plant: y'tau + y = kp*u
        # Discretized using Euler method
        # y[n+1] = (1 - dt/tau)*y[n] + (kp*tau/dt)*u[n]
        self.y += dt * ( - (1/self.tau) * self.y + (self.kp/self.tau) * u)
        return self.y