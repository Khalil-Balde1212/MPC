import simpy
class SimPlant:
    def __init__(self, kp=0.0, time_constant=0.1, dt=0.01):
        self.kp = kp
        self.tau = time_constant
        self.dt = dt
        self.y = 0.0  # initial state
        self.u = 0.0  # control input

    def set_input(self, u):
        self.u = u

    def step(self):
        # Simple linear plant: tau*y' + y = kp*u
        # Using forward Euler: y[n+1] = y[n] + (dt/tau) * (kp*u - y[n])
        self.y = self.y + (self.dt / self.tau) * (self.kp * self.u - self.y)
        return self.y
    
    def run(self, env):
        while True:
            self.step()
            yield env.timeout(self.dt)


    def get_steady_state(self):
        return self.kp * self.u
        