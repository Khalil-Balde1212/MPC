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


if __name__ == "__main__":
    import socket
    import time
    import select
    
    # Socket server setup
    HOST = '127.0.0.1'
    PORT = 65432
    
    plant = SimPlant(kp=1000, time_constant=0.85, dt=0.005)
    control_input = 0.0  # Default control input (scaled)
    sample_rate = 0.02  # Send readings every 20ms
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"Plant server listening on {HOST}:{PORT}")
        print("Waiting for control input values...")
        
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            conn.setblocking(False)  # Non-blocking mode to read commands
            buffer = ""
            
            try:
                while True:
                    # Check for incoming commands
                    ready = select.select([conn], [], [], 0)
                    if ready[0]:
                        try:
                            data = conn.recv(1024).decode('utf-8')
                            if data:
                                buffer += data
                                # Process complete commands (newline-delimited)
                                while '\n' in buffer:
                                    line, buffer = buffer.split('\n', 1)
                                    line = line.strip()
                                    if line:
                                        try:
                                            control_input = float(line)
                                        except ValueError:
                                            break
                        except BlockingIOError:
                            pass
                    
                    # Run plant simulation steps
                    plant.set_input(control_input)
                    for _ in range(int(sample_rate / plant.dt)):
                        value = plant.step()
                    
                    # Send the sensor reading
                    message = f"{value}\n"
                    conn.sendall(message.encode('utf-8'))
                    
                    time.sleep(sample_rate)
            except (BrokenPipeError, ConnectionResetError):
                print("Client disconnected")
        