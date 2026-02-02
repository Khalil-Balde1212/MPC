from time import sleep
from plant_interface import initialize_connection, send_to_serial, read_from_serial, cleanup

if __name__ == "__main__":
    kp, ki = 0.00596, 0.028408163
    perror, ierror = 0.0, 0.0

    setpoint = 2000.0
    dt = 0.02
    initialize_connection()
    send_to_serial(0)
    sleep(1)

    while True:
        current_speed = read_from_serial()  # hypothetical function
        perror = setpoint - current_speed
        ierror += perror * dt

        u = kp * perror + ki * ierror
        # u *= 255/5
        u = max(0, min(255, int(u)))  # clamp to 0-255
        send_to_serial(u)

        sleep(dt)
        
        

