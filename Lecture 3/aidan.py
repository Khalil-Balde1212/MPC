from time import sleep, time
from plant_interface import initialize_connection, send_to_serial, read_from_serial, cleanup

if __name__ == "__main__":
    kp, ki = 0.00596, 0.028408163
    perror, ierror = 0.0, 0.0

    setpoint = 1000.0
    dt = 0.1
    initialize_connection()
    send_to_serial(0)
    sleep(1)

    start_time = time()
    last_time = start_time

    while True:
        try:
            pass
        except KeyboardInterrupt:
            for _ in range(5):
                send_to_serial(0)
                sleep(0.1)
            cleanup()
            break
        
        if time() - last_time >= dt:
            # Simple PI control loop
            current_speed = read_from_serial()  # hypothetical function
            perror = setpoint - current_speed
            ierror += perror * dt

            # Anti-windup for integral error
            ierror = max(-2.5, min(2.5, ierror))
            

            u = kp * perror + ki * ierror
            u *= 255.0/5.0
            u = max(0, min(128, int(u)))  # clamp to 0-255
            send_to_serial(u)
            
            

