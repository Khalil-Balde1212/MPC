from time import sleep
from plant_interface import initialize_connection, send_to_serial, read_from_serial

if __name__ == "__main__":
    # Initialize - this starts the plant and live plot in background
    initialize_connection()
    
    send_to_serial(255) # send a value between 0 and 255
    sleep(1)
    send_to_serial(0)
    sleep(1)
    
    while(True):
        print(read_from_serial())
        sleep(0.5)







    




