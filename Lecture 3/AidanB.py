
from time import time, sleep
from plant_interface import start_live_plot, send_to_plot_serial

if __name__ == "__main__":
    # Start live plotting in background (this initializes the connection internally)
    start_live_plot()
    
    print("Live plot running in background.\n")
    
    # Give the plant an initial input so we can see something
    sleep(1)  # Wait for plot to initialize
    send_to_plot_serial(0)  # Start with a middle value
    
    print("Initial input set. Waiting 5 seconds...")
    sleep(5)
    
    print("Changing to maximum input...")
    send_to_plot_serial(255)
    
    # Keep running
    try:
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("\nExiting...")



    




