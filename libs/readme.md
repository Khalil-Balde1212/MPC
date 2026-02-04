To use the plant interface, call the "plant_interface.initialize_connection()" method to connect. It should detect between windows and linux ports. If all of them fail, Aidan will create a simulated plant and run that as though it were communicating over serial

To read and send commands over serial, use the "read_from_serial()" and "send_to_serial()" methods accordingly