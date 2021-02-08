



import sys
import serial
import time

def flush(serial_port):
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()
    time.sleep(0.001)

def send_command(serial_port, input_string, delay_time):
    to_microcontroller_msg = f'{input_string}'
    serial_port.write(to_microcontroller_msg.encode('UTF-8'))
    if delay_time < 0:
        delay_time = 0
    time.sleep(delay_time/1000)

def sleepUntil(start_time, wait_until_since_start, dt):
    # the wall time when this function should complete:
    final_time = start_time + wait_until_since_start
    # then loop until we've waited long enough
    while time.time() < final_time:
        time.sleep(dt)

def execute_gait_cycle(serial_port, action_dict):    
    actions = action_dict
    primitives = actions.keys()
    start_time = time.time()
    command_sequence =  sorted(actions.items())
    for command in command_sequence:
        msg = command[1]
        command_time = command[0]
        #print("Attempting to send " + msg + " at time " + str(command_time) + ", current time is " + str((rospy.get_time() - start_time)*1000))
        sleepUntil(start_time,command_time/float(1000), 0.001)
        #print("After sleeping current time is: " + str((rospy.get_time() - start_time)*1000))

        # 1) add on the required newline
        if msg[-1] != "\n":
            msg = msg + "\n"
        # We seem to be getting some mis-aligned commands.
        # So, before anything else, write a newline each time. This should clear the junk out?
        
        #self.serial_port.write("! c\n")
        # give the microcontroller a moment
        # maybe 20 ms?
        clear_delay = 0.02
        time.sleep(clear_delay)
        
        #self.serial_port.write("! c\n")
        # 3) Send the message
        send_command(serial_port, msg, 0)
        # send debugging back to terminal
        print("Wrote command to serial port: " + msg[:-1] + " @; " + str((time.time() - start_time)*1000))

    else:
        print("Invalid action")
    #Send ready gisignal
    print("Done.")

def mayonnaise_instrument(device_name, activation_time = 250, frequency = 1, phase = 0.5):
    print(device_name, activation_time, frequency, phase)
    # A welcome message
    print("Running serial_tx_cmdline node with device: " + device_name)
    # create the serial port object, non-exclusive (so others can use it too)
    serial_port = serial.Serial(port=device_name, baudrate=115200, timeout=1,
                                exclusive=False)    # flush out any old data
    flush(serial_port)
    # finishing setup.
    print("Opened port. Ctrl-C to stop.")
    # activation_time = 70
    # frequency = 1.2
    # phase = 0.5
    period = 1000/frequency # milliseconds
    cooling_time = period - activation_time

    send_command(serial_port, "p " + str(activation_time), 0)
    w = 0
    # If not using ROS, we'll do an infinite loop:
    while True:
        # request something to send
        try:
            # execute_gait_cycle(serial_port, {100: "h 2 18", 300: "l 2 18", 301: "h 1 3 16 19", 600: "l 1 3 16 19", 650: "h 0 1 16 17", 850: "l 0 1 16 17",
            #    851+w: "h 0 1 18 19", 1050+w: "l 0 1 18 19", 1100+w: "h 0 1 18 19", 1300+w: "l 0 1 18 19", 2500+w: "s"})
            # execute_gait_cycle(serial_port, {100: "h 2 18", 300: "l 2 18", 301: "h 1 3 16 19", 600: "l 1 3 16 19", 
            #     650: "h 0 1 16 17", 950: "l 0 1 16 17", 1500: "s"})
            # WD Patrick
            # execute_gait_cycle(serial_port, {100: "h 5 19", 300: "l 5 19", 400: "h 4 6 16 18", 700: "l 4 6 16 18", 
            #     701: "h 6 16", 1000: "h 7 17", 1350: "l 6 16", 1450: "l 7 17", 3000: "s"})
            # # normal weighted PATRICK
            execute_gait_cycle(serial_port, {100: "h 10 16", 300: "l 10 16", 400: "h 9 11 17 19", 700: "l 9 11 17 19", 
                701: "h 9 19", 1000: "h 8 18", 1350: "l 9 19", 1450: "l 8 18", 3000: "s"})
            # normal weighted PATRICK
            # execute_gait_cycle(serial_port, {100: "h 0 14", 300: "l 0 14", 400: "h 1 3 13 15", 700: "l 1 3 13 15", 
            #     701: "h 3 13", 1000: "h 2 12", 1350: "l 3 13", 1450: "l 2 12", 3000: "s"})

            # execute_gait_cycle(serial_port, {100: "h 4 18", 300: "l 4 18", 400: "h 5 6 19 17", 700: "l 5 6 19 17", 
            #     701: "h 5 17", 1000: "h 7 16", 1350: "l 5 17", 1450: "l 7 16", 3000: "s"})
            # execute_gait_cycle(serial_port, {100: "h 1 15", 200: "l 1 15", 210: "h 0 2 12 14", 350: "l 0 2 12 14", 
            #     351: "h 2 12", 360: "h 3 13", 600: "l 2 3 12 13", 3000: "s"})
        except KeyboardInterrupt:
            # Nicely shut down this script.
            print("\nShutting down serial_tx_cmdline...")
            sys.exit()



            # the main function: just call the helper, while parsing the serial port path.
if __name__ == '__main__':
    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        mayonnaise_instrument(sys.argv[1])
    except KeyboardInterrupt:
        # why is this here?
        pass

