#!/usr/bin/python

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python uses python 2.7. We need 2.7.

# brittlestar_agent represents the brittle star robot, with communication over USB,
# a list of actions, and publishing/subscribing/services for other nodes to interact with it.

# Usage is:
#	rosrun soft_robot_agents brittlestar_agent.py /dev/ttyACMX

# ...where ACMX is ACM0, ACM1, etc., the name of the serial device for the Nordic microcontrollers' UART.

# Imports:
# because we need the command-line arguments
import sys
# and for the serial/usb/uart via pyserial:
import serial
# may need to manipulate arrays?
# import numpy as np

import os
import numpy as np

from datetime import time
import time

# As with everything else, use a class (this way we can refer back to the pyserial instances)
class BrittlestarAgent:
    dummy_device_name = "{}/dummy_device_log.txt".format(os.path.dirname(os.path.realpath(__file__)))


    @staticmethod
    def action_remapper(action_sequence, reference_limb_number, new_limb_number):
        print("Remapping from limb {} to limb {} for sequence {}.".format(reference_limb_number, new_limb_number, action_sequence))
        remapped_sequence = {}
        def string_remapper(action_string, reference_limb_number, new_limb_number):
            string_chars = action_string.split(" ")
            voltage = string_chars[0]
            reference_limb_numbers = string_chars[1:]
            remapped_string_chars = [voltage]
            for limb_number in reference_limb_numbers:
                hardware_swapped_number = int(limb_number)
                remapped_hardware_swapped_limb_number = np.mod(hardware_swapped_number + np.mod(4 *(new_limb_number-reference_limb_number),20),20)
                remapped_limb_number = str(remapped_hardware_swapped_limb_number)
                remapped_string_chars.append(remapped_limb_number)
                print("original: {}, rotated: {}".format(limb_number, remapped_limb_number))
            remapped_string = " ".join(remapped_string_chars)
            return remapped_string
        
        for time, action_string in action_sequence.items():
            remapped_sequence[time] = string_remapper(action_string,reference_limb_number,new_limb_number)
        
        return remapped_sequence

    @staticmethod
    def get_actions():
        # A helper for code clarity:
        # Create each of the possible actions (strings) here, specifying their times to send
        # Examples: left, right.
        # key: actuation start time, milliseconds
        # value: string to send to the microcontroller
        print("Get actions called, regenerating actions and transition model.")

        #a_limb1 = {100: "h 1 15", 500: "l 1 15", 501: "h 0 2 14 12", 900: "l 0 2 14 12", 901: "h 2 12", 1000: "h 3 13", 1200: "l 2 12 13 3", 2000: "h 5 7 9 11 17 19", 2600: "l 0"}
        a_limb1 = {100: "h 1 15", 300: "l 1 15", 301: "h 0 2 14 12", 500: "l 0 2 14 12", 501: "h 2 12", 600: "h 3 13", 700: "l 2 12 13 3", 2600: "l 0"}
        #a_limb1 = {100: "h 1 15", 200: "l 1 15", 301: "h 0 14", 500: "h 2 12", 599: "l 0 2 14 12", 600: "h 2 12", 900: "h 3 13", 1000: "l 2 12 13 3", 1500: "h 1 15", 1700: "l 1 15", 2500: "l 0"}

        #a_limb1 = {100: "h 1 15", 200: "l 1 15", 201: "h 0 2 14 12", 350: "l 0 2 14 12", 351: "h 2 12", 352: "h 3 13", 500: "l 2 12 13 3", 2500: "l 0"}

        #, 701: "h 18", 800: "l 18", 801: "h 19 16", 900: "l 19", 901: "h 17", 1000: "l 16 17"
        #a_limb1 =  {100: "h 2", 300: "l 2", 400: "h 0 3", 600: "l 0 3", 601: "h 0 1", 800: "l 0 1", 1100: "h 9", 1300: "l 9", 1400: "h 11 8", 1600: "l 11 8", 1601: "h 11 10", 1800: "l 11 10"}#, 1600: "h 2 9", 1800: "l 2 9"}
        a_limb2 =  BrittlestarAgent.action_remapper(a_limb1, reference_limb_number = 1, new_limb_number = 2)
        a_limb3 =  BrittlestarAgent.action_remapper(a_limb1, reference_limb_number = 1, new_limb_number = 3)
        a_limb4 =  BrittlestarAgent.action_remapper(a_limb1, reference_limb_number = 1, new_limb_number = 4)
        a_limb0 =  BrittlestarAgent.action_remapper(a_limb1, reference_limb_number = 1, new_limb_number = 0)
        actions = {"limb0": a_limb0, "limb1": a_limb1, "limb2": a_limb2,"limb3": a_limb3,"limb4": a_limb4,}

        # Predicted transitions for each action.
        # For now, a list, where each is [dx, dy, dtheta]
        # t_left = {"dx" = -0.1, "dy" = 0.0, "dtheta" = 0.0}
        d = 5 # intended distance to move
        battery_leg0_angle = 288
        def leg_ang(num):
            theta = (np.pi/180)*(battery_leg0_angle-(360/5)*num) # Assumption leg0 points along the y axis if the ref point for orientation points along x axis
            return theta
        leg_vecs = np.around(np.array([[d*np.cos(leg_ang(N)), d*np.sin(leg_ang(N))] for N in range(5)]), decimals = 3)
        dxs = leg_vecs[:,0]
        dys = leg_vecs[:,1]

        transitions = {"limb0": [dxs[0],dys[0],0], "limb1": [dxs[1],dys[1],0], "limb2": [dxs[2],dys[2],0],"limb3": [dxs[3],dys[3],0],"limb4": [dxs[4],dys[4],0]} #TODO: replace with transition dist*vectors
        assert(set(actions.keys()) == set(transitions.keys()))
        return (actions, transitions, d)


    
    # The constructor calls a helper to initialize everything, and stores the
    # resulting publisher and serial port object that's created.
    def __init__(self, serial_port):
        self.serial_port = serial_port
        # Dictionaries for actions, transition_model
        # actions is a dict, where the key is time (in msec) that a string command should be sent, and the value is the command itself.
        # transition_model is a dict, with the key as a list of delta x, y, theta from a reference configuration.
        self.action_dict, self.transition_model,self.transition_dist = BrittlestarAgent.get_actions()

        print("Waiting on commands.. \n")

        # Params
        self.timing_resolution = 0.001

    def sleepUntil(self,start_time, wait_until_since_start, dt):
        # the wall time when this function should complete:
        final_time = start_time + wait_until_since_start
        print(wait_until_since_start)
        # then loop until we've waited long enough
        while time.time() < final_time:
            time.sleep(dt)

    def send_command(serial_port, input_string, delay_time):
        to_microcontroller_msg = f'{input_string}'
        serial_port.write(to_microcontroller_msg.encode('UTF-8'))
        if delay_time < 0:
            delay_time = 0
        time.sleep(delay_time/1000)

    def action_callback(self, action):
        # When a message is received:
        # 1) Pull out the actual SMA commands corresponding to the action name
        # 2) format the string based on the action
        # 3) actually send the message
        # the action (name) that was commanded by someone else

        print("Now busy...")
        action_name = action
        actions = self.action_dict
        primitives = actions.keys()
        if action_name in set(primitives):
            print("Executing primitive: {} \nPlease wait...".format(action_name))
            start_time = time.time()
            command_sequence =  sorted(actions[action_name].items())
            print("Command sequence: {}".format(command_sequence))
            for command in command_sequence:
                msg = command[1]
                command_time = command[0]
                #print("Attempting to send " + msg + " at time " + str(command_time) + ", current time is " + str((rospy.get_time() - start_time)*1000))
                self.sleepUntil(start_time,command_time/float(1000), 0.001)
                #print("After sleeping current time is: " + str((rospy.get_time() - start_time)*1000))

                # 1) add on the required newline
                if msg[-1] is not "\n":
                    msg = msg + "\n"
                # We seem to be getting some mis-aligned commands.
                # So, before anything else, write a newline each time. This should clear the junk out?
                
                #self.serial_port.write("! c\n")
                # give the microcontroller a moment
                # maybe 20 ms?
                clear_delay = 0.02
                #rospy.sleep(clear_delay)
                
                #self.serial_port.write("! c\n")
                # 3) Send the message

                to_microcontroller_msg = f'{msg}'
                #print(to_microcontroller_msg)
                self.serial_port.write(to_microcontroller_msg.encode('UTF-8'))
                # send debugging back to terminal
                print("Wrote command to serial port: " + msg[:-1] + " @; " + str((time.time() - start_time)*1000))

        else:
            print("Invalid action \"{0}\". Available actions are: {1}".format(action_name,primitives))
        #Send ready gisignal
        print("Done.")
        print("Ready... \n\n")



    # The primary helper function here opens the serial device,
    # and subscribes to a topic
    def serial_tx_startup(self, device_name):
        # A welcome message
        # Hard-code a timeout for pyserial. Seems recommended, even for tx?
        serial_timeout = 1
        # Next, do the serial setup:
        # Hard-coded: our microcontroller uses the following baud rate:
        psoc_baud = 115200
        # create the serial port objects
        serial_port = serial.Serial(device_name, psoc_baud, timeout=serial_timeout)
        # flush out any old data
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
        # finishing setup.
        print("Opened serial port for BrittlestarAgent.")
        # and return the serial device so that the callback can use it.
        return serial_port


# the main function: create one of these objects, while parsing the serial port path
if __name__ == '__main__':
    # the 0-th arg is the name of the file itself, so we want the 1st
    if len(sys.argv) == 2:
        robot = BrittlestarAgent(sys.argv[1])
    else:
        print("No device specified, not connecting to serial port. Logging commands to file instead.")
        robot = BrittlestarAgent(BrittlestarAgent.dummy_device_name)    
    
		

    # We do the spin() in main. It's not appropriate for a constructor.
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
