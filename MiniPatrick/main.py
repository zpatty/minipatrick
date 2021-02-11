import sys

from multiprocessing import Process, Queue, Pipe

import april_tracking as vision

import cv2

import time

import serial

import multi_serial_comms as m_serial

import one_step_planner

import brittlestar_agent

def start_robot(device_name):
    sender, receiver = Pipe()

    state_queue = Queue()
    cam_process = Process(target=vision.cv_process, args=(state_queue,))
    cam_process.start()

    serial_process = Process(target=m_serial.echo_to_terminal, args=(device_name,sender))
    serial_process.start()

    serial_port = serial.Serial(port=device_name, baudrate=115200, timeout=1,
                                        exclusive=False)

    planner = one_step_planner.OneStepPlanner()
    mini_patrick = brittlestar_agent.BrittlestarAgent(serial_port)
    while True:

        
        # get frame from the queue
        print('looping')
        state = list(state_queue.get())
        print(state)
        #print(state)
        next_action = planner.got_a_state(state)
        print(next_action)
        mini_patrick.action_callback(next_action)
        #time.sleep(0.1)
        #receiver.recv()

if __name__ == '__main__':
    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        start_robot(sys.argv[1])
    except KeyboardInterrupt:
        # why is this here?
        pass


