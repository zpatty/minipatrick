import sys

from multiprocessing import Process, Queue, Pipe

import robot_cv_multiprocess as vision

import cv2

import time

import serial

import multi_serial_comms as m_serial

import cognition_dummy as cognition

def start_robot(device_name):
    sender, receiver = Pipe()

    state_queue = Queue()
    cam_process = Process(target=vision.cv_process, args=(state_queue,))
    cam_process.start()

    serial_process = Process(target=m_serial.echo_to_terminal, args=(device_name,sender))
    serial_process.start()

    serial_port = serial.Serial(port=device_name, baudrate=115200, timeout=1,
                                        exclusive=False)


    while True:

        
        # get frame from the queue
        state = state_queue.get()
        print(state)
        print("now")
        time.sleep(0.1)
        cognition.dummy(serial_port)
        receiver.recv()

if __name__ == '__main__':
    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        start_robot(sys.argv[1])
    except KeyboardInterrupt:
        # why is this here?
        pass


