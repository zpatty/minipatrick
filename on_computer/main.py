import sys

from multiprocessing import Process, Queue, Pipe

import robot_cv_multiprocess as vision

import cv2

import time

import multi_serial_comms as serial

import cognition_dummy as cognition

sender, receiver = Pipe()

state_queue = Queue()
cam_process = Process(target=vision.cv_process, args=(state_queue,))
cam_process.start()

serial_process = Process(target=serial.echo_to_terminal, args=('/dev/ttyACM0',sender))
serial_process.start()


while True:

    
    # get frame from the queue
    state = state_queue.get()
    print(state)
    print("now")
    time.sleep(0.5)
    cognition.dummy('/dev/ttyACM0')
    receiver.recv()



