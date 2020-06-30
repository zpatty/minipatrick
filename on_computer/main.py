from multiprocessing import Process, Queue

import robot_cv_multiprocess as vision

import cv2

import time

import serial_comms as serial

queue = Queue()
cam_process = Process(target=vision.cv_process, args=(queue,))
cam_process.start()

serial_process = Process(target=serial.serial_process_start, daemon=True, args=(serial.CmdLnActionGenerator(),))
serial_process.start()

while True:
    # breakpoint()
    # get frame from the queue
    frame = queue.get()
    # show the output frame
    cv2.imshow("Copy",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            cam_process.terminate()  # Don't do this if shared resources
            serial_process.terminate()
            break
    time.sleep(0.5)


