from contextlib import contextmanager
from multiprocessing import Process, Queue
from queue import Empty
import time
import cv2
import pyrealsense2 as rs
import numpy as np
import argparse
from pycpd import RigidRegistration
from imutils.video import VideoStream
from imutils.video import FPS
import imutils
from pupil_apriltags import Detector


# takes in intel camera stuff and spits out calibrated parameters and a detector class
def init_april_pose(pipeline,cfg):

    # Create streaming profile and extract camera intrinsics
    profile = cfg.get_stream(rs.stream.color)
    intr = profile.as_video_stream_profile().get_intrinsics()
    camera_params = [intr.fx, intr.fy, intr.ppx, intr.ppy]
    # initialize apriltag
    detector = Detector(families='tagStandard41h12')

    return camera_params, detector

def get_april_pose(pipeline, cfg, camera_params, detector):
    # Wait until detecting an apriltag before continuing on
    detections = []
    while detections == []:

        frame = get_frame(pipeline)

        # convert frame to grayscale for apriltag detection
        grayscale_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = detector.detect(grayscale_frame, estimate_tag_pose=True, 
            camera_params = camera_params, tag_size = 0.022/8)
        cv2.imshow("Frame", grayscale_frame)
        cv2.waitKey(1)

        # check if we're detecting 2 apriltags and then get their centers
        if len(detections) == 2:
            april1_center = detections[0].center
            april2_center = detections[1].center
        else:
            detections = []
    return [april1_center, april2_center]


def get_frame(pipeline):
    # Wait for a coherent frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    frame = np.asanyarray(color_frame.get_data())
    return frame

# Function to execute CV and output to script
def cv_process(queue):

    # Set up RealSense Stream
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    cfg = pipeline.start(config)

    # Find apriltag orientation
    camera_params, detector = init_april_pose(pipeline, cfg)
    
    # Initialize the FPS throughput estimator
    fps = None

    # Initialize output
    state = []

    fps = FPS().start()
    start_time = time.time()
    # Main Loop
    while True:

        # Ask the camera for a frame
        
        frame = get_frame(pipeline)
        

        this_time = time.time()
        april1_center, april2_center = get_april_pose(pipeline, cfg, camera_params, detector)
        

            # Collect state variables for output
        state = april1_center, april2_center
            
        if not queue.empty():
            try:
                queue.get_nowait()
            except Empty:
                pass

        # Put state variables in queue
        
        queue.put(state)


        
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        # if the 's' key is selected, we are going to "select" a bounding
        # box to track


        if key == ord("s"):
            continue

        # if the `q` key was pressed, break from the loop
        elif key == ord("q"):
            # close all windows
            print('goodbye')
            cv2.destroyAllWindows()

            self.process.terminate()



if __name__ == '__main__':
    queue = Queue()
    cam_process = Process(target=cv_process, args=(queue,))
    cam_process.start()
    while True:
        output = []
        # get output from the queue
        
        if not queue.empty():
            #print('blocked on get')
            output = queue.get()
            print(output)
        else:
            pass
        # show the output frame
        time.sleep(0.01)



