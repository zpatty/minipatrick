from contextlib import contextmanager
from multiprocessing import Process, Queue
from queue import Empty
import time
import cv2
import pyrealsense2 as rs
import numpy as np
import argparse
from imutils.video import VideoStream
from imutils.video import FPS
import imutils


# @contextmanager
# def open_camera(cam_num):
#     cam = cv2.VideoCapture(cam_num)
#     try:
#         yield cam
#     finally:
#         cam.release()

def update_tracker(box, frame, trackers, fps):
        # grab the new bounding box coordinates of the object
        (success, boxes) = trackers.update(frame)
        # check to see if the tracking was a success
        # loop over the bounding boxes and draw then on the frame
        for box in boxes:
            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # update the FPS counter
        fps.update()
        fps.stop()
        # initialize the set of information we'll be displaying on
        # the frame
        info = [
            #("Tracker", args["tracker"]),
            ("Success", "Yes" if success else "No"),
            ("FPS", "{:.2f}".format(fps.fps())),
        ]
        # loop over the info tuples and draw them on our frame
        (H, W) = frame.shape[:2]
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return [frame, boxes]

def parse_tracker():
        # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--tracker", type=str, default="kcf",
        help="OpenCV object tracker type")
    args = vars(ap.parse_args())

    # initialize a dictionary that maps strings to their corresponding
    # OpenCV object tracker implementations
    OPENCV_OBJECT_TRACKERS = {
        "csrt": cv2.TrackerCSRT_create,
        "kcf": cv2.TrackerKCF_create,
        "boosting": cv2.TrackerBoosting_create,
        "mil": cv2.TrackerMIL_create,
        "tld": cv2.TrackerTLD_create,
        "medianflow": cv2.TrackerMedianFlow_create,
        "mosse": cv2.TrackerMOSSE_create
    }
    return [args, OPENCV_OBJECT_TRACKERS]

def init_tracking(frame, trackers, OPENCV_OBJECT_TRACKERS, args):
    # select region with the robot
    bounding_box = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)

    # crop frame to that region
    cropped_frame = frame[int(bounding_box[1]):int(bounding_box[1]+bounding_box[3]), 
        int(bounding_box[0]):int(bounding_box[0]+bounding_box[2])]

    # set number of features to find and create ORB detection object
    n = 200
    orb = cv2.ORB_create(nfeatures = n)
    # find the keypoints with ORB
    kp = orb.detect(cropped_frame,None)
    # compute the descriptors with ORB
    kp, des = orb.compute(cropped_frame, kp)
    # convert keypoints to numpy
    points2f = cv2.KeyPoint_convert(kp)
    # set size of pixels
    w = 50
    h = w
    # convert to bounding box array format
    boxes = points2f - w
    n = np.size(boxes, 0)
    boxes = np.hstack((boxes, np.full((n,2), w)))
    # consolidate overlapping rectangles (needs to be a tuple)
    boxes = tuple(map(tuple,boxes))
    boxes, weights = cv2.groupRectangles(boxes, 1, 0.5)
    # recomute number of boxes
    n = np.size(boxes, 0)

    # convert coordinates of boxes back to original frame coordinates
    boxes[:,0:2] = boxes[:,0:2] + np.hstack((np.full((n,1), int(bounding_box[0])), 
        np.full((n,1), int(bounding_box[1]))))

    # add a tracker object for each box (needs to be a tuple)
    boxes = tuple(map(tuple,boxes))
    for box in boxes:
        tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
        trackers.add(tracker, frame, box)

    return [boxes]

def read_frames(queue):

    # parse user specified tracking algorithm
    args, OPENCV_OBJECT_TRACKERS = parse_tracker()

    # initialize opencv multitracker
    trackers = cv2.MultiTracker_create()

    # initialize the bounding box coordinates of the object we are going
    # to track
    boxes = None

    pipeline = rs.pipeline()
    config = rs.config()
    #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # initialize the FPS throughput estimator
    fps = None
    while True:
        # Can't put this stuff in a function bc pipeline needs to be 
        # called where it was started
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        # Convert images to numpy arrays
        frame = np.asanyarray(color_frame.get_data())
        
        # check if the tracking has been initialized
        if boxes is not None:
            frame, boxes = update_tracker(boxes, frame, trackers, fps)

        # honestly not quite sure what this does
        if not queue.empty():
            try:
                queue.get_nowait()
            except Empty:
                pass
        queue.put(frame)
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        # if the 's' key is selected, we are going to "select" a bounding
        # box to track


        if key == ord("s"):
            boxes = init_tracking(frame, trackers, OPENCV_OBJECT_TRACKERS, args)
            
            fps = FPS().start()

        # if the `q` key was pressed, break from the loop
        elif key == ord("q"):
            # close all windows
            cv2.destroyAllWindows()

            self.process.terminate()




queue = Queue()
cam_process = Process(target=read_frames, args=(queue,))
cam_process.start()
while True:
    # get frame from the queue
    frame = queue.get()
    # show the output frame
    cv2.imshow("Copy",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            cam_process.terminate()  # Don't do this if shared resources
            break
    time.sleep(0.5)

