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


def read_frames(queue):
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

    # initialize opencv multitracker
    trackers = cv2.MultiTracker_create()

    # initialize the bounding box coordinates of the object we are going
    # to track
    box = None

    pipeline = rs.pipeline()
    config = rs.config()
    #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # initialize the FPS throughput estimator
    fps = None
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        #depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        # Convert images to numpy arrays
        #depth_image = np.asanyarray(depth_frame.get_data())
        frame = np.asanyarray(color_frame.get_data())
        (H, W) = frame.shape[:2]
        # resize the frame (so we can process it faster)
        frame = imutils.resize(frame, width=600)

        if box is not None:
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
                ("Tracker", args["tracker"]),
                ("Success", "Yes" if success else "No"),
                ("FPS", "{:.2f}".format(fps.fps())),
            ]
            # loop over the info tuples and draw them on our frame
            for (i, (k, v)) in enumerate(info):
                text = "{}: {}".format(k, v)
                cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        # if not check:
        #     break
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
            # select the bounding box of the object we want to track (make
            # sure you press ENTER or SPACE after selecting the ROI)
            box = cv2.selectROI("Frame", frame, fromCenter=False,
                showCrosshair=True)
            # grab the appropriate object tracker using our dictionary of
            # OpenCV object tracker objects
            tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
            trackers.add(tracker, frame, box)
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
    frame = queue.get()
    # show the output frame
    cv2.imshow("Copy",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            cam_process.terminate()  # Don't do this if shared resources
            break
    time.sleep(1)

