# Will represent the cv / state estimation functionality as a method to be spawned as a process.
# Andrew Sabelhaus 2020-06-02

# needed from environment: random waits.
import time, random

# Max number of "frames" for the queue
n_frames = 5

def cv_startup(q):
    # Put things to the queue randomly until...
    i_fr = 0
    while i_fr < n_frames:
        # random wait
        time.sleep(random.random())
        # fake data
        msg = 'Frame: ' + str(i_fr)
        # send it
        q.put(msg)
        # increment
        i_fr += 1
    # we're done
    print('CV out of frames, shutting down...')
