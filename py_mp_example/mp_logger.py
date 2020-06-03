# Will represent the logging functionality as a method to be spawned as a process.
# Andrew Sabelhaus 2020-06-02

# needed from environment - maybe not?
# import multiprocessing

def log_startup(q):
    # everyone will put to this queue, so we can do a blocking read
    print('Logger starting.')
    while True:
        next_msg = q.get()
        print('Logger got a message: ', next_msg)
