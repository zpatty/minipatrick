# Little example for various complicated-ness of multiprocessing queues.
# Andrew Sabelhaus 2020-06-02

# needed from environment
import multiprocessing
# our modules
import mp_logger
import mp_cv
import mp_serial_in

if __name__ == '__main__':
    print('Starting up...')
    # To start, make a queue to transfer data to the logger
    log_q = multiprocessing.Queue()
    # target a method within the other module(s)
    log_p = multiprocessing.Process(target=mp_logger.log_startup, args=(log_q,))
    cv_p = multiprocessing.Process(target=mp_cv.cv_startup, args=(log_q,))
    serial_in_p = multiprocessing.Process(target=mp_serial_in.serial_in_startup, args=(log_q,))
    # this way it'll stay alive?
    log_p.daemon = True
    # start them up in order. Log first to catch anything that comes
    log_p.start()
    cv_p.start()
    serial_in_p.start()
    # then wait until they're done. CV should end first.
    cv_p.join()
    serial_in_p.join()
    log_p.join()

    # TO-DO:
    # catch keyboard interrupts and have everyone shut down nicely
    # formatting for log strings? can mp.Queue take objects of a class that we define?
    # will we ever need daemon?