# Will generate messages 'received from serial tty' as a method to be spawned as a process.
# Andrew Sabelhaus 2020-06-02

# needed from environment: random waits.
import time, random

def serial_in_startup(q):
    # infinte loop of "getting data"
    while True:
        msg = 'Recieved from brittlestar_onboard: ' + str(random.random())
        q.put(msg)
        # random wait
        time.sleep(random.random())