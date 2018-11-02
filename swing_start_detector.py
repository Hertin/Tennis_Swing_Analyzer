import numpy as np
import matplotlib.pyplot as plt
from util import *

NOT_STARTED = 0
STARTED     = 1
state = NOT_STARTED
state_string = ['not_started', 'started']

trigger_thresh = 10e7
end_thresh     = 5e7
BUF_SIZE       = 9
X, Y, Z        = 0, 1, 2
mac            = "E5:98:83:1C:02:E3"
peak           = np.array([ 224., 928., 2912., 7872., 7968., 1760., 960., 512., 288.])
swing_data_set = []
acc_buf        = np.zeros((0,3))
swing_data     = np.zeros((0,3))

ch = setup_bluetooth(mac)
y_similarity = 0
while True:
    reading = ch.read()
    acc = np.frombuffer(reading, dtype=np.int16)[None,:]
    is_triggered = False
    has_ended = False
    
    if len(acc_buf) >= BUF_SIZE:
        acc_buf = acc_buf[1:]
        acc_buf = np.append(acc_buf, acc, axis=0)
        y_similarity = np.inner(acc_buf[:,Y], peak)
        is_triggered = y_similarity > trigger_thresh
        has_ended = y_similarity < end_thresh
    else:
        acc_buf = np.append(acc_buf, acc, axis=0)

    if state == NOT_STARTED:
        if is_triggered:
            print('peak detected and start')
            state = STARTED
            swing_data = acc_buf
    elif state == STARTED:
        if has_ended:
            print('swing end')
            state = NOT_STARTED
            swing_data_set.append(swing_data)
        else:
            print('similarity score', y_similarity)
            swing_data = np.append(swing_data, acc, axis=0)


