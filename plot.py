import sys
import random
import signal
import time
import binascii
import numpy as np

from bluepy import btle
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib import style

# with open(output, 'w+') as f:
#     pass

style.use('fivethirtyeight')

fig = plt.figure(dpi=200)
ax1 = fig.add_subplot(1,1,1)



p = btle.Peripheral("E5:98:83:1C:02:E3", btle.ADDR_TYPE_RANDOM)
p.setSecurityLevel("medium")
svc = p.getServiceByUUID("e95d0753-251d-470a-a062-fa1922dfa9a8")
ch = svc.getCharacteristics("e95dca4b-251d-470a-a062-fa1922dfa9a8")[0]
chper = svc.getCharacteristics("e95dfb24-251d-470a-a062-fa1922dfa9a8")[0]
CCCD_UUID = 0x2902

ch_cccd=ch.getDescriptors(forUUID=CCCD_UUID)[0]

chper.write(b'\x02\x00')
p = chper.read()

numbers = []

xs = []
ys = []
zs = []
ts = []

with open('acceleration.data', 'w') as f:
    pass
start = time.time()

def animate(i):
    x = ch.read()
    number = np.frombuffer(x, dtype=np.int16)
#     print(number)
    x, y, z = number[0], number[1], number[2]
    t = time.time() - start
    xs.append(x)
    ys.append(y)
    zs.append(z)
    ts.append(t)
    ax1.clear()

    ax1.plot(ts[-100:], xs[-100:], label='x')
    ax1.plot(ts[-100:], ys[-100:], label='y')
    ax1.plot(ts[-100:], zs[-100:], label='z')
    ax1.legend()

    with open('acceleration.data', 'a') as f:
        f.write(','.join([str(t), str(x), str(y), str(z)])+'\n')

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()
