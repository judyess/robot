# to run in VSC, disconnect from remote WSL connection. Bottom right should say {}Python 3.12.2. idk how to find it again, but make sure the Kernel is the local Python 3.12
# these are provided examples that I'm fiddling with

from matplotlib import pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import math


class Link:
    def __init__(self, pin, origin, endpoint, linkLength):
        self.pin = pin
        self.origin = origin
        self.endpoint = endpoint
        self.linkLength = linkLength

# defining the robit; Need to define next links origin == last links endpoint
link1 = Link(0, (1, 2,0), (5, 6,0), 4)
link2 = Link(1, (5,6,0), (3,4,0), 3.75)

arm = [[(link1.origin), (link1.endpoint)], [link2.origin, link2.endpoint]]


# plot stuff
fig = plt.figure()
ax2 = fig.add_subplot(projection='3d')

for link in arm:
    x, y, z = zip(*link)
    ax2.plot(x, y, z)
    ax2.scatter(x, y, z, c='red', s=100)

# Graph stuff
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

plt.show()
