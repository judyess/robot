# to run in VSC, disconnect from remote WSL connection. Bottom right should say {}Python 3.12.2. idk how to find it again, but make sure the Kernel is the local Python 3.12
# these are provided examples that I'm fiddling with

from matplotlib import pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import math
#plt.style.use('_mpl-gallery')           #Remove this line to see a regular coord frame

# use zip() which will pair 

class Link:
    def __init__(self, id, origin, endpoint, linkLength):
        self.id = id
        self.origin = origin
        self.endpoint = endpoint
        self.vector = ((self.origin[0], self.endpoint[0]), [self.origin[1], self.endpoint[1]]) # because plot([x], [y]) and "[x],[y]" = (x[i], y[i])
        self.linkLength = 0

link1 = Link("link1", (1, 2), (2, 6), 4)

origin = [1,2]
endpoint = [2, 6]

segments = [
    [(0, 0, 0), (1, 1, 1)],
    [(1, 1, 1), (2, 2, 2)],
    [(2, 2, 2), (3, 3, 3)]
]
# Create a figure and axes
fig = plt.figure()
ax2 = fig.add_subplot(projection='3d')

# Plot each line segment
for segment in segments:
    x, y, z = zip(*segment)
    ax2.plot(x, y, z)
    ax2.scatter(x, y, z, c='red', s=100)

# Set labels
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

plt.show()
