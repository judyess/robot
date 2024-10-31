"""
Link.allLinks = array of all Link objects

"""

from matplotlib import pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import math


class Link:

    allLinks = []
    def __init__(self, pin, origin, endpoint, linkLength):

        self.pin = pin
        self.origin = origin
        self.endpoint = endpoint
        self.linkLength = linkLength
        Link.allLinks.append(self) # any new Link object is added to a list. The list is going to be used for defining relationships between links and stuff.
        """
        if(self.pin != 0):
            for link in Link.allLinks:              # In-Progress. Need to figure out how to consider angles when defining positions. Ugh.
                if link.pin == self.pin -1:
                    self.origin = Link.endpoint
                else:
                    self.origin = self.linkLength
        """


# defining the robit; Need to define next links origin == last links endpoint
link1 = Link(0, (1, 2,0), (5, 6,0), 4)
link2 = Link(1, (5,6,0), (3,4,0), 3.75)

arm = []

for link in Link.allLinks:
    arm.append([link.origin, link.endpoint])

print(arm)

# plot stuff
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

for link in arm:
    x, y, z = zip(*link)                                                        # Zip
    ax.plot(x, y, z)
    ax.scatter(x, y, z, c='red', s=100)

# Graph stuff
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(0, 15)
ax.set_ylim(0, 15)
ax.set_zlim(0, 15)

plt.show()
