"""
Writing a model of my robot project because physically testing, retesting and 
    whipping out measuring tape was getting annoying.
Also, learning how to use Matplotlib is cool too.

Link.allLinks = array of all Link objects
"""

from matplotlib import pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import math

# plot stuff
fig = plt.figure()
ax = fig.add_subplot()
#ax = fig.add_subplot(projection='3d')
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

# probably an excessive thing
class Rotation:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
"""
    Math part to-do: 
    1. rotate around a point
    
    90 degree rotation matrix from 0,0:
    [-1, 0, 0]
    [ 0, 1, 0]
    [ 0, 0, 0]
"""
rotate90 = Rotation((np.array([-1,0,0])),(np.array([0,1,0])),np.array(([0,0,0])))
testMatrix = ((rotate90.x*(3-2)), (rotate90.y*(4+2))) # Correctly returns a point that shifts (3,4) but from an origin = (0,0). Need to add more so the origin = the links origin
print(testMatrix)

ax.plot(testMatrix[0][0], testMatrix[1][1])
ax.scatter(testMatrix[0][0], testMatrix[1][1], c='red', s=100) 



# defining the robit; Need to define next links origin == last links endpoint
link1 = Link(0, (1, 2,0), (5, 6,0), 4)
link2 = Link(1, (5,6,0), (3,4,3), 3.75)

arm = []
for link in Link.allLinks:
    arm.append([link.origin, link.endpoint])
print(arm)

''' 3D PLOT
for link in arm:
    x, y, z = zip(*link)                                                        # Zip
    ax.plot(x, y, z)
    ax.scatter(x, y, z, c='red', s=100) # Just adds a dot to the points
'''
# a 2D plotter for ease of view in the graph while testing the math
for link in arm:
    x, y, z = zip(*link)                                                        # Zip
    ax.plot(x, y)
    ax.scatter(x, y, c='red', s=100) # Just adds a dot to the points


# Graph stuff
ax.set_xlabel('X')
ax.set_ylabel('Y')
#ax.set_zlabel('Z')
ax.set_xlim(-5, 7)
ax.set_ylim(-5, 7)
#ax.set_zlim(-10, 10)

plt.show()
