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



# defining the robit; Need to define next links origin == last links endpoint
link1 = Link(0, (0, 0, 0), (3, 4, 0), 4)
link2 = Link(1, link1.endpoint, (5,6,0), 3.75)

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


newLinks = []
# trying to rotate 2 arms by 90 degrees. Angle is hardcoded. 
# currently rotates links individually by 90 degrees, but still from an origin of 0,0. (so each point is acting like a link from 0,0 to its EP)
def rotateLink(link):
    print("rotating")
    print(link.endpoint)
    if link.pin != 0:
        previousX = Link.allLinks[link.pin-2].origin[0]
        previousY = Link.allLinks[link.pin-2].origin[1]
    else:
        previousX = 0
        previousY = 0
    # 2nd new point is wrong
    newX = rotate90.x * link.endpoint[0] #+ (link.origin[0] - previousX)
    newY = rotate90.y * link.endpoint[1] #+ abs(link.origin[0] - link.origin[1])

    if link.pin >= 1:
        newOrigin = newLinks[len(newLinks) - 1].endpoint
    else:
        newOrigin = link.origin

    newLink = Link(len(newLinks)+1, newOrigin, (newX[0], newY[1], 0), 0)
    newLinks.append(newLink)
    print(newLink.origin,", ", newLink.endpoint)
    plotLink(newLink)

def plotLink(link): #correctly connects the previous link to the current link
    if link.pin > 1:
        previousLink = newLinks[len(newLinks) - 1]
    else:
        previousLink = link

    x = previousLink.origin[0]
    y = previousLink.origin[1]
    x2 = link.endpoint[0]
    y2 = link.endpoint[1]
    print(x2, ", ", y2)
    plt.plot([x,x2], [y, y2])
    plt.scatter((x, x2), (y, y2), c='black', s=100)
    
rotateLink(link1)
rotateLink(link2)


# Graph stuff
ax.set_xlabel('X')
ax.set_ylabel('Y')
#ax.set_zlabel('Z')
ax.set_xlim(-6, 10)
ax.set_ylim(-6, 10)
#ax.set_zlim(-10, 10)

plt.show()
