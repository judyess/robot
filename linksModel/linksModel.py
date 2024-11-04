"""
Writing a model of my robot project because physically testing, retesting and 
    whipping out measuring tape was getting annoying.
Also, learning how to use Matplotlib is cool too.

Link.allLinks = array of all Link objects

To-Do: 
1. Find a way to map links to eachother better 
    instead of looping through the entire list of links
2. Figure out which way to display positions
3. Loop through all links at once and call rotate -- or something better
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
    def __init__(self, origin, endpoint, pin = len(allLinks)):
        self.origin = origin
        self.endpoint = endpoint
        self.pin = pin
        Link.allLinks.append(self) # any new Link object is added to a list.



# defining the robit; Need to define next links origin == last links endpoint
link1 = Link((0, 0, 0), (3, 4, 0))
link2 = Link(link1.endpoint, (5,6,0))

originalArm = []
def printArm():
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

def plotArm():
    for link in Link.allLinks:
        coords = [([link.origin, link.endpoint, 0])]
        x, y, z = zip(*coords)                                                        # Zip
        ax.plot(x, y)
        ax.scatter(x, y, c='red', s=100) # Just adds a dot to the points



newLinks = []
# I think this is correct. 
# Need to redefine how the links are related and loop through all links in the arm.
# angle is hard coded
def rotateLink(link):
    print("rotating")
    print(link.endpoint)
    for linki in Link.allLinks:
        if linki.pin == link.pin - 1:
            previousX = linki.origin[0]
            previousY = linki.origin[1]
        else:
            previousX = 0
            previousY = 0

    x1 = previousX
    y1 = previousY
    x2 = link.endpoint[0]
    y2 = link.endpoint[1]
    x = abs(x2 - x1)
    y = abs(y2 - y1)
    theta = 90
    radians = (theta*math.pi)/180
    # rotates a single link, then adds it back to the arm (using x1, y1)
    newX = ((x)*math.cos(radians)) - ((y)*math.sin(radians)) + x1
    newY = ((x)*math.sin(radians)) + ((y)*math.cos(radians)) + y1
    
    if link.pin >= 1:
        newOrigin = Link.allLinks[len(newLinks) - 1].endpoint
    else:
        newOrigin = link.origin


    Link.allLinks[link.pin].origin = newOrigin
    Link.allLinks[link.pin].endpoint = (newX, newY, 0)
    plotLink(link)



def plotLink(link): #correctly connects the previous link to the current link
    for linki in Link.allLinks:
        if linki.pin == link.pin - 1:
            previousLink = linki
        else:
            previousLink = link

    x = previousLink.origin[0]
    y = previousLink.origin[1]
    x2 = link.endpoint[0]
    y2 = link.endpoint[1]
    print("new points: ", x2, ", ", y2)
    plt.plot([x,x2], [y, y2])
    plt.scatter((x, x2), (y, y2), c='black', s=100)


printArm()
plotArm()
#Have to call this on links in order. Do it better.
rotateLink(link1)
rotateLink(link2)
printArm()


# Graph stuff
ax.set_xlabel('X')
ax.set_ylabel('Y')
#ax.set_zlabel('Z')
ax.set_xlim(-9, 10)
ax.set_ylim(-9, 10)
#ax.set_zlim(-10, 10)

plt.show()
