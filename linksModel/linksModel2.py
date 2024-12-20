from matplotlib import pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import math


fig = plt.figure()
ax = fig.add_subplot()

class Link:
    def __init__(self, origin, endpoint, pin):
        self.origin = origin
        self.endpoint = endpoint
        self.pin = pin

    def __eq__(self, other): # 
        if not isinstance(other, Link):
            return False
        return self.name == other.name 


allLinks = []
def printLink(link):
    print(link.origin)
    print(link.endpoint)
    print(link.pin)

link1 = Link((0, 0, 0), (0.0, 5, 0), 0)
link2 = Link(link1.endpoint, (0.0,8.5,0), 1)
link3 = Link(link2.endpoint, (0.0,12.0,0), 2)

allLinks.append(link1)
allLinks.append(link2)
allLinks.append(link3)


def plotArm(array):
    for link in array:
        coords = [link.origin, link.endpoint]
        x, y, z = zip(*coords)                                              
        ax.plot(x, y)
        ax.scatter(x, y, c='red', s=100) 


newLinks = []
#returns the correct coords 
# but only link1's coords match the output from the Arduino code.
# because it is returning values equal to applying the same angle to all links
# but if I apply the same angle in Arduino for each link, they do return the same results.
# it's just that the plot doesn't match how the arm is actually moving.
# (!) Need to redefine how the links are associated with eachother
def rotateLink(link): 

    print("rotating")
    print(link.endpoint)
    for linki in allLinks:
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
    theta = 31.95
    radians = (theta*math.pi)/180

    newY = (((x)*math.cos(radians)) - ((-y)*math.sin(radians)) + x1)
    newX = (((x)*math.sin(radians)) + ((y)*math.cos(radians)) + y1)
    
    if link.pin >= 1:
        newOrigin = newLinks[len(newLinks) - 1].endpoint
    else:
        newOrigin = link.origin

    newLink = Link(newOrigin, (newX, newY, 0), len(newLinks)+1)
    newLinks.append(newLink)
    printLink(newLink)


"""
11/10 Works.
Code has been stripped down. Currently, the rotateLink function has to be called on all links in an array, and it puts it in a new array.
I need the function to be able to rotate the arm from a specific point of rotation, 
    return the new positions and make sure they are connected to the links that did not change positions.
I am using two different arrays now so I can see how the links have changed positions but this might be making the code more complicated at this point. idk.
"""
plotArm(allLinks)

for link in allLinks:
    rotateLink(link)

plotArm(newLinks)
    



# Graph stuff
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_xlim(-15, 15)
ax.set_ylim(-5, 15)
plt.show()