from matplotlib import pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import math


fig = plt.figure()
ax = fig.add_subplot()

class Link:
    allLinks = []
    def __init__(self, origin, endpoint, pin = len(allLinks)):
        #Link.allLinks.append(self)   # SOMETHING CORE 
        self.origin = origin
        self.endpoint = endpoint
        self.pin = pin

    def __eq__(self, other): 
        if not isinstance(other, Link):
            return False
        return self.name == other.name 
        
def printArm(link):
    print(link.origin)
    print(link.endpoint)
    print(link.pin)

link1 = Link((0, 0, 0), (3, 4, 0), 0)
link2 = Link(link1.endpoint, (5,6,0), 1)

Link.allLinks.append(link1)
Link.allLinks.append(link2)
print(link1.pin)
print(link2.pin)
for link in Link.allLinks:
    printArm(link)


def plotArm(array):
    for link in array:
        coords = [link.origin, link.endpoint]
        x, y, z = zip(*coords)                                              
        ax.plot(x, y)
        ax.scatter(x, y, c='red', s=100) 


newLinks = []
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
    theta = 45
    radians = (theta*math.pi)/180

    newX = ((x)*math.cos(radians)) - ((y)*math.sin(radians)) + x1
    newY = ((x)*math.sin(radians)) + ((y)*math.cos(radians)) + y1
    
    if link.pin >= 1:
        newOrigin = newLinks[len(newLinks) - 1].endpoint
    else:
        newOrigin = link.origin

    newLink = Link(newOrigin, (newX, newY, 0), len(newLinks)+1)
    newLinks.append(newLink)
    plotLink(newLink)


def plotLink(link): #correctly connects the previous link to the current link
    for linki in newLinks:
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

plotArm(Link.allLinks)

for link in Link.allLinks:
    rotateLink(link)
    



# Graph stuff
ax.set_xlabel('X')
ax.set_ylabel('Y')
#ax.set_zlabel('Z')
ax.set_xlim(-9, 10)
ax.set_ylim(-9, 10)
#ax.set_zlim(-10, 10)

plt.show()