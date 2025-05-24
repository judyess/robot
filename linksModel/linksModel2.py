from matplotlib import pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import math


fig = plt.figure()
ax = fig.add_subplot()
testAngle = 45
plt.title(testAngle)

class Link:
    def __init__(self, root, endpoint, pin):
        self.root = root
        self.endpoint = endpoint
        self.pin = pin

    def __eq__(self, other): # this defines link equivalency. Links are equal if their "names" match. 
        if not isinstance(other, Link):
            return False
        return self.name == other.name 

allLinks = []
# Link(starting_coords, ending_coords, pin)
link1 = Link((0, 0, 0), (0.0, 5, 0), 0)
link2 = Link(link1.endpoint, (0.0,8.5,0), 1)
link3 = Link(link2.endpoint, (0.0,12.0,0), 2)
allLinks.append(link1)
allLinks.append(link2)
allLinks.append(link3)

def printLink(link):
    print("LINK:", link.pin, "= ROOT:", link.root, " ENDPOINT:", link.endpoint)

def plotArm(array):
    for link in array:
        coords = [link.root, link.endpoint]
        x, y, z = zip(*coords)                                              
        ax.plot(x, y)
        ax.scatter(x, y, c='red', s=100) 


#returns the correct coords 
# but only link1's coords match the output from the Arduino code.
# because it is returning values equal to applying the same angle to all links
# but if I apply the same angle in Arduino for each link, they do return the same results.
# it's just that the plot doesn't match how the arm is actually moving.
# (!) Need to redefine how the links are associated with eachother
newLinks = []
def rotateArm(link): 
    print("rotating")
    print(link.endpoint)
    for child in allLinks:
        if child.pin == link.pin - 1:
            previousX = child.root[0] # 05/22/25 possibly needs to be an endpoint coordinate instead of the root. idk. brief look.
            previousY = child.root[1]
        else:
            previousX = 0
            previousY = 0

    x1 = previousX
    y1 = previousY
    x2 = link.endpoint[0]
    y2 = link.endpoint[1]
    x = abs(x2 - x1)
    y = abs(y2 - y1)
    theta = testAngle
    radians = (theta*math.pi)/180

    newY = (((x)*math.cos(radians)) - ((-y)*math.sin(radians)) + x1)
    newX = (((x)*math.sin(radians)) + ((y)*math.cos(radians)) + y1)
    
    if link.pin >= 1:
        newroot = newLinks[len(newLinks) - 1].endpoint
    else:
        newroot = link.root

    newLink = Link(newroot, (newX, newY, 0), len(newLinks)+1)
    newLinks.append(newLink)
    printLink(newLink)

def rotateLink(link, angle): #5/24/25
    for child in allLinks:
        if child.pin == link.pin - 1:
            parentEndpointX = child.root[0] 
            parentEndpointY = child.root[1]
        else:
            parentEndpointX = 0
            parentEndpointY = 0

    x1 = link.root[0]
    y1 = link.root[1]
    x2 = link.endpoint[0]
    y2 = link.endpoint[1]
    x = abs(x2 - x1)
    y = abs(y2 - y1) # 5/24 I cant remember what this is for
    radians = (angle*math.pi)/180

    newY = (((x)*math.cos(radians)) + ((-y)*math.sin(radians)) + x1) # changed - to +. Seems to be correct but link2 is starting at link1's root instead of endpoint
    newX = (((x)*math.sin(radians)) + ((y)*math.cos(radians)) + y1)
    
    if link.pin >= 1:
        newroot = newLinks[len(newLinks) - 1].endpoint
    else:
        newroot = link.root

    newLink = Link(newroot, (newX, newY, 0), len(newLinks)+1)
    newLinks.append(newLink)
    printLink(newLink)




"""
11/10 Works.
Code has been stripped down. Currently, the rotateArm function has to be called on all links in an array, and it puts it in a new array.
I need the function to be able to rotate the arm from a specific point of rotation, 
    return the new positions and make sure they are connected to the links that did not change positions.
I am using two different arrays now so I can see how the links have changed positions but this might be making the code more complicated at this point. idk.
"""
#plotArm(allLinks)

#for link in allLinks:
#    rotateArm(link)
rotateLink(link1, 90)
rotateLink(link2, 45)
rotateLink(link3, 90) # is not rotating according to link2's position. 
# ^ rotating link3 at 90 degrees looks like its rotating in frame 0, but at 45 degrees, it looks like its going 0 or 180 degrees in frame 0. idk what that is.
# link 3's plot looks like the angle at 45 degrees is appearing as the angle between link2's root to link3's endpoint.
plotArm(newLinks)
    
# Graph stuff
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_xlim(-15, 15)
ax.set_ylim(-5, 15)
plt.show()