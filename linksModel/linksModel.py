"""
Writing a model of my robot project because physically testing, retesting and 
    whipping out measuring tape was getting annoying.
Also, learning how to use Matplotlib is cool too.

Link.allLinks = array of all Link objects

!!! Everything has to be redefined

# 11/4 all current definitions work as they should

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
from Link import Link


newPositions=[]
currentArm = []

testAngle = 80

# plot stuff
fig = plt.figure()
ax = fig.subplots()

# Link Objects (name, position)
link1 = Link("link1", (3, 4, 0))
link2 = Link("link2", (5,4,0))
link3 = Link("link3", (4,7,8))

all = Link.allLinks

def printLink(link):
    print(link.pin, ": ", link.origin, ", ", link.position)

def printArm(array):
    for link in array:
        printLink(link)

""" DOES NOT WORK. KEEPS RETURNING CRAP
def previous(link):
    if link.pin != 0:
        return Link.allLinks[link.pin-1]
    else:
        return link
def next(link):
    if link.pin != len(all)-1:
        return all[link.pin+1]
"""


def rotateLink(link):
    previous = link
    if link.pin != 0:
        try:
            previous = previous(link)
        except:
            previous = link

    #rotates a single link, then adds it back to the arm (using x1, y1)
    x1 = previous.position[0]
    y1 = previous.position[1]
    x2 = link.position[0]
    y2 = link.position[1]
    x = abs(x2 - x1)
    y = abs(y2 - y1)
    theta = testAngle
    radians = (theta*math.pi)/180
    newX = ((x)*math.cos(radians)) - ((y)*math.sin(radians)) + x1
    newY = ((x)*math.sin(radians)) + ((y)*math.cos(radians)) + y1
    newPosition = (newX, newY, 0)

    #update positions
    link.position = newPosition
    if link.pin != len(all)-1:
        nextLink = all[link.pin+1]
        nextLink.origin = newPosition 
    
    # plot new positin
    newPositions.append(link)
    plotArm(newPositions)


def plotArm(array):
    for link in array:
        coords = [link.origin, link.position]
        x, y, z = zip(*coords)                                              
        ax.plot(x, y)
        ax.scatter(x, y, c='red', s=100) 

def main():
    plotArm(all)
    printArm(all)
    for link in all:
        rotateLink(link)
    plotArm(newPositions)
    printArm(newPositions)

main()

# Graph stuff
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_xlim(-9, 10)
ax.set_ylim(-9, 10)
plt.show()