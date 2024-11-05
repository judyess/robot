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

# plot stuff
fig = plt.figure()
ax = fig.add_subplot()

# Link Objects (name, position)
link1 = Link("link1", (3, 4, 0))
link2 = Link("link2", (5,4,0))
link3 = Link("link3", (4,7,8))

def printArm(link):
    print(link.origin)
    print(link.position)
    print(link.pin)

for link in Link.allLinks:
    printArm(link)
arm = []
for link in Link.allLinks:
    arm.append([link.origin, link.position])
print(arm)



def plotArm():
    for link in arm:
        x, y, z = zip(*link)                                              
        ax.plot(x, y)
        ax.scatter(x, y, c='red', s=100) 

def rotateLink(link):
    #x1 = previousX
    #y1 = previousY
    x2 = link.position[0]
    y2 = link.position[1]
    #x = abs(x2 - x1)
    #y = abs(y2 - y1)
    theta = 45
    radians = (theta*math.pi)/180
    # rotates a single link, then adds it back to the arm (using x1, y1)
    #newX = ((x)*math.cos(radians)) - ((y)*math.sin(radians)) + x1
    #newY = ((x)*math.sin(radians)) + ((y)*math.cos(radians)) + y1
    plotLink(link)



def plotLink(link): 

    #print("new points: ", x2, ", ", y2)
    #plt.plot([x,x2], [y, y2])
    #plt.scatter((x, x2), (y, y2), c='black', s=100)
    plotArm()




currentArm = []

for link in Link.allLinks:
    currentArm.append(link)

for link in currentArm:
    rotateLink(link)
    



# Graph stuff
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_xlim(-9, 10)
ax.set_ylim(-9, 10)
plt.show()