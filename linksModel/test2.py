# 05/25/2025
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import math
length = 1
rotation = 90
root = {
    "x":"y"
}

class Link:
    def __init__(self, pin, root, endpoint, length):
        self.pin = pin
        self.root = root
        self.endpoint = endpoint
        self.length = length
        Link.setup(self)
    def checkDuplicate(self):
        isDuplicate = False
        for link in Link.arm:
            if self == link:
                isDuplicate = True
                return isDuplicate
        return isDuplicate
    def __eq__(self, other): # this defines link equivalency. Links are equal if they have the same pin
        if not isinstance(other, Link):
            return False
        return self.pin == other.pin   
    arm = []
    def setup(self):
        if Link.checkDuplicate(self) == True:
            Link.arm[self.pin] = self
        else:
            Link.arm.append(self)
link1 = Link(0, (0, 0), (3, 0), 3)
link2 = Link(1, link1.endpoint, (6,0), 3)
link3 = Link(2, link2.endpoint, (9,0), 3)    
    
def atan():
    print("atan(y/x)")

def convertToRadians(degrees):
    radians = (degrees*math.pi)/180
    return radians

def isParent(child, parent):
    if(child.pin == parent.pin+1):
        return True
    else:
        return False
def update():
    for link in Link.arm:
        if link.pin >= 1:
            Link.arm[link.pin - 1].endpoint = link.root
    
def rotate(link, angle):
    x1 = link.root[0]
    y1 = link.root[1]
    x2 = link.endpoint[0]
    y2 = link.endpoint[1]
    x = abs(x2 - x1)
    y = abs(y2 - y1)
    radians = convertToRadians(angle)

    x =  (((x2)*math.cos(radians)) - ((y2)*math.sin(radians)))+x1
    y = (((y2)*math.cos(radians)) + ((x2)*math.sin(radians)))+y1
    link.endpoint = (x, y)
    if link != Link.arm[-1]:
        Link.arm[link.pin+1].root = link.endpoint

# Test functions
def printLink():
    for link in Link.arm:
        print("LINK:", link.pin, "= ROOT:", link.root, " ENDPOINT:", link.endpoint)


# PLOTTING STUFF
fig = plt.figure()
ax = fig.add_subplot()

def display():
    for link in Link.arm:
        coords = [link.root, link.endpoint]
        x, y = zip(*coords)                                              
        ax.plot(x, y)
        ax.scatter(x, y, c='red', s=100) 


# Begin
rotate(link1, 45)
rotate(link2, 90)
rotate(link3, 45)
display()
printLink()

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_xlim(-15, 15)
ax.set_ylim(-5, 15)
plt.show()