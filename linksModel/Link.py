class Link:
    allLinks = []
    numberOfLinks = 0
    
    def __init__(self, name, position, origin = (0,0,0), pin = 0):
        self.origin = origin
        self.position = position
        self.name = name
        self.pin = pin
        Link.setup(self) # The only call to setup()


    def __eq__(self, other): # defines what it means for two links to be eqeual
        if not isinstance(other, Link):
            return False
        return self.name == other.name # Links are equal if they have the same name
    
    def checkDuplicate(self):
        isDuplicate = False
        for link in Link.allLinks:
            if self == link:
                isDuplicate = True
                return isDuplicate
        return isDuplicate
    
 
    """This setup() function is meant to run only once to set the initial values"""
    def setup(link): 
        if Link.checkDuplicate(link) == False:
            link.pin = Link.numberOfLinks
            Link.numberOfLinks+=1
            Link.allLinks.append(link)
            Link.updateOrigin(link)
        else:
            return

    """ 
    a links origin is equal to the previous links position. This sets the argument links origin to the previous links (end) position
    this loops through all links in the arm
    """
    def updateOrigin(link):
        for i in range(link.pin, len(Link.allLinks)-1):
            otherLink = Link.allLinks[i]
            if link.pin !=0:
                try:
                    if otherLink.pin == link.pin - 1:
                        link.origin = otherLink.position
                except:
                    link.origin = (0,0,0)

    """
    updates the origin of all links that follow the active/moving link
    this only loops from the current link to the last link.
    """
    def update(link, newPosition):
        Link.updateOrigin(link)
        link.position = newPosition
        
        for i in range(link.pin+1, len(Link.allLinks)-1):
            followingLink = Link.allLinks[i]
            followingLink.origin = link.position
                

    """
    This returns the previous link. If its the first link (index 0), it will return "none"
    """
    def previous(link): 
        if link.pin != 0:
            if Link.allLinks[link.pin - 1].pin < link.pin: # without this, the 0 index link will return a previous link equal to the last link in the list.
                previous = Link.allLinks[link.pin - 1]
                return previous.pin
        
        return link.pin