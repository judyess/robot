class Link:
    allLinks = []
    numberOfLinks = 0
    
    def __init__(self, name, position, origin = (0,0,0), pin = 999):
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

    """ Thinking about using threads so this function is automatically called whenever a links position changes.
        *Reminder: 
            This cycles through all links starting from 0 to n. 
            The robot arm actually only needs to update the active/moving link and all f o l l o w i n g links.
            May want to rewrite to instead loop through links i to n. Where i is the active link's index.
    """
    def updateOrigin(link):
        for item in Link.allLinks:
                try:
                    if item.pin == link.pin - 1:
                        link.origin = item.position
                except:
                    link.origin = (0,0,0)

    """
    This returns the previous link. If its the first link (index 0), it will return "none"
    """
    def previous(link): 
            if Link.allLinks[link.pin - 1].pin < link.pin: # without this, the 0 index link will return a previous link equal to the last link in the list.
                previous = Link.allLinks[link.pin - 1]
                return previous
            else:
                return link