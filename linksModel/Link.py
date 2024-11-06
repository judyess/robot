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
    
    """ 
    a links origin is equal to the previous links position. This sets the argument links origin to the previous links (end) position
    this loops through all links in the arm
    """
    def setOrigin(link):
        for item in Link.allLinks:
            if link.pin !=0:
                try:
                    if item.pin == link.pin - 1:
                        link.origin = item.position
                except:
                    link.origin = (0,0,0)

        """This setup() function is meant to run only once to set the initial values"""
    def setup(link): 
        if Link.checkDuplicate(link) == False:
            link.pin = Link.numberOfLinks
            Link.numberOfLinks+=1
            Link.allLinks.append(link)
            Link.setOrigin(link)
        else:
            return

