class Link:
    allLinks = []
    numberOfLinks = 0
    
    def __init__(self, name, position, origin = (0,0,0), pin = 999):
        self.origin = origin
        self.position = position
        self.name = name
        self.pin = pin
        Link.addToList(self)

    def __eq__(self, other): # defines what it means for two links to be eqeual
        if not isinstance(other, Link):
            return False
        return self.name == other.name # Links are equal if they have the same name
        

    def setPin():
        return (len(Link.allLinks))
    
    
    def checkDuplicate(self):
        isDuplicate = False
        for link in Link.allLinks:
            if self == link:
                isDuplicate = True
                return isDuplicate
        return isDuplicate
    
    def addToList(link): 
        if Link.checkDuplicate(link) == False:
            link.pin = Link.numberOfLinks
            Link.numberOfLinks+=1
            Link.allLinks.append(link)
            Link.update(link)
        else:
            return

    def update(link):
        for item in Link.allLinks:
                try:
                    if item.pin == link.pin - 1:
                        link.origin = item.position
                except:
                    link.origin = (0,0,0)