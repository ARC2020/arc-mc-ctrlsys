from math import tan

class Speed():
    def __init__(self):
        self.fTarget = 0
    
    def setTarget(self, target):
        self.fTarget = target

    def calcTarget(self, blobs):
        '''
        returns a new speed based off of blob position
        '''
        calcSpeed = 0
        return calcSpeed

class Steering():
    def __init__(self):
        pass

    def calcAngle(self, posBike, posTarget, distanceTarget):
        '''
        returns angle in radians 
        need to add bounds 
        '''
        return tan(posBike - posTarget) / distanceTarget

