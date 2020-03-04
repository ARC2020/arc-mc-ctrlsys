'''
* Project           : ARC 2020 = Control System 
*
* Program name      : pid.py
*
* Author            : Kirsty
*
* Date created      : 2020-02-03
*
* Purpose           : Control system to run 
*
* Revision History  :
*
* Date          Author      Ref    Revision (Date in YYYY-MM-DD format) 
* 2020-02-03    kirsty      1      Made file  
*
'''
import time

class Pid():

    def __init__( self, fKp = 0, fKi = 0, fKd = 0):
        self.fKp            = fKp
        self.fKi            = fKi
        self.fKd            = fKd
        self.fError         = 0
        self.fIntegral      = 0
        self.fPrevError     = 0

    def enableTime(self, sampleTime)    
        self.timeToggle     = True
        self.sampleTime     = sampleTime
        self.nowTime        = time.time()
        self.oldTime        = self.nowTime

    def disableTime(self)
        self.timeToggle = False

    def setTarget(self, fTarget):
        self.fTarget = fTarget
    
    def setSampleTime(self, time):
        self.sampleTime = time

    def setWindupLimit(self, fWindup):
        self.fWindupLimit = fWindup

    def resetIntegral(self):
        self.fIntegral = 0

    def windupCrossover(self):
        '''
        possible anti-windup 
        removes integral term if error crosses zero
        '''
        if abs(self.fError + self.fPrevError) != abs(self.fError) + abs(self.fPrevError):
            self.resetIntegral()

    
    def windupGuard(self):
        '''
        possible anti-windup 
        saturates integral term if out of bounds
        '''
        if not hasattr(self, "fWindupLimit"):
            return
        if self.fIntegral > self.fWindupLimit:
            self.fIntegral = self.fWindupLimit
        elif self.fIntegral < -self.fWindupLimit:
            self.fIntegral = -self.fWindupLimit
   

    def run(self, fInput):
        timeCoeff = 1 
        if hasattr(self, "timeToggle"):
            if self.timeToggle:
                nowTime = time.time()
                diffTime = nowTime - self.oldTime
                if diffTime > self.sampleTime:
                    timeCoeff = diffTime
                else:
                    return 0

        # calculate error
        self.fError = self.fTarget - fInput
        self.fIntegral += self.fError*timeCoeff
        fDerivative = (self.fError - self.fPrevError)/timeCoeff
        
        # antiwindup 
        # check if error crosses zero
        # self.windupCrossover() 
        # check if integral term too large
        # self.windupGuard() 

        # calculate output 
        fOutput = self.fKp * self.fError + self.fKi * \
            self.fIntegral + self.fKd*fDerivative

        self.fPrevError = self.fError
        return fOutput

if __name__ == "__main__":
    # testing this class
    # this is not a good test, need to either test
    # with empirical data or simulate environment 
    import numpy as np 
    import matplotlib.pyplot as plt
    a = np.zeros(100)
    #a[25:75] = 1
    a[50:] = 1
    ctrl = Pid(fKp = 1,fKi = 0.10)
    ctrl.setTarget(1)
    b = []
    for val in a:
        b.append(ctrl.run(val))
    plt.subplot(211)
    plt.plot(a)
    plt.subplot(212)
    plt.plot(b)
    plt.show()