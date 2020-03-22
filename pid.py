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
* 2020-03-21    kirsty      2      Removed hungarian notation
'''
import time
from json import load, dumps

class Pid():

    def __init__(self, objectName, kp = 0, ki = 0, kd = 0):
        self.objectName    = objectName
        self.kp            = kp
        self.ki            = ki
        self.kd            = kd
        self.error         = 0
        self.integral      = 0
        self.prevError      = 0
        self.filename      = "pid-params.json"

    def loadJson(self, objectName = None, filename = None):
        if filename is None:
            filename = self.filename
        if objectName is None:
            objectName = self.objectName
        with open(filename) as f:
            data = load(f)
            constants = data[objectName]
            self.kp = constants["Kp"]
            self.ki = constants["Ki"]
            self.kd = constants["Kd"]
            self.setTarget(constants["target"])

    def writeJson(self, objectName = None, filename = None):
        if filename is None:
            filename = self.filename
        if objectName is None:
            objectName = self.objectName
        with open(filename, 'r') as f:
            # load json in case other params were changed since init load
            data = load(f)
        data[objectName]["Kp"] = self.kp
        data[objectName]["Ki"] = self.ki
        data[objectName]["Kd"] = self.kd
        # don't update target
        with open(filename, 'w') as f:
            f.write(dumps(data, indent = 4, sort_keys=True))

    def enableTime(self, sampleTime):
        self.timeToggle     = True
        self.sampleTime     = sampleTime
        self.nowTime        = time.time()
        self.oldTime        = self.nowTime

    def disableTime(self):
        self.timeToggle = False

    def setTarget(self, target):
        self.target = target
    
    def setSampleTime(self, time):
        self.sampleTime = time

    def setWindupLimit(self, windup):
        self.windupLimit = windup

    def resetIntegral(self):
        self.integral = 0

    def windupCrossover(self):
        '''
        possible anti-windup 
        removes integral term if error crosses zero
        '''
        if abs(self.error + self.prevError) != abs(self.error) + abs(self.prevError):
            self.resetIntegral()

    
    def windupGuard(self):
        '''
        possible anti-windup 
        saturates integral term if out of bounds
        '''
        if not hasattr(self, "windupLimit"):
            return
        if self.integral > self.windupLimit:
            self.integral = self.windupLimit
        elif self.integral < -self.windupLimit:
            self.integral = -self.windupLimit
   

    def run(self, input):
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
        self.error = self.target - input
        self.integral += self.error*timeCoeff
        derivative = (self.error - self.prevError)/timeCoeff
        
        # antiwindup 
        # check if error crosses zero
        self.windupCrossover() 
        # check if integral term too large
        self.windupGuard() 

        # calculate output 
        output = self.kp * self.error + self.ki * \
            self.integral + self.kd*derivative

        self.prevError = self.error
        return output

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