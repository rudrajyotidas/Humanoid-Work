import math
import numpy as np


class LIPM:

    def __init__(self, x0, v0, z0, step_size, deltaT):

        #Initial Conditions
        self.x0 = x0
        self.v0 = v0
        self.zc = z0

        #Time step of update
        self.deltaT = deltaT

        #Current time
        self.t = 0

        #State of the LIPM at time t
        self.xt = 0
        self.zt = 0
        self.vt = 0
        self.a = 0

        #Acceleration due to graity
        self.g = 9.81

        #Tc
        self.Tc = math.sqrt(self.zc/self.g)

        #Leg Locations
        self.left_foot_x = 0
        self.right_foot_x = 0

        #Orbital energy
        self.orbital_energy = 0

        #Step length
        self.s = step_size

        #Final x and v of COM
        self.xf = 0
        self.vf = 0

        #Supporting leg
        self.support_leg = 'right'

        #Change in orbital energy for walk
        self.deltaE = 0

        #xCOM at which support exchange occurs
        self.x_support_exchange = self.right_foot_x + ((self.zc)*(self.deltaE))/((self.s)*(self.g)) + self.s/2

    def updateState(self):
        #Increase time
        self.t = self.t + self.deltaT
        
        w = self.t/self.Tc

        #Update x COM depenidng on the leg
        #self.xt = (self.x0)*math.cosh(w) + (self.Tc)*(self.v0)*math.sinh(w) (previous updte, when we didnt consider any steps)
        if self.support_leg == 'right':
            self.xt = self.right_foot_x + (self.x0)*math.cosh(w) + (self.Tc)*(self.v0)*math.sinh(w)
        
        elif self.support_leg == 'left':
            self.xt = self.left_foot_x + (self.x0)*math.cosh(w) + (self.Tc)*(self.v0)*math.sinh(w)

        #Update v COM
        self.vt = (self.x0)*math.sinh(w)/(self.Tc) + (self.v0)*(math.cosh(w))

        #Update orbital energy
        #self.orbital_energy = (self.vt**2 - (self.xt**2)*(self.zc)/self.g)/2
        #Remains constant

        if(self.xt > self.x_support_exchange):
            self.switchSupport()

    def switchSupport(self):

        if self.support_leg == 'right':

            #Change leg 
            self.support_leg = 'left'

            #Update foot coordinates
            self.left_foot_x = self.right_foot_x + self.s

            #Update next exchange point
            self.x_support_exchange = self.left_foot_x + ((self.zc)*(self.deltaE))/((self.s)*(self.g)) + self.s/2

            #Update inital conditions
            self.x0 = self.xt - self.left_foot_x
            self.v0 = self.vt

        elif self.support_leg == 'left':

            #Change leg 
            self.support_leg = 'right'

            #Update foot coordinates
            self.right_foot_x = self.left_foot_x + self.s

            #Update next exchange point
            self.x_support_exchange = self.right_foot_x + ((self.zc)*(self.deltaE))/((self.s)*(self.g)) + self.s/2

            #Update inital conditions
            self.x0 = self.xt - self.right_foot_x
            self.v0 = self.vt

        print("Time: %.2f"%self.t + "switched support to: " + self.support_leg)
        self.t = 0
        self.v0 = self.vt

if __name__ == '__main__':
    x0 = -0.5
    v0 = 3
    z0 = 2
    s = 1

    LIPM(x0,v0,z0,s,0.001)