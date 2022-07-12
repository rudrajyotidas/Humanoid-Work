import numpy as np

class LIPM3D:
    
    def __init__(self, dt=0.01, T_sup=1, zc=1, support_leg='left leg'):
        
        # Time
        self.t=0

        # Time step
        self.dt = dt

        # Time of support
        self.T_sup = T_sup

        # COM initial state of 3D LIPM equation wrt support foot
        self.x0 = 0
        self.y0 = 0
        self.vx0 = 0
        self.vy0 = 0

        # COM state at time t wrt support foot
        self.xt = 0
        self.yt = 0
        self.vxt = 0
        self.vyt = 0

        # Height of COM (remains constant for now)
        self.zc = zc

        # Current foot placements (x and y coordinates of support foot)
        self.px = 0
        self.py = 0

        # Foot placements of next step (x and y coordinates of the next step)
        # These are the modified foot placements after optimisation
        self.px_star = 0
        self.py_star = 0

        # Final state for current gait
        self.x_bar = 0
        self.y_bar = 0
        self.vx_bar = 0
        self.vy_bar = 0

        # Support Leg
        self.support_leg = support_leg

        # Walk Parameters (to be implemented later)

        # self.walk_parameters = []
        self.sx = 0
        self.sy = 0

        # Number of steps taken
        self.steps = 0

        # Utility constants (these constants show up many times)
        self.g = 9.8
        self.Tc = 0
        self.C = 0
        self.S = 0

        self.left_foot = [0,0,0]
        self.right_foot = [0,0,0]
        self.COM = [0,0,0]

    def init_model(self, COM_pos, COM_vel, left_leg, right_leg, sx, sy):
        self.COM = COM_pos
        self.left_foot = left_leg
        self.right_foot = right_leg

        self.zc = COM_pos[2]
        self.Tc = np.sqrt(self.zc/self.g)
        self.C = np.cosh(self.T_sup/self.Tc)
        self.S = np.sinh(self.T_sup/self.Tc)

        self.sx = sx
        self.sy = sy

        # Set initial conditions of equation depending on support leg
        if self.support_leg == 'left leg':
            self.px = left_leg[0]
            self.py = left_leg[1]

        elif self.support_leg == 'right leg':
            self.px = right_leg[0]
            self.py = right_leg[1]

        self.x0 = COM_pos[0] - self.px
        self.y0 = COM_pos[1] - self.py
        self.vx0 = COM_vel[0]
        self.vy0 = COM_vel[1]

        # Update next step location
        # self.finalNextFootLocation()

    def updateState(self):

        # Update time
        self.t = self.t + self.dt

        # If support time has elapsed, switch leg
        if self.t >= self.T_sup:
            self.switchLeg()

        Ct = np.cosh(self.t/self.Tc)
        St = np.sinh(self.t/self.Tc)

        # Update xCOM and yCOM
        self.xt = (self.x0)*Ct + (self.Tc)*(self.vx0)*St
        self.yt = (self.y0)*Ct + (self.Tc)*(self.vy0)*St

        # Update vCOM
        self.vxt = (self.x0)*St/self.Tc + self.vx0*Ct
        self.vyt = (self.y0)*St/self.Tc + self.vy0*Ct       

    def referenceFootNextStep(self, sx, sy, theta=0):
        
        # Theta kept to consider rotation
        # Will be implemented later

        # Assumes that the humanoid faces positive x direction

        # Calculate foot placements of next step
        if self.support_leg == 'left leg':
            px_ref = self.px + sx
            py_ref = self.py - sy

        elif self.support_leg == 'right leg':
            px_ref = self.px + sx
            py_ref = self.py + sy

        return px_ref, py_ref
    
    def finalDestinationState(self, sx, sy, theta=0):

        # Gives the desired final condition of next step

        # Actually, sx and sy of the NEXT step is needed, will be implemented later when I introduce walk parameters

        # Theta kept to consider rotation
        # Will be implemented later

        C = self.C
        S = self.S
        Tc = self.Tc

        px_ref, py_ref = self.referenceFootNextStep(sx, sy)

        # Note that we need to calculate x_bar and y_bar for the next step
        # So if the current foot is left, we use the expression for right
        if self.support_leg == 'left leg':
            x_bar = sx/2.0
            y_bar = sy/2.0

        if self.support_leg == 'right leg':
            x_bar = sx/2.0
            y_bar = -sy/2.0

        # Desired COM position, if we assume our walk trajectory to be a 3D LIPM trajectory
        # with initial position as halfway of step sizes and inital velocities zero
        xd = px_ref + x_bar
        yd = py_ref + y_bar

        # Desired COM velocities at the end of current step
        vx_bar = x_bar*(1+C)/(Tc*S)
        vy_bar = y_bar*(C-1)/(Tc*S)

        return xd, yd, vx_bar, vy_bar

    def finalNextFootLocation(self, a=1, b=1, theta=0):

        # if not self.walk_parameters:
        #     sx=0
        #     sy=0
        # else:
        #     (sx,sy) = self.walk_parameters.pop(0)

        xd, yd, vxd, vyd = self.finalDestinationState(self.sx, self.sy, theta)

        px_ref, py_ref = self.referenceFootNextStep(self.sx, self.sy)

        # Final state of current step, using analytical solution of 2D LIP
        xT = (self.x0)*(self.C) + (self.Tc)*(self.vx0)*(self.S) 
        yT = (self.y0)*(self.C) + (self.Tc)*(self.vy0)*(self.S)
        vxT = (self.x0)*(self.S)/(self.Tc) + self.vx0*self.C
        vyT = (self.y0)*(self.S)/(self.Tc) + self.vy0*self.C

        # Initial global state of next step (wrt origin), found from final state of current step
        x_i = xT + self.px
        y_i = yT + self.py
        vx_i = vxT
        vy_i = vyT

        # Find the modified foot placement by optimising error

        C = self.C
        S = self.S
        Tc = self.Tc
        D = a*(C - 1)**2 + b*(S/Tc)**2

        self.px_star = -a*(C-1)*(xd - C*x_i - Tc*S*vx_i)/D - b*S*(vxd - S*x_i/Tc - C*vx_i)/(Tc*D)
        self.py_star = -a*(C-1)*(yd - C*y_i - Tc*S*vy_i)/D - b*S*(vyd - S*y_i/Tc - C*vy_i)/(Tc*D)

        print("Reference: ", px_ref, py_ref, "Modified: ", self.px_star, self.py_star)

    def switchLeg(self):

        # Handles switching leg

        self.steps = self.steps + 1

        if self.support_leg == 'left leg':
            self.right_foot = [self.px_star, self.py_star, 0]

            COMx = self.xt + self.left_foot[0]
            COMy = self.yt + self.left_foot[1]

            self.x0 = COMx - self.right_foot[0]
            self.y0 = COMy - self.right_foot[1]

            self.px = self.px_star
            self.py = self.py_star
            self.support_leg = 'right leg'

        elif self.support_leg == 'right leg':
            self.left_foot = [self.px_star, self.py_star, 0]

            COMx = self.xt + self.right_foot[0]
            COMy = self.yt + self.right_foot[1]

            self.x0 = COMx - self.left_foot[0]
            self.y0 = COMy - self.left_foot[1]

            self.px = self.px_star
            self.py = self.py_star
            self.support_leg = 'left leg'

        print("Leg Swapped to ", self.support_leg, "coordinates: ", self.px, self.py)

        self.vx0 = self.vxt
        self.vy0 = self.vyt
        self.t = 0
        
        # Calulcate foot placement for next step, so that we can start swinging the leg
        # Although swinging isnt implemented here
        self.finalNextFootLocation()

