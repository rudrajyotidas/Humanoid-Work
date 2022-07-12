import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib import gridspec

from LIPM3D import LIPM3D

# Lists to store data as model runs
COMx = list()
COMy = list()
left_foot_X = list()
left_foot_Y = list()
left_foot_Z = list()
right_foot_X = list()
right_foot_Y = list()
right_foot_Z = list()

# Initial state of COM
COM_pos0 = [-0.4, 0.2, 1.0]
COM_v0 = [1, -0.1]

# Initial foot positions
left_foot_pos = [-0.2, 0.3, 0]
right_foot_pos = [0.2, -0.3, 0]

delta_t = 0.04

# For foot placement modification
a = 1.0
b = 1.0

# Step sizes
sx = 0.5
sy = 0.4

# Initialise the model
model = LIPM3D(dt = delta_t, T_sup=0.5)

model.init_model(COM_pos0, COM_v0, left_foot_pos, right_foot_pos, sx, sy)
model.px_star = right_foot_pos[0]
model.py_star = right_foot_pos[1]

simTime = 10
global_time = 0

#---------------------- Recording Data ---------------------------
for i in range(int(simTime/delta_t)):

    # Update time
    global_time = global_time + delta_t

    # Update state
    model.updateState()

    # Record data

    # Since xt is wrt planted foot, we add px and py to xt and yt respectively to record COM positions
    COMx.append(model.xt + model.px)
    COMy.append(model.yt + model.py)

    # Record foot locations
    # Need to implement a swing stage
    left_foot_X.append(model.left_foot[0])
    left_foot_Y.append(model.left_foot[1])
    left_foot_Z.append(0)
    right_foot_X.append(model.right_foot[0])
    right_foot_Y.append(model.right_foot[1])
    right_foot_Z.append(0)

    

#------------------- Animation Viewing Settings --------------------------

fig = plt.figure(figsize=(8, 10))

# One plot in 1st row, another plot in second row with the specified height ratios
spec = gridspec.GridSpec(nrows=2, ncols=1, height_ratios=[2.5, 1])

# The first plot will be a 3D plot for the legs and COM
ax = fig.add_subplot(spec[0], projection ='3d')

ax.set_xlim(-1.0, 4.0)
ax.set_ylim(-2.0, 2.0)
ax.set_zlim(-0.01, 2.0)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')

# View angle of the 3D plot
ax.view_init(20, -150)

# Second plot will be a 2D plot for the COM and foot positions
bx = fig.add_subplot(spec[1], autoscale_on=False)
bx.set_xlim(-1.0, 5.0)
bx.set_ylim(-0.8, 0.8)
bx.set_aspect('equal')
bx.set_xlabel('x (m)')
bx.set_ylabel('y (m)')
bx.grid(ls='--')


#------------------- Creating the animation ------------------#

# Create a Ball object for COM and foots
class Ball:
    def __init__(self, size=10, shape='o'):
        self.scatter, = ax.plot([], [], [], shape, markersize=size, animated=True)

    def update(self, pos):
        # Update position of ball in the plot
        self.scatter.set_data_3d(pos)

# Create a Line object for legs in 3D
class Line:
    def __init__(self, size=1, color='g'):
        self.line, = ax.plot([], [], [], linewidth=size, color=color, animated=True)

    def update(self, pos):
        # Update line in 3D plot
        self.line.set_xdata(pos[0])
        self.line.set_ydata(pos[1])
        self.line.set_3d_properties(np.asarray(pos[2]))

# Animator
class Animate3D:
    def __init__(self):
        self.origin = Ball(size=2, shape='ko')
        self.COM = Ball(size=16, shape='ro')
        self.COM_traj = Line(size=1, color='g')
        self.COM_loc = Ball(size=2, shape='ro')

        self.left_foot_loc = Ball(size=5, shape='bo')
        self.right_foot_loc = Ball(size=5, shape='mo')

        self.left_leg = Line(size=3, color='b')
        self.right_leg = Line(size=3, color='m')

    def updateAnim(self, COM_pos, COM_trajectory, lf_pos, rf_pos):

        self.origin.update([0,0,0])
        self.COM.update(COM_pos)
        self.COM_traj.update(COM_trajectory)

        # Current location is the last entry of COM_trajectory
        self.COM_loc.update(COM_trajectory[:,-1])

        self.left_foot_loc.update(lf_pos)
        self.right_foot_loc.update(rf_pos)

        leftleg_points = np.zeros((3,2))
        leftleg_points[:,0] = COM_pos
        leftleg_points[:,1] = lf_pos
        self.left_leg.update(leftleg_points)

        rightleg_points = np.zeros((3,2))
        rightleg_points[:,0] = COM_pos
        rightleg_points[:,1] = rf_pos
        self.right_leg.update(rightleg_points)

        artists = []
        artists.append(self.origin.scatter)
        artists.append(self.left_leg.line)
        artists.append(self.right_leg.line)
        artists.append(self.COM.scatter)
        artists.append(self.COM_traj.line)
        artists.append(self.COM_loc.scatter)
        artists.append(self.left_foot_loc.scatter)
        artists.append(self.right_foot_loc.scatter)

        if COM_pos[0] >= 3.0:
            ax.set_xlim(-1.0 + COM_pos[0] - 3.0, 4.0 + COM_pos[0] - 3.0)
        elif COM_pos[0] <= 0:
            ax.set_xlim(-1.0 + COM_pos[0], 4.0 + COM_pos[0])

        if COM_pos[1] >= 1.5:
            ax.set_ylim(-2 + COM_pos[1] - 1.5, 2 + COM_pos[1] - 1.5)
        elif COM_pos[1] <= -1.5:
            ax.set_ylim(-2 + COM_pos[1] + 1.5, 2 + COM_pos[1] + 1.5)

        ax.set_zlim(-0.01, 2.0)

        artists.append(ax)

        return artists

# Create Animator object
LIPM3D_Animator = Animate3D()

def ani_3D_init():
        return []

def ani_3D_update(i):
    COM_pos = [COMx[i], COMy[i], 1]
    COM_pos_trajectory = np.zeros((3, i))
    COM_pos_trajectory[0,:] = COMx[0:i]
    COM_pos_trajectory[1,:] = COMy[0:i]
    COM_pos_trajectory[2,:] = np.zeros((1,i))

    left_foot_pos = [left_foot_X[i], left_foot_Y[i], left_foot_Z[i]]
    right_foot_pos = [right_foot_X[i], right_foot_Y[i], right_foot_Z[i]]

    artists = LIPM3D_Animator.updateAnim(COM_pos, COM_pos_trajectory, left_foot_pos, right_foot_pos)

    return artists 

original_ani, = bx.plot(0, 0, marker='o', markersize=2, color='k')
left_foot_pos_ani, = bx.plot([], [], 'o', lw=2, color='b')
COM_traj_ani, = bx.plot(COMx[0], COMy[0], markersize=2, color='g')
COM_pos_ani, = bx.plot(COMx[0], COMy[0], marker='o', markersize=6, color='r')
left_foot_pos_ani, = bx.plot([], [], 'o', markersize=10, color='b')
right_foot_pos_ani, = bx.plot([], [], 'o', markersize=10, color='m')

COM_pos_str = 'COM = (%.2f, %.2f)'
ani_text_COM_pos = bx.text(0.05, 0.9, '', transform=bx.transAxes)

def ani_2D_init():
    COM_traj_ani.set_data(COMx[0:0], COMy[0:0])
    COM_pos_ani.set_data(COMx[0], COMy[0])
    left_foot_pos_ani.set_data(left_foot_X[0], left_foot_Y[0])
    right_foot_pos_ani.set_data(right_foot_X[0], right_foot_Y[0])

    return [COM_pos_ani, COM_traj_ani, left_foot_pos_ani, right_foot_pos_ani]

def ani_2D_update(i):
    COM_traj_ani.set_data(COMx[0:i], COMy[0:i])
    COM_pos_ani.set_data(COMx[i], COMy[i])
    left_foot_pos_ani.set_data(left_foot_X[i], left_foot_Y[i])
    right_foot_pos_ani.set_data(right_foot_X[i], right_foot_Y[i])

    ani_text_COM_pos.set_text(COM_pos_str % (COMx[i], COMy[i]))

    # # automatic set the x, y view limitation 
    # bx.set_xlim(-2.0 + COM_pos_x[i], 3.0 + COM_pos_x[i])
    # bx.set_ylim(-0.8 + COM_pos_y[i], 0.8 + COM_pos_y[i])

    return [COM_pos_ani, COM_traj_ani, left_foot_pos_ani, right_foot_pos_ani, ani_text_COM_pos, bx]

data_len = len(COMx)
ani_3D = FuncAnimation(fig, ani_3D_update, frames=range(1, data_len), init_func=ani_3D_init, interval=1.0/delta_t, blit=True, repeat=True)
ani_2D = FuncAnimation(fig=fig, init_func=ani_2D_init, func=ani_2D_update, frames=range(1, data_len), interval=1.0/delta_t, blit=True, repeat=True)

plt.show()
   

