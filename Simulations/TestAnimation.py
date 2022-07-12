import matplotlib.pyplot as plt
import matplotlib.animation as animation
from LIPM import LIPM

#Initialise model with model parameters
x0 = -1.0
v0 = 5
z0 = 1
s = 2
delta_t = 0.01

model = LIPM(x0,v0,z0,s,delta_t)

#Lists that will store data to plot
xt_arr = list()
vt_array = list()
COM_x = list()
COM_z = list()
ll_x_array = list()
ll_z_array = list()
rl_x_array = list()
rl_z_array = list()

#Generate data by running model
for i in range(1000):
    
    COM_x.append(model.xt)
    COM_z.append(model.zc)

    ll_z_array.append(0)
    ll_x_array.append(model.left_foot_x)

    rl_z_array.append(0)
    rl_x_array.append(model.right_foot_x)

    model.updateState()

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-10, 10), ylim=(-0.5, 2))

LIPM_ll_animator, = ax.plot([], [], 'o-', lw=4, color='b')
LIPM_rl_animator, = ax.plot([], [], 'o-', lw=4, color='k')
LIPM_COM_animator, = ax.plot([], [], marker='o', markersize=20, color='r')

#Initiliasation function for animation
def init():
    LIPM_ll_animator.set_data([],[])
    LIPM_rl_animator.set_data([],[])
    LIPM_COM_animator.set_data(COM_x[0], COM_z[0])

    return [LIPM_ll_animator, LIPM_rl_animator, LIPM_COM_animator]

#The function that will be called again and again
def animate(i):
    ll_x = [COM_x[i], ll_x_array[i]]
    ll_y = [COM_z[i], ll_z_array[i]]

    rl_x = [COM_x[i], rl_x_array[i]]
    rl_y = [COM_z[i], rl_z_array[i]]

    LIPM_ll_animator.set_data(ll_x, ll_y)
    LIPM_rl_animator.set_data(rl_x, rl_y)
    LIPM_COM_animator.set_data(COM_x[i], COM_z[i])

    return [LIPM_ll_animator, LIPM_rl_animator, LIPM_COM_animator]

ani = animation.FuncAnimation(fig=fig, init_func=init, func=animate, frames=range(1, 1000), interval=1.0/delta_t, blit=True)

plt.show()