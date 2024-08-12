# Pulled from /chauby/BipedalWalkingRobots/LIPM/LIPM.py
# This is a 2D Bipedal Walking Pattern Generator based on LIPM model
from math import sqrt
from math import sinh
from math import cosh
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Deinition of Linear Inverted Pendulum
class Stepper2D:
    def __init__(self, x0 = 0, v0 = 0, z0 = 1, delta_t = 0.001):
        self.x0 = x0
        self.v0 = v0 
        self.z0 = z0

        self.x = x0
        self.zc = z0 # z constant

        self.delta_t = delta_t
        self.xt = 0
        self.zt = 0
        self.vt = 0
        self.g = 9.8 # gravity acceleration constant
        self.mass = 10.0
        self.acc = 0.0
        self.Tc = sqrt(self.zc / self.g)
        self.t = 0

        self.left_foot_x = 0
        self.left_foot_z = 0
        self.right_foot_x = 0
        self.right_foot_z = 0
        self.support_leg = 'left_leg' # left_leg or right_leg

        self.orbital_energy = 0
        self.target_orbital_energy = 0
        self.capture_point = 0

        self.step_length = 0
        self.xf = 0
        self.vf = 0

    def update(self):
        self.t += self.delta_t
        self.xt, self.vt = self.updateXtVt(self.x0, self.v0, self.t)
        self.zt = self.zc
        self.orbital_energy = self.updateOrbitalEnergy(self.xt, self.vt)

    # update xt and vt at time t
    def updateXtVt(self, x0, v0, t):
        tau = t/self.Tc
        xt = x0*cosh(tau) + self.Tc*v0*sinh(tau)
        vt = (x0/self.Tc)*sinh(tau) + v0*cosh(tau)
        return xt, vt

    # update the orbital energy, for each single cycle, the orbital energy keeps constant
    def updateOrbitalEnergy(self, x, v):
        orbital_energy = (v**2 - (self.g/(self.zc))*x**2)/2
        return orbital_energy

    # update the capture point associated with the target orbital energy
    def updateCapturePoint(self, xt, vt, target_OE):
        if(vt**2 < 2*target_OE):
            capture_point = self.capture_point
            print("warning: calculate the capture point failed, the velocity is too low. caputre_point=", capture_point)
        else:
            capture_point = xt + np.sign(vt)*sqrt((vt**2 - 2*target_OE)*self.zc/self.g)
        return capture_point

    # switch the support leg for continue walking
    def switchSupportLeg(self):
        if self.support_leg == 'left_leg':
            self.x0 = self.left_foot_x + self.xt - self.right_foot_x
            self.support_leg = 'right_leg'
        elif self.support_leg == 'right_leg':
            self.x0 = self.right_foot_x + self.xt - self.left_foot_x
            self.support_leg = 'left_leg'
        else:
            print('Error: support leg is a wrong value:', self.support_leg)

        print("t = %.2f" % self.t + ", switch the support leg to " + self.support_leg)
        self.t = 0 # reset t
        self.v0 = self.vt

# functions definition for animation
def initAnimation():
    LIPM_left_leg_ani.set_data([], [])
    LIPM_right_leg_ani.set_data([], [])
    LIPM_COM_ani.set_data(COM_x[0], COM_z[0])
    COM_pos.set_text('')

    xt_point_ani.set_data(0, xt_array[0])
    vt_point_ani.set_data(0, vt_array[0])

    return [LIPM_left_leg_ani, LIPM_right_leg_ani, LIPM_COM_ani, COM_pos, xt_point_ani, vt_point_ani]

def animate(index):
    left_leg_x = [COM_x[index], left_foot_x_array[index]]
    left_leg_y = [COM_z[index], left_foot_z_array[index]]
    right_leg_x = [COM_x[index], right_foot_x_array[index]]
    right_leg_y = [COM_z[index], right_foot_z_array[index]]

    LIPM_left_leg_ani.set_data(left_leg_x, left_leg_y)
    LIPM_right_leg_ani.set_data(right_leg_x, right_leg_y)
    LIPM_COM_ani.set_data(COM_x[index], COM_z[index])
    COM_pos.set_text(COM_str % (COM_x[index], COM_z[index]))

    xt_point_ani.set_data(index, xt_array[index])
    vt_point_ani.set_data(index, vt_array[index])

    return [LIPM_left_leg_ani, LIPM_right_leg_ani, LIPM_COM_ani, COM_pos, xt_point_ani, vt_point_ani]

orbital_energy_array = list()
xt_array = list()
vt_array = list()
COM_x = list()
COM_z = list()
left_foot_x_array = list()
left_foot_z_array = list()
right_foot_x_array = list()
right_foot_z_array = list()

x0 = 0.0
v0 = 0.5
z0 = 1.0
delta_t = 0.02
LIPM_model = Stepper2D(x0, v0, z0, delta_t) # x0, v0, z, delta_t
LIPM_model.target_orbital_energy = 2

data_len = 200
switch_index = 40

j = 0
swing_foot_reference_x = [0]
for i in range(1, data_len+1):
    LIPM_model.update()

    if i <= switch_index: # first step
        LIPM_model.capture_point = LIPM_model.updateCapturePoint(LIPM_model.xt, LIPM_model.vt, LIPM_model.target_orbital_energy)
        if LIPM_model.support_leg == 'left_leg':
            LIPM_model.right_foot_x = LIPM_model.left_foot_x + LIPM_model.capture_point
            # record the COM position
            COM_x.append(LIPM_model.xt + LIPM_model.left_foot_x)
            COM_z.append(LIPM_model.zt)
        else:
            LIPM_model.left_foot_x = LIPM_model.right_foot_x + LIPM_model.capture_point
            # record the COM position
            COM_x.append(LIPM_model.xt + LIPM_model.right_foot_x)
            COM_z.append(LIPM_model.zt)
    else: # continue walking after the first step
        if LIPM_model.support_leg == 'left_leg':
            LIPM_model.right_foot_x = swing_foot_reference_x[j]
            # record the COM position
            COM_x.append(LIPM_model.xt + LIPM_model.left_foot_x)
            COM_z.append(LIPM_model.zt)
        else:
            LIPM_model.left_foot_x = swing_foot_reference_x[j]
            # record the COM position
            COM_x.append(LIPM_model.xt + LIPM_model.right_foot_x)
            COM_z.append(LIPM_model.zt)
        j = j + 1 # update the index for reference swing foot position

    # switch the support leg and calculate necessary information
    if((i >= switch_index) and (i % switch_index == 0)):
        LIPM_model.switchSupportLeg()

        # calculate the capture point at the time of the next support leg switch
        xt, vt = LIPM_model.updateXtVt(LIPM_model.x0, LIPM_model.v0, LIPM_model.delta_t*(switch_index))
        orbital_energy = LIPM_model.updateOrbitalEnergy(xt, vt)
        capture_point = LIPM_model.updateCapturePoint(xt, vt, LIPM_model.target_orbital_energy)

        # interpolation the reference foot trajectory of the swing leg from current position to the next capture point
        if LIPM_model.support_leg == 'left_leg':
            swing_foot_reference_x = np.linspace(LIPM_model.right_foot_x, LIPM_model.left_foot_x + capture_point, switch_index)
        else:
            swing_foot_reference_x = np.linspace(LIPM_model.left_foot_x, LIPM_model.right_foot_x + capture_point, switch_index)
        j = 0

    # record the data
    xt_array.append(LIPM_model.xt)
    vt_array.append(LIPM_model.vt)
    left_foot_x_array.append(LIPM_model.left_foot_x)
    left_foot_z_array.append(LIPM_model.left_foot_z)
    right_foot_x_array.append(LIPM_model.right_foot_x)
    right_foot_z_array.append(LIPM_model.right_foot_z)
    orbital_energy_array.append(LIPM_model.orbital_energy)

# plot the animation
fig = plt.figure(figsize=(12, 6))
ax = fig.add_subplot(311, autoscale_on=False, xlim=(-1, 9), ylim=(-0.1, 1.2))
# ax.set_aspect('equal')
ax.grid(ls='--')

LIPM_left_leg_ani, = ax.plot([], [], 'o-', lw=4, color='b')
LIPM_right_leg_ani, = ax.plot([], [], 'o-', lw=4, color='k')
LIPM_COM_ani, = ax.plot(COM_x[0], COM_z[0], marker='o', markersize=20, color='r')

COM_str = 'COM = (%.1f, %.1f)'
COM_pos = ax.text(0.05, 0.9, '', transform=ax.transAxes)

bx = fig.add_subplot(312, autoscale_on=False, xlim=(0, data_len), ylim=(-2, 2))
bx.grid(ls='--')
bx.plot(xt_array)
xt_point_ani, = bx.plot(0, xt_array[0], marker='o', color='r')
plt.xlabel('time (s)')
plt.ylabel('COM position x')

cx = fig.add_subplot(313, autoscale_on=False, xlim=(0, data_len), ylim=(0, 5))
cx.grid(ls='--')
cx.plot(vt_array)
vt_point_ani, = cx.plot(0, vt_array[0], marker='o', color='r')
plt.xlabel('time (s)')
plt.ylabel('COM velocity')


ani_LIPM = animation.FuncAnimation(fig=fig, init_func=initAnimation, func=animate, frames=range(1, data_len), interval=1.0/delta_t, blit=True)
# ani_LIPM.save('LIPM_double_legs.gif', writer='imagemagick')

plt.show()