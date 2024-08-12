# -*-coding:UTF-8 -*-
from math import sin
from math import cos
from math import pi
import numpy as np
import plotly.graph_objects as go
import time

start = time.time()
# Definition of 3D Linear Inverted Pendulum
class Stepper3D:
    def __init__(self,
                 dt=0.001,
                 T_sup=1.0,
                 support_leg='left_leg'):
        self.dt = dt
        self.t = 0
        self.T_sup = T_sup # support time

        self.p_x = 0  # desired foot location x
        self.p_y = 0  # desired foot location y

        self.p_x_star = 0 # modified foot location x
        self.p_y_star = 0 # modified foot location y

        # Initialize the gait parameters
        self.s_x = 0.0
        self.s_y = 0.0

        # COM initial state
        self.x_0 = 0
        self.vx_0 = 0
        self.y_0 = 0
        self.vy_0 = 0

        # COM real-time state
        self.x_t = 0
        self.vx_t = 0
        self.y_t = 0
        self.vy_t = 0

        # COM desired state
        self.x_d = 0
        self.vx_d = 0
        self.y_d = 0
        self.vy_d = 0

        # final state for one gait unit
        self.bar_x = 0.0
        self.bar_y = 0.0
        self.bar_vx = 0.0
        self.bar_vy = 0.0

        self.support_leg = support_leg
        self.left_foot_pos = [0.0, 0.0, 0.0]
        self.right_foot_pos = [0.0, 0.0, 0.0]
        self.COM_pos = [0.0, 0.0, 0.0]
    
    def initializeModel(self, COM_pos, left_foot_pos, right_foot_pos):
        self.COM_pos = COM_pos
        self.left_foot_pos = left_foot_pos
        self.right_foot_pos = right_foot_pos

        self.zc = self.COM_pos[2]
        self.T_c = np.sqrt(self.zc / 9.81) # set gravity parameter as 9.81
        self.C = np.cosh(self.T_sup/self.T_c)
        self.S = np.sinh(self.T_sup/self.T_c)
    
    def updateParameters(self, T_sup):
        self.T_sup = T_sup
        self.C = np.cosh(self.T_sup/self.T_c)
        self.S = np.sinh(self.T_sup/self.T_c)

    def step(self):
        self.t += self.dt
        t = self.t
        T_c = self.T_c

        self.x_t = self.x_0*np.cosh(t/T_c) + T_c*self.vx_0*np.sinh(t/T_c)
        self.vx_t = self.x_0/T_c*np.sinh(t/T_c) + self.vx_0*np.cosh(t/T_c)

        self.y_t = self.y_0*np.cosh(t/T_c) + T_c*self.vy_0*np.sinh(t/T_c)
        self.vy_t = self.y_0/T_c*np.sinh(t/T_c) + self.vy_0*np.cosh(t/T_c)

    def calculateXtVt(self, t):
        T_c = self.T_c

        x_t = self.x_0*np.cosh(t/T_c) + T_c*self.vx_0*np.sinh(t/T_c)
        vx_t = self.x_0/T_c*np.sinh(t/T_c) + self.vx_0*np.cosh(t/T_c)

        y_t = self.y_0*np.cosh(t/T_c) + T_c*self.vy_0*np.sinh(t/T_c)
        vy_t = self.y_0/T_c*np.sinh(t/T_c) + self.vy_0*np.cosh(t/T_c)

        return x_t, vx_t, y_t, vy_t

    def nextReferenceFootLocation(self, s_x, s_y, theta=0):
        if self.support_leg is 'left_leg': # then the next support leg is the right leg
            p_x_new = self.p_x + np.cos(theta)*s_x - np.sin(theta)*s_y
            p_y_new = self.p_y + np.sin(theta)*s_x + np.cos(theta)*s_y
        elif self.support_leg is 'right_leg': # then the next support leg is the left leg
            p_x_new = self.p_x + np.cos(theta)*s_x + np.sin(theta)*s_y
            p_y_new = self.p_y + np.sin(theta)*s_x - np.cos(theta)*s_y

        return p_x_new, p_y_new

    def nextState(self, s_x, s_y, theta=0):
        '''
        Calculate next final state at T_sup
        '''
        if self.support_leg is 'left_leg':
            bar_x_new = np.cos(theta)*s_x/2.0 - np.sin(theta)*s_y/2.0
            bar_y_new = np.sin(theta)*s_x/2.0 + np.cos(theta)*s_y/2.0
        elif self.support_leg is 'right_leg':
            bar_x_new = np.cos(theta)*s_x/2.0 + np.sin(theta)*s_y/2.0
            bar_y_new = np.sin(theta)*s_x/2.0 - np.cos(theta)*s_y/2.0
        return bar_x_new, bar_y_new

    def nextVel(self, bar_x=0, bar_y=0, theta=0):
        C = self.C
        S = self.S
        T_c = self.T_c

        bar_vx_new = np.cos(theta)*(1+C)/(T_c*S)*bar_x - np.sin(theta)*(C-1)/(T_c*S)*bar_y
        bar_vy_new = np.sin(theta)*(1+C)/(T_c*S)*bar_x + np.cos(theta)*(C-1)/(T_c*S)*bar_y

        return bar_vx_new, bar_vy_new

    def targetState(self, p_x, bar_x, bar_vx):
        x_d = p_x + bar_x
        vx_d = bar_vx

        return x_d, vx_d
    
    def modifiedFootLocation(self, a=1.0, b=1.0, x_d=0, vx_d=0, x_0=0, vx_0=0):
        C = self.C
        S = self.S
        T_c = self.T_c
        D = a*(C - 1)**2 + b*(S/T_c)**2

        p_x_star = -a*(C-1)*(x_d - C*x_0 - T_c*S*vx_0)/D - b*S*(vx_d - S*x_0/T_c - C*vx_0)/(T_c*D)

        return p_x_star

    def calculateFootLocationForNextStep(self, s_x=0.0, s_y=0.0, a=1.0, b=1.0, theta=0.0, x_0=0.0, vx_0=0.0, y_0=0.0, vy_0=0.0):
        self.s_x = s_x
        self.s_y = s_y

        # ----------------------------- calculate desired COM states and foot locations for the given s_x, s_y and theta
        # calculate desired foot locations
        print(self.p_x, self.p_y)
        p_x_new, p_y_new = self.nextReferenceFootLocation(s_x, s_y, theta)
        # print('-- p_x_new=%.3f'%p_x_new, ', p_y_new=%.3f'%p_y_new)

        # calculate desired COM states
        bar_x, bar_y = self.nextState(s_x, s_y, theta)
        bar_vx, bar_vy = self.nextVel(bar_x, bar_y, theta)
        # print('-- bar_x=%.3f'%bar_x, ', bar_y=%.3f'%bar_y)
        # print('-- bar_vx=%.3f'%bar_vx, ', bar_vy=%.3f'%bar_vy)

        # calculate target COM state in the next step
        self.x_d, self.vx_d = self.targetState(p_x_new, bar_x, bar_vx)
        self.y_d, self.vy_d = self.targetState(p_y_new, bar_y, bar_vy)
        # print('-- x_d=%.3f'%self.x_d, ', vx_d=%.3f'%self.vx_d)
        # print('-- y_d=%.3f'%self.y_d, ', vy_d=%.3f'%self.vy_d)

        # ----------------------------- calculate modified foot locations based on the current actual COM states
        # correct the modified foot locations to minimize the errors
        self.p_x_star = self.modifiedFootLocation(a, b, self.x_d, self.vx_d, x_0, vx_0)
        self.p_y_star = self.modifiedFootLocation(a, b, self.y_d, self.vy_d, y_0, vy_0)
        # print('-- p_x_star=%.3f'%self.p_x_star, ', p_y_star=%.3f'%self.p_y_star)

    def switchSupportLeg(self):
        if self.support_leg is 'left_leg':
            # print('\n---- switch the support leg to the right leg')
            self.support_leg = 'right_leg'
            COM_pos_x = self.x_t + self.left_foot_pos[0]
            COM_pos_y = self.y_t + self.left_foot_pos[1]
            self.x_0 = COM_pos_x - self.right_foot_pos[0]
            self.y_0 = COM_pos_y - self.right_foot_pos[1]
        elif self.support_leg is 'right_leg':
            # print('\n---- switch the support leg to the left leg')
            self.support_leg = 'left_leg'
            COM_pos_x = self.x_t + self.right_foot_pos[0]
            COM_pos_y = self.y_t + self.right_foot_pos[1]
            self.x_0 = COM_pos_x - self.left_foot_pos[0]
            self.y_0 = COM_pos_y - self.left_foot_pos[1]

        self.t = 0
        self.vx_0 = self.vx_t
        self.vy_0 = self.vy_t

# Calculating a forward trajectory
COM_pos_x, COM_pos_y, COM_pos_z = ([] for _ in range(3))
left_foot_pos_x, left_foot_pos_y, left_foot_pos_z = ([] for _ in range(3))
right_foot_pos_x, right_foot_pos_y, right_foot_pos_z = ([] for _ in range(3))
support_foot_pos_x, support_foot_pos_y, support_foot_pos_z = ([] for _ in range(3))

# Initialize the COM position and velocity
COM_pos_0 = [-0.4, 0.2, 1.0]
COM_v0 = [1.0, -0.01]
zc = COM_pos_0[2]

# Initialize the foot positions
left_foot_pos = [-0.2, 0.3, 0]
right_foot_pos = [0.2, -0.3, 0]

delta_t = 0.05

s_x = 0.5
s_y = 0.4
a = 1.0
b = 1.0
theta = 0.0

LIPM_model = Stepper3D(dt=delta_t, T_sup=0.5)
LIPM_model.initializeModel(COM_pos_0, left_foot_pos, right_foot_pos)


LIPM_model.support_leg = 'left_leg' # set the support leg to right leg in next step
if LIPM_model.support_leg is 'left_leg':
    support_foot_pos = LIPM_model.left_foot_pos
    LIPM_model.p_x = LIPM_model.left_foot_pos[0]
    LIPM_model.p_y = LIPM_model.left_foot_pos[1]
else:
    support_foot_pos = LIPM_model.right_foot_pos
    LIPM_model.p_x = LIPM_model.right_foot_pos[0]
    LIPM_model.p_y = LIPM_model.right_foot_pos[1]

LIPM_model.x_0 = LIPM_model.COM_pos[0] - support_foot_pos[0]
LIPM_model.y_0 = LIPM_model.COM_pos[1] - support_foot_pos[1]
LIPM_model.vx_0 = COM_v0[0]
LIPM_model.vy_0 = COM_v0[1]

step_num = 0
total_time = 10 # seconds
global_time = 0

swing_data_len = int(LIPM_model.T_sup/delta_t)
swing_foot_pos = np.zeros((swing_data_len, 3))
j = 0

switch_index = swing_data_len

for i in range(int(total_time/delta_t)):
    global_time += delta_t

    LIPM_model.step()

    if step_num >= 1:
        if LIPM_model.support_leg is 'left_leg':
            LIPM_model.right_foot_pos = [swing_foot_pos[j,0], swing_foot_pos[j,1], swing_foot_pos[j,2]]
        else:
            LIPM_model.left_foot_pos = [swing_foot_pos[j,0], swing_foot_pos[j,1], swing_foot_pos[j,2]]
        j += 1

    # record data
    COM_pos_x.append(LIPM_model.x_t + support_foot_pos[0])
    COM_pos_y.append(LIPM_model.y_t + support_foot_pos[1])
    COM_pos_z.append(LIPM_model.zc)
    support_foot_pos_x.append(support_foot_pos[0])
    support_foot_pos_y.append(support_foot_pos[1])
    support_foot_pos_z.append(support_foot_pos[2])
    left_foot_pos_x.append(LIPM_model.left_foot_pos[0])
    left_foot_pos_y.append(LIPM_model.left_foot_pos[1])
    left_foot_pos_z.append(LIPM_model.left_foot_pos[2])
    right_foot_pos_x.append(LIPM_model.right_foot_pos[0])
    right_foot_pos_y.append(LIPM_model.right_foot_pos[1])  
    right_foot_pos_z.append(LIPM_model.right_foot_pos[2])

    # switch the support leg
    if (i > 0) and (i % switch_index == 0):
        j = 0

        LIPM_model.switchSupportLeg() # switch the support leg
        step_num += 1

        # theta -= 0.04 # set zero for walking forward, set non-zero for turn left and right

        if step_num >= 5: # stop forward after 5 steps
            s_x = 0.0

        if step_num >= 10:
            s_y = 0.0

        if LIPM_model.support_leg is 'left_leg':
            support_foot_pos = LIPM_model.left_foot_pos
            LIPM_model.p_x = LIPM_model.left_foot_pos[0]
            LIPM_model.p_y = LIPM_model.left_foot_pos[1]
        else:
            support_foot_pos = LIPM_model.right_foot_pos
            LIPM_model.p_x = LIPM_model.right_foot_pos[0]
            LIPM_model.p_y = LIPM_model.right_foot_pos[1]

        # calculate the next foot locations, with modification, stable
        x_0, vx_0, y_0, vy_0 = LIPM_model.calculateXtVt(LIPM_model.T_sup) # calculate the xt and yt as the initial state for next step

        if LIPM_model.support_leg is 'left_leg':
            x_0 = x_0 + LIPM_model.left_foot_pos[0] # need the absolute position for next step
            y_0 = y_0 + LIPM_model.left_foot_pos[1] # need the absolute position for next step
        else:
            x_0 = x_0 + LIPM_model.right_foot_pos[0] # need the absolute position for next step
            y_0 = y_0 + LIPM_model.right_foot_pos[1] # need the absolute position for next step

        LIPM_model.calculateFootLocationForNextStep(s_x, s_y, a, b, theta, x_0, vx_0, y_0, vy_0)
        # print('p_star=', LIPM_model.p_x_star, LIPM_model.p_y_star)

        # calculate the foot positions for swing phase
        if LIPM_model.support_leg is 'left_leg':
            right_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]
            swing_foot_pos[:,0] = np.linspace(LIPM_model.right_foot_pos[0], right_foot_target_pos[0], swing_data_len)
            swing_foot_pos[:,1] = np.linspace(LIPM_model.right_foot_pos[1], right_foot_target_pos[1], swing_data_len)
            swing_foot_pos[1:swing_data_len-1, 2] = 0.1
        else:
            left_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]
            swing_foot_pos[:,0] = np.linspace(LIPM_model.left_foot_pos[0], left_foot_target_pos[0], swing_data_len)
            swing_foot_pos[:,1] = np.linspace(LIPM_model.left_foot_pos[1], left_foot_target_pos[1], swing_data_len)
            swing_foot_pos[1:swing_data_len-1, 2] = 0.1
# %%
# %%  Running an animation
# Create figure
# Plot the main pendulum arm from support to CoM
fig = go.Figure(go.Scatter3d(x=[], y=[], z=[],
                            mode="lines+markers",
                            marker=dict(color="red", size=5),
                            line = dict(color="black", width=3)
                            )
                )
# Add red trajectory line for CoM
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="red", size=2),
                            line = dict(color="red", width=3)
                            
                            )
                )
# Add blue markers for the left foot position
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="blue", size=2),
                            line = dict(color="blue", width=3)
                            
                            )
                )
# Add purple markers for the right foot position
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="purple", size=2),
                            line = dict(color="purple", width=3)
                            )
                )

# Frames                # Add blac lines+ red markers for pendulum arm
frames = [go.Frame(data=[go.Scatter3d(x=[support_foot_pos_x[k], COM_pos_x[k]],
                                    y=[support_foot_pos_y[k], COM_pos_y[k]],
                                    z=[support_foot_pos_z[k], COM_pos_z[k]]
                                    ),
                        # Add red lines+markers for CoM trajectory
                        go.Scatter3d(x=COM_pos_x[:k+1],
                                        y=COM_pos_y[:k+1],
                                        z=COM_pos_z[:k+1]
                                        ),
                        # Add blue lines+markers for left foot
                        go.Scatter3d(x=left_foot_pos_x[:k+1],
                                        y=left_foot_pos_y[:k+1],
                                        z=left_foot_pos_z[:k+1]
                                        ),
                        # Add purple lines+marker for right foot
                        go.Scatter3d(x=right_foot_pos_x[:k+1],
                                        y=right_foot_pos_y[:k+1],
                                        z=right_foot_pos_z[:k+1]
                                        ),
                        ],
                traces= [0, 1, 2, 3],
                name=f'frame{k}'
                )for k  in  range(len(COM_pos_x)-1)
        ]

fig.update(frames=frames)

def frame_args(duration):
    return {
            "frame": {"duration": duration},
            "mode": "immediate",
            "fromcurrent": True,
            "transition": {"duration": duration, "easing": "linear"},
            }


sliders = [
    {"pad": {"b": 10, "t": 60},
    "len": 0.9,
    "x": 0.1,
    "y": 0,
    
    "steps": [
                {"args": [[f.name], frame_args(0)],
                "label": str(k),
                "method": "animate",
                } for k, f in enumerate(fig.frames)
            ]
    }
        ]

fig.update_layout(

    updatemenus = [{"buttons":[
                    {
                        "args": [None, frame_args(20)],
                        "label": "Play", 
                        "method": "animate",
                    },
                    {
                        "args": [[None], frame_args(0)],
                        "label": "Pause", 
                        "method": "animate",
                }],
                    
                "direction": "left",
                "pad": {"r": 10, "t": 70},
                "type": "buttons",
                "x": 0.1,
                "y": 0,
            }
        ],
        sliders=sliders
    )

fig.update_layout(scene = dict(xaxis=dict(range=[-1, 5], autorange=False),
                            yaxis=dict(range=[-1, 1], autorange=False),
                            zaxis=dict(range=[0 , zc+1], autorange=False)
                            )
                )

fig.update_layout(sliders=sliders)
fig.layout.scene.aspectratio = {'x':1, 'y':1, 'z':1}
fig.show()

end = time.time()

print('Time Elapsed:' + str(end-start) + ' seconds')
