# %%
# %% [markdown]
#   Modeling a 3D Linear Inverted Pendulum
# %%
# Import necessary libraries
from math import sin
from math import cos
from math import pi
import numpy as np
import plotly.graph_objects as go
import time
import matplotlib.pyplot as plt

start = time.time()

# %%
# %% [markdown]
# Modeling a 3D Pendulum - Nonlinear
class PEND3D:
    def __init__(self, state_i, L, l, m, ts):
        self.theta, self.phi, self.theta_dot, self.phi_dot = state_i
        self.L = L # angular momentum in Z held as constant
        self.l = l # length of the pendulum
        self.m = m # mass of the pendulum
        self.g = 9.81 # acceleration due to gravity
        self.ts = ts # timestep of calculation
        self.t = 0 # time vector

    def update(self):
        # Differential Equation
        self.theta_ddot = sin(self.theta)*cos(self.theta)*(self.phi_dot**2) - (self.g / self.l)*sin(self.theta)
        self.phi_ddot = (-2 * self.L) / (self.l**2 * self.m) * self.theta_dot * cos(self.theta) / sin(self.theta)**3
        # Integration for states
        self.t += self.ts
        self.theta_dot = self.theta_dot + self.theta_ddot * self.ts
        self.phi_dot = self.phi_dot + self.phi_ddot * self.ts
        self.theta = self.theta + self.theta_dot * self.ts
        self.phi = self.phi + self.phi_dot * self.ts
        # Convert from spherical to cartesian
        self.x, self.y, self.z = self.sph_to_cart(self.theta, self.phi)
        self.x_dot, self.y_dot, self.z_dot = self.sph_to_cart_dot(self.theta, self.phi, self.theta_dot, self.phi_dot)
    
    # Convert to cartesian position coordinates
    def sph_to_cart(self, theta, phi):
        x = self.l * sin(theta) * cos(phi)
        y = self.l * sin(theta) * sin(phi)
        z = -self.l * cos(theta)

        return x, y, z
    # Convert to cartesian velocity coordinates
    def sph_to_cart_dot(self, theta, phi, theta_dot, phi_dot):
        x_dot = self.l*(theta_dot*cos(theta)*cos(phi) - phi_dot*sin(theta)*sin(phi))
        y_dot = self.l*(theta_dot*sin(theta)*cos(phi) + phi_dot*sin(theta)*cos(phi))
        z_dot = -self.l*theta_dot*sin(theta)

        return x_dot, y_dot, z_dot

# %% Math & Calculation
if __name__ == '__main__':
    t, theta, phi, theta_dot, phi_dot, x, y, z, x_dot, y_dot, z_dot = ([] for _ in range(11))

    ts = 0.1
    t_span = 10 

    state_i = [pi/6, 0, 0, 2] # theta = pi/6, phi = 0, theta_dot = 0, phi_dot = 2
    L = 50
    l = 10
    m = 1

    model = PEND3D(state_i, L, l, m, ts)
    
    n = round(t_span / ts)
    for i in range(1,n+1):
        model.update()
        t.append(model.t)
        x.append(model.x)
        y.append(model.y)
        z.append(model.z)
        x_dot.append(model.x_dot)
        y_dot.append(model.y_dot)
        z_dot.append(model.z_dot)


    # %%  Running an animation

    # Create figure
    fig = go.Figure(go.Scatter3d(x=[], y=[], z=[],
                                mode="markers",
                                marker=dict(color="red", size=2)
                                )
                    )
    # Add black line trace from origin to current point
    fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                                mode='lines',
                                line=dict(color='black', width=2)
                                )
                    )
    
    # Frames
    frames = [go.Frame(data=[go.Scatter3d(x=x[:k+1],
                                        y=y[:k+1],
                                        z=z[:k+1]
                                        ),
                            # Add black line trace from origin
                             go.Scatter3d(x=[0, x[k]],
                                        y=[0, y[k]],
                                        z=[0, z[k]],
                                        )
                            ],
                    traces= [0, 1],
                    name=f'frame{k}'      
                    )for k  in  range(len(x)-1)
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

    fig.update_layout(scene = dict(xaxis=dict(range=[-l, l], autorange=False),
                                yaxis=dict(range=[-l, l], autorange=False),
                                zaxis=dict(range=[-l, l], autorange=False)
                                )
                    )

    fig.update_layout(sliders=sliders)
    fig.layout.scene.aspectratio = {'x':1, 'y':1, 'z':1}
    fig.show()

    end = time.time()
    print('Time elasped:' + str(end-start) + ' seconds')

   
   # Plotting some figures
    # %%
    # Plot the pendulum figure
    fig1 = plt.figure(figsize=(12, 10))
    ax = fig1.add_subplot(111, xlim=(-l, l), ylim=(-l, l), zlim=(-l, l), projection='3d')
    ax.plot(0, 0, 0, color='black', marker='o')  # Black dot for origin
    ax.plot([0, x[0]], [0, y[0]], [0, z[0]], color='blue')  # Pendulum arm at t = 0
    ax.plot(x[0], y[0], z[0], color='red', marker='o')  # Red dot for start point
    ax.plot(x, y, z, 'r-') # Pendulum mass trjaectory
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    ax.set_zlabel('Position Z')
    plt.title('Spherical Pendulum Motion')

    # Plot the resulting states
    fig2 = plt.figure(figsize=(12, 6))
    bx = fig2.add_subplot(321)
    bx.grid(ls='--')
    bx.plot(t,x)
    #x_ani, = bx.plot(t[0], x[0], marker='o', color='r')
    plt.xlabel('time (s)')
    plt.ylabel('Position X')

    cx = fig2.add_subplot(323)
    cx.grid(ls='--')
    cx.plot(t,y)
    #y_ani, = cx.plot(t[0], y[0], marker='o', color='r')
    plt.xlabel('time (s)')
    plt.ylabel('Position Y')

    dx = fig2.add_subplot(325)
    dx.grid(ls='--')
    dx.plot(t,z)
    #z_ani, = dx.plot(t[0], z[0], marker='o', color='r')
    plt.xlabel('time (s)')
    plt.ylabel('Position Z')

    ex = fig2.add_subplot(322)
    ex.grid(ls='--')
    ex.plot(t,x_dot)
    #x_dot_ani, = ex.plot(t[0], x_dot[0], marker='o', color='r')
    plt.xlabel('time (s)')
    plt.ylabel('Velocity X')

    fx = fig2.add_subplot(324)
    fx.grid(ls='--')
    fx.plot(t,y_dot)
    #y_dot_ani, = fx.plot(t[0], y_dot[0], marker='o', color='r')
    plt.xlabel('time (s)')
    plt.ylabel('Velocity Y')

    gx = fig2.add_subplot(326)
    gx.grid(ls='--')
    gx.plot(t,z_dot)
    #z_dot_ani, = gx.plot(t[0], z_dot[0], marker='o', color='r')
    plt.xlabel('time (s)')
    plt.ylabel('Velocity Z')

    #ani_pendulum = animation.FuncAnimation(fig=fig1, init_func=initAnimation, func=animate, frames=range(1, n), interval=1.0/ts, blit=False)

    plt.show()