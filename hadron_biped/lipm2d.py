# Modeling a 2D Linear Inverted Pendulum Stepper Model

# Import necessary libraries
from math import sin
from math import cos
from math import pi
import numpy as np
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import time

start = time.time()

# Model of a 2D inverted pendulum consists of a CoM attached by a massless telescopic leg
# with a pushing force, f, and an ankle torque, tau.

# Linearizing this pendulum results in an analytical solution for the trajectory

class LIPM2D:
    def __init__(self, state_i, f, tau, m, ts) -> None:
        self.theta, self.r, self.theta_dot, self.r_dot = state_i
        self.f = f
        self.tau = tau
        self.m = m
        self.g = 9.81
        self.ts = ts
        self.t = 0
        self.x, self.z = self.sph_to_cart(self.theta, self.r)
        self.x_dot, self.z_dot = self.sph_to_cart_dot(self.theta, self.r, self.theta_dot, self.r_dot)

    def update(self):
        # Differential Equation
        self.theta_ddot = ((self.tau/self.m) + self.g*self.r*sin(self.theta) - 2*self.r*self.r_dot*self.theta_dot) / self.r**2
        self.r_ddot = (self.f/self.m) + self.r*self.theta_dot**2 - self.g*cos(self.theta)
        # Integration for states
        self.t += self.ts
        self.theta_dot = self.theta_dot + self.theta_ddot * self.ts
        self.r_dot = self.r_dot + self.r_ddot * self.ts
        self.theta = self.theta + self.theta_dot * self.ts
        self.r = self.r + self.r_dot * self.ts
        # Convert from spherical to cartesian
        self.x, self.z = self.sph_to_cart(self.theta, self.r)
        self.x_dot, self.z_dot = self.sph_to_cart_dot(self.theta, self.r, self.theta_dot, self.r_dot)

    def sph_to_cart(self, theta, r):
        x = r*sin(theta)
        z = r*cos(theta)

        return x, z
    
    def sph_to_cart_dot(self, theta, r, theta_dot, r_dot):
        x_dot = r_dot*cos(theta) - r*theta_dot*sin(theta)
        z_dot = r_dot*sin(theta) + r*theta_dot*cos(theta)

        return x_dot, z_dot

if __name__ == '__main__':
    t, theta, r, theta_dot, r_dot, x, z, x_dot, z_dot = ([] for _ in range(9))

    f = 0
    tau = 0
    m = 1
    l = 1
    state_i = [pi/4, l, 0, 0] # theta = pi/6, r = 0, theta_dot = 0, r_dot = 2

    ts = 0.01
    t_span = 0.5
    n = round(t_span / ts)

    model = LIPM2D(state_i, f, tau, m, ts)
    
    for i in range(1, n+1):
        # Cases of Different Push Force: 0, Constant Leg Length, Fall Down & Accelerate Out, CoM Moves Right
        #model.f = 0
        #model.f = model.m*model.g*cos(model.theta) - model.m*model.r*model.theta_dot**2
        #model.f = model.m*model.g
        model.f = model.m*model.g/cos(model.theta)
        if model.z > 0:
            model.update()

        t.append(model.t)
        x.append(model.x)
        z.append(model.z)
        x_dot.append(model.x_dot)
        z_dot.append(model.z_dot)

    # %%  Running an animation

    # Create figure
    fig = go.Figure(go.Scatter(x=[], y=[],
                                mode="markers",
                                marker=dict(color="red", size=2)
                                )
                    )
    # Add black line trace from origin to current point
    fig.add_trace(go.Scatter(x=[0, x[0]],
                            y=[0, z[0]],
                            mode='lines',
                            line=dict(color='black', width=2)
                            )
                )
    
    # Frames
    frames = [go.Frame(data=[go.Scatter(x=x[:k+1],
                                        y=z[:k+1]
                                        ),
                            # Add black line trace from origin
                             go.Scatter(x=[0, x[k]],
                                        y=[0, z[k]],
                                        mode='lines',
                                        line=dict(color='black', width=2)
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
                                yaxis=dict(range=[-l, l], autorange=False)
                                )
                    )

    fig.update_layout(sliders=sliders)
    fig.layout.scene.aspectratio = {'x':1, 'y':1}
    fig.show()

    end = time.time()
    print('Time Elapsed: ' + str(end-start) + ' seconds')

   
   # Plotting some figures
    # %%

    # Plot the resulting states
    fig2 = plt.figure(figsize=(12, 6))
    bx = fig2.add_subplot(221)
    bx.grid(ls='--')
    bx.plot(t,x)
    plt.xlabel('Time (s)')
    plt.ylabel('Position X')

    dx = fig2.add_subplot(222)
    dx.grid(ls='--')
    dx.plot(t,z)
    plt.xlabel('Time (s)')
    plt.ylabel('Position Z')

    ex = fig2.add_subplot(223)
    ex.grid(ls='--')
    ex.plot(t,x_dot)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity X')


    gx = fig2.add_subplot(224)
    gx.grid(ls='--')
    gx.plot(t,z_dot)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity Z')

    #ani_pendulum = animation.FuncAnimation(fig=fig1, init_func=initAnimation, func=animate, frames=range(1, n), interval=1.0/ts, blit=False)

    plt.show()