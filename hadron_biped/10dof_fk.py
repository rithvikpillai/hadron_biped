from math import pi
from math import sin
from math import cos
from math import radians
import numpy as np
import plotly.graph_objects as go
import time

start = time.time()
# Calculating the foot position given joint angles and a fixed COM at origin.
def forwardKinemLeft(theta_arr):
    # Left Leg
    # j0 to j1
    j0 = theta_arr[0] # Hip rotating in the X direction
    x12 = -0.5
    y12 = 2
    z12 = -2
    # j1 to j2
    j1 = theta_arr[1] # Hip rotating in the Y direction
    x23 = 0
    y23 = 0
    z23 = -2
    # j2 to j3
    j2 = theta_arr[2] # Knee rotating in the Y direction
    x34 = 0
    y34 = 0
    z34 = -4
    # j3 to j4
    j3 = theta_arr[3] # Ankle rotating in the Y direction
    x45 = 1
    y45 = -0.5
    z45 = -2
    # j4 to End Effector (Left Foot)
    j4 = theta_arr[4]# Ankle rotating in the X direction
    x5e = 0
    y5e = 0.5
    z5e = -1

    # 3D Transform Matrices
    R1 = np.matrix(((1, 0, 0, 0),(0, cos(j0), -sin(j0), 0),(0, sin(j0), cos(j0), 0),(0, 0, 0, 1)))
    S1 = np.matrix(((1, 0, 0, x12),(0, 1, 0, y12),(0, 0, 1, z12),(0, 0, 0, 1)))
    A12 = np.matmul(R1,S1)
    #A12 = np.matrix(((1, 0, 0, x12),(0, cos(j0), -sin(j0), y12),(0, sin(j0), cos(j0), z12),(0, 0, 0, 1)))
    R2 = np.matrix(((cos(j1), 0, sin(j1), 0),(0, 1, 0, 0),(-sin(j1), 0, cos(j1), 0),(0, 0, 0, 1)))
    S2 = np.matrix(((1, 0, 0, x23),(0, 1, 0, y23),(0, 0, 1, z23),(0, 0, 0, 1)))
    A23 = np.matmul(R2,S2)
    #A23 = np.matrix(((cos(j1), 0, sin(j1), x23),(0, 1, 0, y23),(-sin(j1), 0, cos(j1), z23),(0, 0, 0, 1)))
    R3 = np.matrix(((cos(j2), 0, sin(j2), 0),(0, 1, 0, 0),(-sin(j2), 0, cos(j2), 0),(0, 0, 0, 1)))
    S3 = np.matrix(((1, 0, 0, x34),(0, 1, 0, y34),(0, 0, 1, z34),(0, 0, 0, 1)))
    A34 = np.matmul(R3,S3)
    #A34 = np.matrix(((cos(j2), 0, sin(j2), x34),(0, 1, 0, y34),(-sin(j2), 0, cos(j2), z34),(0, 0, 0, 1)))
    R4 = np.matrix(((cos(j3), 0, sin(j3), 0),(0, 1, 0, 0),(-sin(j3), 0, cos(j3), 0),(0, 0, 0, 1)))
    S4 = np.matrix(((1, 0, 0, x45),(0, 1, 0, y45),(0, 0, 1, z45),(0, 0, 0, 1)))
    A45 = np.matmul(R4,S4)
    #A45 = np.matrix(((cos(j3), 0, sin(j3), x45),(0, 1, 0, y45),(-sin(j3), 0, cos(j3), z45),(0, 0, 0, 1)))
    R5 = np.matrix(((1, 0, 0, 0),(0, cos(j4), -sin(j4), 0),(0, sin(j4), cos(j4), 0),(0, 0, 0, 1)))
    S5 = np.matrix(((1, 0, 0, x5e),(0, 1, 0, y5e),(0, 0, 1, z5e),(0, 0, 0, 1)))
    A5e = np.matmul(R5,S5)
    #A5e = np.matrix(((1, 0, 0, x5e),(0, cos(j4), -sin(j4), y5e),(0, sin(j4), cos(j4), z5e),(0, 0, 0, 1)))

    A13 = np.matmul(A12, A23)
    A14 = np.matmul(A13, A34)
    A15 = np.matmul(A14, A45)
    A1e = np.matmul(A15, A5e)

    # Outputs for this 10DOF Inverse Kinematics Model are:
    j0_pos_x, j0_pos_y, j0_pos_z = [0, 0, 0]
    j1_pos_x, j1_pos_y, j1_pos_z = [A12[0,3], A12[1,3], A12[2,3]]
    j2_pos_x, j2_pos_y, j2_pos_z = [A13[0,3], A13[1,3], A13[2,3]]
    j3_pos_x, j3_pos_y, j3_pos_z = [A14[0,3], A14[1,3], A14[2,3]]
    j4_pos_x, j4_pos_y, j4_pos_z = [A15[0,3], A15[1,3], A15[2,3]]
    lf_pos_x, lf_pos_y, lf_pos_z = [A1e[0,3], A1e[1,3], A1e[2,3]]

    return j0_pos_x, j0_pos_y, j0_pos_z, j1_pos_x, j1_pos_y, j1_pos_z, j2_pos_x, j2_pos_y, j2_pos_z, j3_pos_x, j3_pos_y, j3_pos_z, j4_pos_x, j4_pos_y, j4_pos_z, lf_pos_x, lf_pos_y, lf_pos_z

def forwardKinemRight(theta_arr):
    # Left Leg
    # j0 to j1
    j0 = theta_arr[0] # Hip rotating in the X direction
    x12 = -0.5
    y12 = -2
    z12 = -2
    # j1 to j2
    j1 = theta_arr[1] # Hip rotating in the Y direction
    x23 = 0
    y23 = 0
    z23 = -2
    # j2 to j3
    j2 = theta_arr[2] # Knee rotating in the Y direction
    x34 = 0
    y34 = 0
    z34 = -4
    # j3 to j4
    j3 = theta_arr[3] # Ankle rotating in the Y direction
    x45 = 1
    y45 = 0.5
    z45 = -2
    # j4 to End Effector (Left Foot)
    j4 = theta_arr[4]# Ankle rotating in the X direction
    x5e = 0
    y5e = -0.5
    z5e = -1

    # 3D Transform Matrices
    R1 = np.matrix(((1, 0, 0, 0),(0, cos(j0), -sin(j0), 0),(0, sin(j0), cos(j0), 0),(0, 0, 0, 1)))
    S1 = np.matrix(((1, 0, 0, x12),(0, 1, 0, y12),(0, 0, 1, z12),(0, 0, 0, 1)))
    A12 = np.matmul(R1,S1)
    #A12 = np.matrix(((1, 0, 0, x12),(0, cos(j0), -sin(j0), y12),(0, sin(j0), cos(j0), z12),(0, 0, 0, 1)))
    R2 = np.matrix(((cos(j1), 0, sin(j1), 0),(0, 1, 0, 0),(-sin(j1), 0, cos(j1), 0),(0, 0, 0, 1)))
    S2 = np.matrix(((1, 0, 0, x23),(0, 1, 0, y23),(0, 0, 1, z23),(0, 0, 0, 1)))
    A23 = np.matmul(R2,S2)
    #A23 = np.matrix(((cos(j1), 0, sin(j1), x23),(0, 1, 0, y23),(-sin(j1), 0, cos(j1), z23),(0, 0, 0, 1)))
    R3 = np.matrix(((cos(j2), 0, sin(j2), 0),(0, 1, 0, 0),(-sin(j2), 0, cos(j2), 0),(0, 0, 0, 1)))
    S3 = np.matrix(((1, 0, 0, x34),(0, 1, 0, y34),(0, 0, 1, z34),(0, 0, 0, 1)))
    A34 = np.matmul(R3,S3)
    #A34 = np.matrix(((cos(j2), 0, sin(j2), x34),(0, 1, 0, y34),(-sin(j2), 0, cos(j2), z34),(0, 0, 0, 1)))
    R4 = np.matrix(((cos(j3), 0, sin(j3), 0),(0, 1, 0, 0),(-sin(j3), 0, cos(j3), 0),(0, 0, 0, 1)))
    S4 = np.matrix(((1, 0, 0, x45),(0, 1, 0, y45),(0, 0, 1, z45),(0, 0, 0, 1)))
    A45 = np.matmul(R4,S4)
    #A45 = np.matrix(((cos(j3), 0, sin(j3), x45),(0, 1, 0, y45),(-sin(j3), 0, cos(j3), z45),(0, 0, 0, 1)))
    R5 = np.matrix(((1, 0, 0, 0),(0, cos(j4), -sin(j4), 0),(0, sin(j4), cos(j4), 0),(0, 0, 0, 1)))
    S5 = np.matrix(((1, 0, 0, x5e),(0, 1, 0, y5e),(0, 0, 1, z5e),(0, 0, 0, 1)))
    A5e = np.matmul(R5,S5)
    #A5e = np.matrix(((1, 0, 0, x5e),(0, cos(j4), -sin(j4), y5e),(0, sin(j4), cos(j4), z5e),(0, 0, 0, 1)))

    A13 = np.matmul(A12, A23)
    A14 = np.matmul(A13, A34)
    A15 = np.matmul(A14, A45)
    A1e = np.matmul(A15, A5e)

    # Outputs for this 10DOF Inverse Kinematics Model are:
    j0_pos_x, j0_pos_y, j0_pos_z = [0, 0, 0]
    j1_pos_x, j1_pos_y, j1_pos_z = [A12[0,3], A12[1,3], A12[2,3]]
    j2_pos_x, j2_pos_y, j2_pos_z = [A13[0,3], A13[1,3], A13[2,3]]
    j3_pos_x, j3_pos_y, j3_pos_z = [A14[0,3], A14[1,3], A14[2,3]]
    j4_pos_x, j4_pos_y, j4_pos_z = [A15[0,3], A15[1,3], A15[2,3]]
    lf_pos_x, lf_pos_y, lf_pos_z = [A1e[0,3], A1e[1,3], A1e[2,3]]

    return j0_pos_x, j0_pos_y, j0_pos_z, j1_pos_x, j1_pos_y, j1_pos_z, j2_pos_x, j2_pos_y, j2_pos_z, j3_pos_x, j3_pos_y, j3_pos_z, j4_pos_x, j4_pos_y, j4_pos_z, lf_pos_x, lf_pos_y, lf_pos_z

""" theta_arr = [pi/2, 0, 0, 0, 0]
j0_pos_x, j0_pos_y, j0_pos_z, j1_pos_x, j1_pos_y, j1_pos_z, j2_pos_x, j2_pos_y, j2_pos_z, j3_pos_x, j3_pos_y, j3_pos_z, j4_pos_x, j4_pos_y, j4_pos_z, lf_pos_x, lf_pos_y, lf_pos_z = forwardKinem(theta_arr)
print(lf_pos_x,lf_pos_y,lf_pos_z) """

""" angles = [0, 45, 90, 135, 180]  # Angles from 0 to 180 degrees
combinations = []
for angle1 in angles:
    for angle2 in angles:
        for angle3 in angles:
            for angle4 in angles:
                for angle5 in angles:
                    for angle6 in angles:
                        for angle7 in angles:
                            for angle8 in angles:
                                for angle9 in angles:
                                    for angle10 in angles:
                                        combinations.append([angle1, angle2, angle3, angle4, angle5, angle6, angle7, angle8, angle9, angle10])

# Now 'combinations' contains all possible combinations of angles for 10 joints
print("Total combinations:", len(combinations))

j0_deg, j1_deg, j2_deg, j3_deg, j4_deg = ([] for _ in range(5))
j5_deg, j6_deg, j7_deg, j8_deg, j9_deg = ([] for _ in range(5))
n = 1000
for combo in combinations[:n]:
    j0_deg.append(combo[0])
    j1_deg.append(combo[1])
    j2_deg.append(combo[2])
    j3_deg.append(combo[3])
    j4_deg.append(combo[4])
    j5_deg.append(combo[5])
    j6_deg.append(combo[6])
    j7_deg.append(combo[7])
    j8_deg.append(combo[8])
    j9_deg.append(combo[9]) """

# Inputs to the forward kinematic animation
j0_deg = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
j1_deg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, -20, -30, -40, -50, -60, -70,-80, -90, -90, -80, -70, -60, -50, -40, -30, -20, -10, 0]
j2_deg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0]
j3_deg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0]
j4_deg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, -20, -30, -40, -50, -60, -70,-80, -90, -90, -80, -70, -60, -50, -40, -30, -20, -10, 0]

j5_deg = [0, -10, -20, -30, -40, -50, -60, -70, -80, -90, -90, -80, -70, -60, -50, -40, -30, -20, -10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
j6_deg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, -20, -30, -40, -50, -60, -70,-80, -90, -90, -80, -70, -60, -50, -40, -30, -20, -10, 0]
j7_deg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0]
j8_deg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0]
j9_deg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0]


def deg2rad_list(j_deg):
    j_rad = []
    for deg in j_deg:
        j_rad.append(radians(deg))
    return j_rad

j0_rad = deg2rad_list(j0_deg)
j1_rad = deg2rad_list(j1_deg)
j2_rad = deg2rad_list(j2_deg)
j3_rad = deg2rad_list(j3_deg)
j4_rad = deg2rad_list(j4_deg)

j5_rad = deg2rad_list(j5_deg)
j6_rad = deg2rad_list(j6_deg)
j7_rad = deg2rad_list(j7_deg)
j8_rad = deg2rad_list(j8_deg)
j9_rad = deg2rad_list(j9_deg)

n = len(j0_rad)
j0_pos_x, j0_pos_y, j0_pos_z = ([None] * n for _ in range(3))
j1_pos_x, j1_pos_y, j1_pos_z = ([None] * n for _ in range(3)) 
j2_pos_x, j2_pos_y, j2_pos_z = ([None] * n for _ in range(3)) 
j3_pos_x, j3_pos_y, j3_pos_z = ([None] * n for _ in range(3)) 
j4_pos_x, j4_pos_y, j4_pos_z = ([None] * n for _ in range(3)) 
lf_pos_x, lf_pos_y, lf_pos_z = ([None] * n for _ in range(3))

j5_pos_x, j5_pos_y, j5_pos_z = ([None] * n for _ in range(3))
j6_pos_x, j6_pos_y, j6_pos_z = ([None] * n for _ in range(3)) 
j7_pos_x, j7_pos_y, j7_pos_z = ([None] * n for _ in range(3)) 
j8_pos_x, j8_pos_y, j8_pos_z = ([None] * n for _ in range(3)) 
j9_pos_x, j9_pos_y, j9_pos_z = ([None] * n for _ in range(3)) 
rf_pos_x, rf_pos_y, rf_pos_z = ([None] * n for _ in range(3))

for i in range(n):
    left_theta_arr = [j0_rad[i], j1_rad[i],j2_rad[i], j3_rad[i], j4_rad[i]]
    j0_pos_x[i], j0_pos_y[i], j0_pos_z[i], j1_pos_x[i], j1_pos_y[i], j1_pos_z[i], j2_pos_x[i], j2_pos_y[i], j2_pos_z[i], j3_pos_x[i], j3_pos_y[i], j3_pos_z[i], j4_pos_x[i], j4_pos_y[i], j4_pos_z[i], lf_pos_x[i], lf_pos_y[i], lf_pos_z[i] = forwardKinemLeft(left_theta_arr)

    right_theta_arr = [j5_rad[i], j6_rad[i], j7_rad[i], j8_rad[i], j9_rad[i]]
    j5_pos_x[i], j5_pos_y[i], j5_pos_z[i], j6_pos_x[i], j6_pos_y[i], j6_pos_z[i], j7_pos_x[i], j7_pos_y[i], j7_pos_z[i], j8_pos_x[i], j8_pos_y[i], j8_pos_z[i], j9_pos_x[i], j9_pos_y[i], j9_pos_z[i], rf_pos_x[i], rf_pos_y[i], rf_pos_z[i] = forwardKinemRight(right_theta_arr)

# Animation of the Mathematics
# Create figure
# Plot the first joint + link in red
fig = go.Figure(go.Scatter3d(x=[], y=[], z=[],
                            mode="lines+markers",
                            marker=dict(color="red", size=5),
                            line = dict(color="red", width=3)
                            )
                )
# Plot the second joint + link in blue
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="blue", size=2),
                            line = dict(color="blue", width=3)
                            
                            )
                )
# Plot the third joint + link in green
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="green", size=2),
                            line = dict(color="green", width=3)
                            
                            )
                )
# Plot the fourth joint + link in yellow
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="yellow", size=2),
                            line = dict(color="yellow", width=3)
                            )
                )
# Plot the fifth joint + link to Left Foot in orange
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="orange", size=2),
                            line = dict(color="orange", width=3)
                            )
                )
# Plot the sixth joint + link in red
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode="lines+markers",
                            marker=dict(color="red", size=5),
                            line = dict(color="red", width=3)
                            )
                )
# Plot the seventh joint + link in blue
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="blue", size=2),
                            line = dict(color="blue", width=3)
                            
                            )
                )
# Plot the eight joint + link in green
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="green", size=2),
                            line = dict(color="green", width=3)
                            
                            )
                )
# Plot the ninth joint + link in yellow
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="yellow", size=2),
                            line = dict(color="yellow", width=3)
                            )
                )
# Plot the tenth joint + link to Right Foot in orange
fig.add_trace(go.Scatter3d(x=[], y=[], z=[],
                            mode='lines+markers',
                            marker=dict(color="orange", size=2),
                            line = dict(color="orange", width=3)
                            )
                )

# Frames                # Joint 1 to Joint 2
frames = [go.Frame(data=[go.Scatter3d(x=[j0_pos_x[k], j1_pos_x[k]],
                                    y=[j0_pos_y[k], j1_pos_y[k]],
                                    z=[j0_pos_z[k], j1_pos_z[k]]
                                    ),
                        # Joint 2 to Joint 3
                        go.Scatter3d(x=[j1_pos_x[k], j2_pos_x[k]],
                                    y=[j1_pos_y[k], j2_pos_y[k]],
                                    z=[j1_pos_z[k], j2_pos_z[k]]
                                        ),
                        # Joint 3 to Joint 4
                        go.Scatter3d(x=[j2_pos_x[k], j3_pos_x[k]],
                                    y=[j2_pos_y[k], j3_pos_y[k]],
                                    z=[j2_pos_z[k], j3_pos_z[k]]
                                        ),
                        # Joint 4 to Joint 5
                        go.Scatter3d(x=[j3_pos_x[k], j4_pos_x[k]],
                                    y=[j3_pos_y[k], j4_pos_y[k]],
                                    z=[j3_pos_z[k], j4_pos_z[k]]
                                        ),
                        # Joint 5 to Left Foot
                        go.Scatter3d(x=[j4_pos_x[k], lf_pos_x[k]],
                                    y=[j4_pos_y[k], lf_pos_y[k]],
                                    z=[j4_pos_z[k], lf_pos_z[k]]
                                        ),
                        # Joint 6 to Joint 7
                        go.Scatter3d(x=[j5_pos_x[k], j6_pos_x[k]],
                                    y=[j5_pos_y[k], j6_pos_y[k]],
                                    z=[j5_pos_z[k], j6_pos_z[k]]
                                        ),
                        # Joint 7 to Joint 8
                        go.Scatter3d(x=[j6_pos_x[k], j7_pos_x[k]],
                                    y=[j6_pos_y[k], j7_pos_y[k]],
                                    z=[j6_pos_z[k], j7_pos_z[k]]
                                        ),
                        # Joint 8 to Joint 9
                        go.Scatter3d(x=[j7_pos_x[k], j8_pos_x[k]],
                                    y=[j7_pos_y[k], j8_pos_y[k]],
                                    z=[j7_pos_z[k], j8_pos_z[k]]
                                        ),
                        # Joint 9 to Joint 10
                        go.Scatter3d(x=[j8_pos_x[k], j9_pos_x[k]],
                                    y=[j8_pos_y[k], j9_pos_y[k]],
                                    z=[j8_pos_z[k], j9_pos_z[k]]
                                        ),
                        # Joint 10 to Right Foot
                        go.Scatter3d(x=[j9_pos_x[k], rf_pos_x[k]],
                                    y=[j9_pos_y[k], rf_pos_y[k]],
                                    z=[j9_pos_z[k], rf_pos_z[k]]
                                        ),
                        ],
                traces= [0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
                name=f'frame{k}'
                )for k  in  range(len(lf_pos_x))
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
                        "args": [None, frame_args(n)],
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

fig.update_layout(scene = dict(xaxis=dict(range=[-15, 15], autorange=False),
                            yaxis=dict(range=[-15, 15], autorange=False),
                            zaxis=dict(range=[-15, 5], autorange=False)
                            )
                )

fig.update_layout(sliders=sliders)
fig.layout.scene.aspectratio = {'x':1, 'y':1, 'z':1}
fig.show()

end = time.time()
print('Time Elapsed:' + str(end-start) + ' seconds')
