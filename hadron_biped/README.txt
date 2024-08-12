HADRON Biped README

\hardware - contains BoM, CAD files, & printing instructions
\arduino - contains low-level actuation & sensor code
\images - contains pictures & videos of hardware prints, software plots, math references, and live control

\python

// Robot Geometry

hadron_geom.py - A calibration dataset that initializes the robot geometry (given the same link/joint setup).

forward_kinem.py - Forward kinematics mathematics for a 10DoF biped. Inputs are each joint angle, outputs are the position & direction vectors for each link end.

inverse_kinem.py - Inverse kinematics computation (analytical, want to include optimization later) that solves for joint angles given a position/direction vector for a link end. Can be called for a single position setpoint or an array of streamed setpoints

// Models

lipm2d.py - Simulates a 2D Linear Inverted Pendulum (variable length) ODE, and plots a 2D trajectory animation and 2D position & velocity.
Inputs: f, tau, m, l, ts, t_span, state_i = [theta, r, theta_dot, r_dot]
Outputs: state = [theta, r, theta_dot, r_dot] -> [x, z, x_dot, z_dot]

3dpend.py - Simulates a 3D pendulum (spherical derived from Langrange) ODE and plots a 3D trajectory, 3D position & velocity, and an animation.
Inputs: L, l, m, ts, t_span, state_i = [theta, phi, theta_dot, phi_dot]
Outputs: state = [theta, phi, theta_dot, phi_dot] -> [x, y, z, x_dot, y_dot, z_dot]
Improvements: 
- ODE is incorrect (unstable), needs to be rederived

stepper2d.py - Simulates 2D stepper model using the 3D LIPM trajectory capture point model
Inputs: x0, v0, z0, delta_t, target_orbital_energy, data_len, switch_index
Outputs: xt, vt, left_foot_x, left_foot_z, right_foot_x, right_foot_z, orbital_energy
Improvements:
- Change animation from matplotlib to plotly.go

stepper3d.py - Simulates a 3D stepper model using a 3D LIPM trajectory capture point model
Inputs: COM_pos_0, COM_v0, zc, left_foot_pos, right_foot_pos, delta_t, s_x, s_y, a, b, theta, T_sup, total_time
Outputs: COM_pos, left_foot_pos, right_foot_pos (x, y, z)
Improvements:
- Step positions need to be plotted in animation
- Pendulum arm & origin point need to be plotted


\arduino:

motor_test.ino - test script to sweep a servo motor
motor_state_test.ino - test script to read potentiometer outputs and calculate velocity & acceleration from position feedback
imu_test.ino - test script to read IMU sensor outputs
step_test.ino - test script running a step trajectory
walk.ino - test script runnning an open loop dual step trajectory






