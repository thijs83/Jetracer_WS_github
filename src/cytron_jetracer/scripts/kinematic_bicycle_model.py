#!/usr/bin/env python
import numpy as np


def evaluate_Fx_2(vx, th):
    #define parameters
    v_friction = 1.3852415
    v_friction_static = 0.15652551
    v_friction_static_tanh_mult = 44.221073
    v_friction_quad = 0.093703546

    tau_offset = 0.19454427
    tau_offset_reverse = 0.26989207
    tau_steepness = 15.530939
    tau_steepness_reverse = 10.14599
    tau_sat_high = 2.3438272
    tau_sat_high_reverse = 1.4451292



    #friction model
    static_friction = np.tanh(v_friction_static_tanh_mult  * vx) * v_friction_static
    v_contribution = - static_friction - vx * v_friction - np.sign(vx) * vx ** 2 * v_friction_quad 

    #for positive throttle
    th_activation1 = (np.tanh((th - tau_offset) * tau_steepness) + 1) * tau_sat_high
    #for negative throttle
    th_activation2 = (np.tanh((th + tau_offset_reverse) * tau_steepness_reverse)-1) * tau_sat_high_reverse

    throttle_contribution = (th_activation1 + th_activation2) 

    # --------
    
    Fx = throttle_contribution + v_contribution
    Fx_r = Fx * 0.5
    Fx_f = Fx * 0.5
    return Fx_r, Fx_f


def evaluate_steer_angle(steer_command):
    a = -1.2053807
    b = 0.38302866
    c = 0.08739186
    steer_angle = b * np.tanh(a * (steer_command-c))
    return steer_angle

def kinematic_bicycle_2(z):
    # NOTE: this model must match what was used to fit the data in Nominal_model_fittingVxVyW
    # This is the model obtained with improved throttle curve and better data set. August 2023.
    # Also has improved steering curve

    #z = throttle delta x y theta vx vy w
    u = z[0:2]
    x = z[2:]
    th = u[0]
    #steer_angle = - u[1] * (17 / 180.0 * np.pi)  # needs to be converted to radians positive in anti-clockwise sense
    vx = x[3]
    vy = x[4]
    w = x[5]
    # #use this to generate data
    # #this is a simple kinematic bycicle, plus steering reduction at high speed

    # #hard coded parameters
    L = 0.175
    m = 1.6759806


    #evaluate steering angle 
    steer_angle = evaluate_steer_angle(u[1])


    #evaluate forward force
    Fx_r, Fx_f = evaluate_Fx_2(vx, th)

    
    # longitudinal dynamics
    forces_longit_sum = Fx_r + Fx_f * np.cos(steer_angle)
    acc_x =  forces_longit_sum / m # acceleration in the longitudinal direction



    #Delta_vx = forces_longit_sum * dt /m

    #simple bycicle nominal model
    xdot1 = vx * np.cos(x[2])  # x_dot = v * cos(eta)
    xdot2 = vx * np.sin(x[2])  # y_dot = v * sin(eta)
    xdot3 = vx * np.tan(steer_angle) / L  # v * tan(delta) / L
    xdot4 =  acc_x     
    xdot5 = 0  # for now no side slipping
    xdot6 = 0  # actually this will be taken as w value, not multipied by dt

    # assemble derivatives [x y theta vx vy omega]
    xdot = np.array([xdot1, xdot2, xdot3, xdot4, xdot5, xdot6])

    return xdot





