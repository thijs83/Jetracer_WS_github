import numpy as np
import sys
sys.path.insert(0, '/home/litosh/forces_pro')
import math
from scipy.interpolate import CubicSpline
import warnings 
# turn off low rank waring from chebfit
warnings.simplefilter('ignore', np.RankWarning)


# Realize the track structure for each available choice
def produce_track(choice,n_checkpoints):
    # initialize the gate as empty
    gates = []

    if choice == 'savoiardo':

        R = 0.8  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.82
        theta_init2 = np.pi * -0.5
        theta_end2 = np.pi * 0.5
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkpoints)
        theta_init4 = np.pi * 0.5
        theta_end4 = np.pi * 1.5
        theta_vec4 = np.linspace(theta_init4, theta_end4, n_checkpoints)

        Checkpoints_x1 = np.linspace(- 1.5 * R, 1.5 * R, n_checkpoints)
        Checkpoints_y1 = np.zeros(n_checkpoints) - R
        Checkpoints_x2 = 1.5 * R + R * np.cos(theta_vec2)
        Checkpoints_y2 = R * np.sin(theta_vec2)
        Checkpoints_x3 = np.linspace(1.5 * R, -1.5*R, n_checkpoints)
        Checkpoints_y3 = R * np.ones(n_checkpoints)
        Checkpoints_x4 = -1.5* R + R * np.cos(theta_vec4)
        Checkpoints_y4 = R * np.sin(theta_vec4)

        xy_lenght =  (6 * R) + (2 * np.pi * R)      # lenght of the xy path
        #amplitude = [1.5 * R]                       # amplitude of the sin wave, i.e. the max height of the path
        amplitude = [1.5 * R , 0.5 * R ]            # amplitude of the sin wave, i.e. the max height of the path
        n_reps = 3                                  # how many complete cycles do we want in a single lap
        n_parts = 4                                 # how many parts do we want the path to be divided into
        offset = 2                                  # offset (z-wise) of the path from the origin

        Checkpoints_x = [*Checkpoints_x2[0:n_checkpoints - 1],
                         *Checkpoints_x3[0:n_checkpoints - 1], *Checkpoints_x4[0:n_checkpoints - 1], *Checkpoints_x1[0:n_checkpoints]]
        Checkpoints_y = [*Checkpoints_y2[0:n_checkpoints - 1],
                         *Checkpoints_y3[0:n_checkpoints - 1], *Checkpoints_y4[0:n_checkpoints -1], *Checkpoints_y1[0:n_checkpoints]]
        
        Checkpoints_z = produce_sin_z(n_checkpoints, n_parts, amplitude, n_reps, xy_lenght, offset)


    elif choice == 'double_donut':

        R = 1                           # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        theta_init1 = np.pi * -0.5
        theta_end1 = np.pi * 0.0
        theta_vec1 = np.linspace(theta_init1, theta_end1, n_checkpoints)
        theta_init2 = np.pi * 1
        theta_end2 = np.pi * -1
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkpoints)
        theta_init3 = np.pi * 0
        theta_end3 = np.pi * 1.5
        theta_vec3 = np.linspace(theta_init3, theta_end3, n_checkpoints)

        Checkpoints_x1 = - R + R * np.cos(theta_vec1)
        Checkpoints_y1 =  + R * np.sin(theta_vec1)
        Checkpoints_x2 = + R + R * np.cos(theta_vec2)
        Checkpoints_y2 =  + R * np.sin(theta_vec2)
        Checkpoints_x3 = - R + R * np.cos(theta_vec3)
        Checkpoints_y3 = + R * np.sin(theta_vec3)

        xy_lenght =  (4 * np.pi * R)                # lenght of the xy path
        amplitude = [1 * R]                         # amplitude of the sin wave, i.e. the max height of the path
        n_reps = 1                                  # how many complete cycles do we want in a single lap
        n_parts = 3                                 # how many parts do we want the path to be divided into
        offset = 2                                  # offset (z-wise) of the path from the origin

        Checkpoints_x = [*Checkpoints_x1[0:n_checkpoints - 1], *Checkpoints_x2[0:n_checkpoints - 1],
                         *Checkpoints_x3[0:n_checkpoints]]
        Checkpoints_y = [*Checkpoints_y1[0:n_checkpoints - 1], *Checkpoints_y2[0:n_checkpoints - 1],
                         *Checkpoints_y3[0:n_checkpoints]]
        
        Checkpoints_z = produce_sin_z(n_checkpoints, n_parts, amplitude, n_reps, xy_lenght, offset)


    elif choice == 'savoiardo_saturate_steering':
        R = 0.3  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        theta_init2 = np.pi * -0.5
        theta_end2 = np.pi * 0.5
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkpoints)
        theta_init4 = np.pi * 0.5
        theta_end4 = np.pi * 1.5
        theta_vec4 = np.linspace(theta_init4, theta_end4, n_checkpoints)

        Checkpoints_x1 = np.linspace(- 4.5 * R, 4.5 * R, n_checkpoints)
        Checkpoints_y1 = np.zeros(n_checkpoints) - R
        Checkpoints_x2 = 4.5 * R + R * np.cos(theta_vec2)
        Checkpoints_y2 = R * np.sin(theta_vec2)
        Checkpoints_x3 = np.linspace(4.5 * R, -4.5*R, n_checkpoints)
        Checkpoints_y3 = R * np.ones(n_checkpoints)
        Checkpoints_x4 = -4.5* R + R * np.cos(theta_vec4)
        Checkpoints_y4 = R * np.sin(theta_vec4)

        xy_lenght = (12 * np.pi * R)                # lenght of the xy path
        amplitude = [ R ]                           # amplitude of the sin wave, i.e. the max height of the path
        n_reps = 1                                  # how many complete cycles do we want in a single lap
        n_parts = 4                                 # how many parts do we want the path to be divided into
        offset = 2                                  # offset (z-wise) of the path from the origin

        Checkpoints_x = [*Checkpoints_x2[0:n_checkpoints - 1],
                         *Checkpoints_x3[0:n_checkpoints - 1], *Checkpoints_x4[0:n_checkpoints - 1], *Checkpoints_x1[0:n_checkpoints ]]
        Checkpoints_y = [*Checkpoints_y2[0:n_checkpoints - 1],
                         *Checkpoints_y3[0:n_checkpoints - 1], *Checkpoints_y4[0:n_checkpoints], *Checkpoints_y1[0:n_checkpoints - 1]]
        
        Checkpoints_z = produce_sin_z(n_checkpoints, n_parts, amplitude, n_reps, xy_lenght, offset)    


    elif choice == 'circle':
        n_checkpoints = 4 * n_checkpoints
        R = 1  
        theta_init = np.pi * -0.5
        theta_end = np.pi * 1.5
        theta_vec = np.linspace(theta_init, theta_end, n_checkpoints)

        xy_lenght = (2 * np.pi * R)                 # lenght of the xy path
        amplitude = [ R ]                           # amplitude of the sin wave, i.e. the max height of the path
        n_reps = 1                                  # how many complete cycles do we want in a single lap
        n_parts = 1                                 # how many parts do we want the path to be divided into
        offset = 1                                  # offset (z-wise) of the path from the origin

        Checkpoints_x = R * np.cos(theta_vec)
        Checkpoints_y = R * np.sin(theta_vec)
        Checkpoints_z = produce_sin_z(n_checkpoints, n_parts, amplitude, n_reps, xy_lenght, offset) 


    elif choice == 'racetrack_saturate_steering':

        R = 0.8  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        theta_init2 = np.pi * -0.5
        theta_end2 = np.pi * 0.5
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkpoints)

        theta_init3 = np.pi * 1.5
        theta_end3 = np.pi * 0.5
        theta_vec3 = np.linspace(theta_init3, theta_end3, n_checkpoints)

        theta_init6 = np.pi * 0.5
        theta_end6 = np.pi * 1.0
        theta_vec6 = np.linspace(theta_init6, theta_end6, n_checkpoints)

        theta_init8 = np.pi * -1.0
        theta_end8 = np.pi * 0.0
        theta_vec8 = np.linspace(theta_init8, theta_end8, n_checkpoints)

        theta_init10 = np.pi * 1.0
        theta_end10 = np.pi * 0.0
        theta_vec10 = np.linspace(theta_init10, theta_end10, n_checkpoints)

        theta_init12 = np.pi * -1.0
        theta_end12 = np.pi * -0.5
        theta_vec12 = np.linspace(theta_init12, theta_end12, n_checkpoints)        

        # DEFINED STARTING FROM START POINT AND THEN SHIFT IT LATER IF NEEDED
        Checkpoints_x1 = np.linspace(0, 3*R, n_checkpoints)
        Checkpoints_y1 = np.zeros(n_checkpoints) - R

        Checkpoints_x2 = + 3*R + R * np.cos(theta_vec2)
        Checkpoints_y2 =  R * np.sin(theta_vec2)

        Checkpoints_x3 = 3*R + R * np.cos(theta_vec3)
        Checkpoints_y3 = 2*R +R * np.sin(theta_vec3)

        Checkpoints_x4 = + 3*R + R * np.cos(theta_vec2)
        Checkpoints_y4 = + 4*R + R * np.sin(theta_vec2)

        Checkpoints_x5 = np.linspace(3*R, -3*R, n_checkpoints)
        Checkpoints_y5 = np.zeros(n_checkpoints) + 5*R

        Checkpoints_x6 = - 3*R + 2*R * np.cos(theta_vec6)
        Checkpoints_y6 = + 3*R + 2*R * np.sin(theta_vec6)

        Checkpoints_x7 = np.zeros(n_checkpoints) - 5*R
        Checkpoints_y7 = np.linspace(3 * R, 0, n_checkpoints)

        Checkpoints_x8 = - 4.5 * R + 0.5 * R * np.cos(theta_vec8)
        Checkpoints_y8 = + 0.5 * R * np.sin(theta_vec8)

        Checkpoints_x9 = np.zeros(n_checkpoints) - 4*R
        Checkpoints_y9 = np.linspace(0, 2*R, n_checkpoints)

        Checkpoints_x10 = - 3.5 * R + 0.5 * R * np.cos(theta_vec10)
        Checkpoints_y10 = + 2 * R + 0.5 * R * np.sin(theta_vec10)

        Checkpoints_x11 = np.zeros(n_checkpoints) - 3*R
        Checkpoints_y11 = np.linspace(2 * R, 0, n_checkpoints)

        Checkpoints_x12 = - 2.5 * R + 0.5 * R * np.cos(theta_vec8)
        Checkpoints_y12 = + 0.5 * R * np.sin(theta_vec8)

        Checkpoints_x13 = np.zeros(n_checkpoints) - 2*R
        Checkpoints_y13 = np.linspace(0, 2*R, n_checkpoints)

        Checkpoints_x14 = - 1.5 * R + 0.5 * R * np.cos(theta_vec10)
        Checkpoints_y14 = + 2 * R + 0.5 * R * np.sin(theta_vec10)

        Checkpoints_x15 = np.zeros(n_checkpoints) - 1 * R
        Checkpoints_y15 = np.linspace(2 * R, 0, n_checkpoints)

        Checkpoints_x16 = + R * np.cos(theta_vec12)
        Checkpoints_y16 = + R * np.sin(theta_vec12)

        xy_lenght = (10*np.pi*R) + 24*R             # lenght of the xy path
        amplitude = [ 1*R , 2*R ]                   # amplitude of the sin wave, i.e. the max height of the path
        n_reps = 8                                  # how many complete cycles do we want in a single lap
        n_parts = 16                                # how many parts do we want the path to be divided into
        offset = 3                                  # offset (z-wise) of the path from the origin

        Checkpoints_x = [*Checkpoints_x1[0:n_checkpoints - 1],
                         *Checkpoints_x2[0:n_checkpoints - 1],
                         *Checkpoints_x3[0:n_checkpoints - 1],
                         *Checkpoints_x4[0:n_checkpoints - 1],
                         *Checkpoints_x5[0:n_checkpoints - 1],
                         *Checkpoints_x6[0:n_checkpoints - 1],
                         *Checkpoints_x7[0:n_checkpoints - 1],
                         *Checkpoints_x8[0:n_checkpoints - 1],
                         *Checkpoints_x9[0:n_checkpoints - 1],
                         *Checkpoints_x10[0:n_checkpoints - 1],
                         *Checkpoints_x11[0:n_checkpoints - 1],
                         *Checkpoints_x12[0:n_checkpoints - 1],
                         *Checkpoints_x13[0:n_checkpoints - 1],
                         *Checkpoints_x14[0:n_checkpoints - 1],
                         *Checkpoints_x15[0:n_checkpoints - 1],
                         *Checkpoints_x16[0:n_checkpoints]]

        Checkpoints_y = [*Checkpoints_y1[0:n_checkpoints - 1],
                         *Checkpoints_y2[0:n_checkpoints - 1],
                         *Checkpoints_y3[0:n_checkpoints - 1],
                         *Checkpoints_y4[0:n_checkpoints - 1],
                         *Checkpoints_y5[0:n_checkpoints - 1],
                         *Checkpoints_y6[0:n_checkpoints - 1],
                         *Checkpoints_y7[0:n_checkpoints - 1],
                         *Checkpoints_y8[0:n_checkpoints - 1],
                         *Checkpoints_y9[0:n_checkpoints - 1],
                         *Checkpoints_y10[0:n_checkpoints - 1],
                         *Checkpoints_y11[0:n_checkpoints - 1],
                         *Checkpoints_y12[0:n_checkpoints - 1],
                         *Checkpoints_y13[0:n_checkpoints - 1],
                         *Checkpoints_y14[0:n_checkpoints - 1],
                         *Checkpoints_y15[0:n_checkpoints - 1],
                         *Checkpoints_y16[0:n_checkpoints]]
        
        Checkpoints_z = produce_sin_z(n_checkpoints, n_parts, amplitude, n_reps, xy_lenght, offset) 


    elif choice == 'gain_sweep_track':
        R = 0.4  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        straight_bit_half_length = 2.3

        theta_init2 = np.pi * -0.5
        theta_end2 = np.pi * 0.5
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkpoints)
        theta_init4 = np.pi * 0.5
        theta_end4 = np.pi * 1.5
        theta_vec4 = np.linspace(theta_init4, theta_end4, n_checkpoints)

        Checkpoints_x1 = np.linspace(- straight_bit_half_length, straight_bit_half_length, n_checkpoints)
        Checkpoints_y1 = np.zeros(n_checkpoints) - R
        Checkpoints_x2 = straight_bit_half_length + R * np.cos(theta_vec2)
        Checkpoints_y2 = R * np.sin(theta_vec2)
        Checkpoints_x3 = np.linspace(straight_bit_half_length, -straight_bit_half_length, n_checkpoints)
        Checkpoints_y3 = R * np.ones(n_checkpoints)
        Checkpoints_x4 = -straight_bit_half_length + R * np.cos(theta_vec4)
        Checkpoints_y4 = R * np.sin(theta_vec4)

        xy_lenght = (3*np.pi*R) + 4*straight_bit_half_length      # lenght of the xy path
        amplitude = [ 2 * R ]                                     # amplitude of the sin wave, i.e. the max height of the path
        n_reps = 3                                                # how many complete cycles do we want in a single lap
        n_parts = 4                                               # how many parts do we want the path to be divided into
        offset = 1                                                # offset (z-wise) of the path from the origin

        Checkpoints_x = [*Checkpoints_x1[0:n_checkpoints - 1], *Checkpoints_x2[0:n_checkpoints - 1],
                         *Checkpoints_x3[0:n_checkpoints - 1], *Checkpoints_x4[0:n_checkpoints]]
        Checkpoints_y = [*Checkpoints_y1[0:n_checkpoints - 1], *Checkpoints_y2[0:n_checkpoints - 1],
                         *Checkpoints_y3[0:n_checkpoints - 1], *Checkpoints_y4[0:n_checkpoints]]
        
        Checkpoints_z = produce_sin_z(n_checkpoints, n_parts, amplitude, n_reps, xy_lenght, offset)


    elif choice == 'racetrack_Lab':
        R = 0.5  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        length = 0
        # DEFINED STARTING FROM  START POINT AND THEN SHIFT IT LATER IF NEEDED

        Checkpoints_x1, Checkpoints_y1, length = straight([-1+3*R, 1], [0, 0], n_checkpoints, length)

        Checkpoints_x2, Checkpoints_y2, length = curve([1, 2*R], 2 * R, [-0.5, 0.5], n_checkpoints, length)

        Checkpoints_x3, Checkpoints_y3, length = straight([1, -1], [4*R, 4*R], n_checkpoints, length)

        Checkpoints_x4, Checkpoints_y4, length = curve([-1, 2.5*R], 1.5 * R, [0.5, 1.5], n_checkpoints, length)

        Checkpoints_x5, Checkpoints_y5, length = straight([-1, 1], [R, R], n_checkpoints, length)

        Checkpoints_x6, Checkpoints_y6, length = curve([1, 2 * R], 1 * R, [-0.5, 0.5], n_checkpoints, length)

        Checkpoints_x7, Checkpoints_y7, length = straight([1, -1], [3 * R, 2 * R], n_checkpoints, length)

        Checkpoints_x8, Checkpoints_y8, length = curve([-1, 3 * R], 1 * R, [1.5, 0], n_checkpoints, length)

        Checkpoints_x9, Checkpoints_y9, length = straight([-1 + R, -1 + R], [3 * R, 2 * R], n_checkpoints, length)

        Checkpoints_x10, Checkpoints_y10, length = curve([-1 + 3*R, 2 * R], 2 * R, [-1.0, -0.5], n_checkpoints, length)

        xy_lenght = length                                        # lenght of the xy path
        amplitude = [ 3 * R ]                                     # amplitude of the sin wave, i.e. the max height of the path
        n_reps = 3                                                # how many complete cycles do we want in a single lap
        n_parts = 10                                              # how many parts do we want the path to be divided into
        offset = 1                                                # offset (z-wise) of the path from the origin

        Checkpoints_x = [*Checkpoints_x1[0:n_checkpoints - 1],
                         *Checkpoints_x2[0:n_checkpoints - 1],
                         *Checkpoints_x3[0:n_checkpoints - 1],
                         *Checkpoints_x4[0:n_checkpoints - 1],
                         *Checkpoints_x5[0:n_checkpoints - 1],
                         *Checkpoints_x6[0:n_checkpoints - 1],
                         *Checkpoints_x7[0:n_checkpoints - 1],
                         *Checkpoints_x8[0:n_checkpoints - 1],
                         *Checkpoints_x9[0:n_checkpoints - 1],
                         *Checkpoints_x10[0:n_checkpoints]]
        y_shift = 2*R  #towards the bottom

        Checkpoints_y = [*Checkpoints_y1[0:n_checkpoints - 1] - y_shift,
                         *Checkpoints_y2[0:n_checkpoints - 1] - y_shift,
                         *Checkpoints_y3[0:n_checkpoints - 1] - y_shift,
                         *Checkpoints_y4[0:n_checkpoints - 1] - y_shift,
                         *Checkpoints_y5[0:n_checkpoints - 1] - y_shift,
                         *Checkpoints_y6[0:n_checkpoints - 1] - y_shift,
                         *Checkpoints_y7[0:n_checkpoints - 1] - y_shift,
                         *Checkpoints_y8[0:n_checkpoints - 1] - y_shift,
                         *Checkpoints_y9[0:n_checkpoints - 1] - y_shift,
                         *Checkpoints_y10[0:n_checkpoints] - y_shift]
        
        Checkpoints_z = produce_sin_z(n_checkpoints, n_parts, amplitude, n_reps, xy_lenght, offset) 

    ## SPLINE TRACKS ##
    # tridimentional spline tracks, each tangent vector is defined at the extremity of the spline and represent the gate through which the drone must pass
    elif choice == 'spline_circle':
        R = 3                                     # radius of the circle
        tg_val = 5                                # module of the tangent vector at the extremity of the spline

        ## first spline ##
        # define the points needed for the interpolation
        t1 = [0, 1]                               # t parameter of the spline
        xp1 = 0                                   # 1D data for X dimension
        yp1 = -R                                  # 1D data for Y dimension
        zp1 = 2                                   # 1D data for Z dimension
        # define the extremity values (start and stop of the spline) of the tangent values for x,y,z (for the end we're gonna use the start of the following spline)
        tg1_init = [tg_val,0,0]
        # assemble the spline information
        spline1 = [t1, xp1, yp1, zp1, tg1_init]

        ## second spline ##
        t2 = [0, 1]                 
        xp2 = R
        yp2 = 0
        zp2 = 2
        tg2_init = [0, tg_val, 0]
        spline2 = [t2, xp2, yp2, zp2, tg2_init]

        ## third spline ##
        t3 = [0, 1]                    
        xp3 = 0
        yp3 = R
        zp3 = 2
        tg3_init = [-tg_val, 0, 0]
        spline3 = [t3, xp3, yp3, zp3, tg3_init]

        ## fourth spline ##
        t4 = [0, 1]
        xp4 = -R
        yp4 = 0
        zp4 = 2
        tg4_init = [0, -tg_val, 0]
        spline4 = [t4, xp4, yp4, zp4, tg4_init]

        # generate the spline vector
        splines = [spline1, spline2, spline3, spline4]

        # generate the checkpoints vector
        Checkpoints_x, Checkpoints_y, Checkpoints_z = spline_assembler(n_checkpoints, splines)

        ## produce the points to plot the gates ##
        # initialize the gate vector
        gates = np.zeros((len(splines), 3))

        # generate the gates coordinates vector
        for i in range(0, len(splines)):
            gates[i,:] = splines[i][1:4]
        

    elif choice == 'spline_race_track':
        tg_val = 5                                # module of the tangent vector at the extremity of the spline

        ## first spline ##
        # define the points needed for the interpolation
        t1 = [0, 1]                               # t parameter of the spline
        xp1 = -4                                  # 1D data for X dimension
        yp1 = -3                                  # 1D data for Y dimension
        zp1 = 2                                   # 1D data for Z dimension

        # define the extremity values (start and stop of the spline) of the tangent values for x,y,z (for the end we're gonna use the start of the following spline)
        tg1_init = [tg_val,0,0]
            
        spline1 = [t1, xp1, yp1, zp1, tg1_init]   # assembled points

        ## second spline ##
        t2 = [0, 1]                    
        xp2 = -1     
        yp2 = -1
        zp2 = 2
        tg2_init = [0, tg_val, 0]
        spline2 = [t2, xp2, yp2, zp2, tg2_init]

        ## third spline ##
        t3 = [0, 1]                       
        xp3 = 2
        yp3 = 1
        zp3 = 2 
        tg3_init = [tg_val, 0, 0]
        spline3 = [t3, xp3, yp3, zp3, tg3_init]

        ## fourth spline ##
        t4 = [0, 1]
        xp4 = 2
        yp4 = 4
        zp4 = 2
        tg4_init = [-tg_val, 0, 0]
        spline4 = [t4, xp4, yp4, zp4, tg4_init]

        ## fifth spline ##
        t5 = [0, 1]
        xp5 = -1
        yp5 = 2.5
        zp5 = 2
        tg5_init = [0, -tg_val, tg_val]
        spline5 = [t5, xp5, yp5, zp5, tg5_init]

        ## sixth spline ##
        t6 = [0, 1]
        xp6 = 2
        yp6 = 1
        zp6 = 4
        tg6_init = [tg_val, 0, 0]
        spline6 = [t6, xp6, yp6, zp6, tg6_init]

        ## seventh spline ##
        t7 = [0, 1]
        xp7 = 5
        yp7 = -2
        zp7 = 4
        tg7_init = [0, -tg_val, 0]
        spline7 = [t7, xp7, yp7, zp7, tg7_init]

        ## eight spline ##
        t8 = [0, 1]
        xp8 = 0
        yp8 = -5
        zp8 = 5
        tg8_init = [-tg_val, 0, -tg_val]
        spline8 = [t8, xp8, yp8, zp8, tg8_init]

        # generate the spline vector
        splines = [spline1, spline2, spline3, spline4, spline5, spline6, spline7, spline8]

        # generate the checkpoints vector
        Checkpoints_x, Checkpoints_y, Checkpoints_z = spline_assembler(n_checkpoints, splines)   

        ## produce the points to plot the gates ##
        # initialize the gate vector
        gates = np.zeros((len(splines), 3))

        # generate the gates vector
        for i in range(0, len(splines)):
            gates[i,:] = splines[i][1:4]


    else:
        print('Invalid choice of track:')
        print('You selected: ', choice)


    return Checkpoints_x, Checkpoints_y, Checkpoints_z, gates


# straight path
def straight(xlims, ylims, n_checkpoints, length):
    Checkpoints_x = np.linspace(xlims[0], xlims[1], n_checkpoints)
    Checkpoints_y = np.linspace(ylims[0], ylims[1], n_checkpoints)
    length = length + math.dist([xlims[0], ylims[0]], [xlims[1], ylims[1]])
    return Checkpoints_x, Checkpoints_y, length


# definition of a curve for a path
def curve(centre, R,theta_extremes, n_checkpoints, length):
    theta_init = np.pi * theta_extremes[0]
    theta_end = np.pi * theta_extremes[1]
    theta_vec = np.linspace(theta_init, theta_end, n_checkpoints)
    Checkpoints_x = centre[0] + R * np.cos(theta_vec)
    Checkpoints_y = centre[1] + R * np.sin(theta_vec)
    length = length + abs(theta_end - theta_init) * R

    return Checkpoints_x, Checkpoints_y, length


# produces the coefficients for a sinusoidal behaviour for the z axis of the path (useful to tridimentionalize the former 2D path)
def produce_sin_z(n_checkpoints,n_parts, amplitude, n_reps, xy_track_lenght,offset):
    # tot_checkpoint = tot number of checkpoints needed by the path
    # amplitude = (single or vector) amplitude of the sin wave, i.e. the max height of the path
    # n_reps = how many complete sin cycles we want in a single lap
    # xy_track_lenght = lenght of the xy path

    # Tot num of checkpoints needed by the path. Since each part of the track except the last one has (n_checkpoints-1) points we have to remove the extra points from the total count
    tot_checkpoints = (n_checkpoints * n_parts) - (n_parts-1)

    # define the interval of values to be placed into the sin function
    v_span = np.linspace(0, xy_track_lenght, tot_checkpoints)

    f = n_reps / xy_track_lenght    # frequency
    coeff = 2 * np.pi * f           # sin(2*pi*f*x)

    # apply to each sin wave a different amplitude
    if len(amplitude) == 1:
        # Define the z checkpoints. The z checkpoints are a sin function with a frequency that is proportional to the lenght of the xy path
        Checkpoints_z = offset + (amplitude * np.sin(coeff * v_span))
    
    # if the number of amplitudes is equal or bigger to the number of repetitions, then take only the first n_reps amplitudes
    elif len(amplitude) >= n_reps:
        # initialize Checkpoints_z as empty
        Checkpoints_z = np.array([])    
        
        for i in range (0, n_reps):
            shift = math.floor( tot_checkpoints / n_reps )
            start = i * shift
            stop  = start + shift

            if i < n_reps - 1:
                # not last repetition -> apply the batch [start:stop]
                Checkpoints_z = np.concatenate( ( Checkpoints_z, offset + amplitude[i] * np.sin(coeff * v_span[ start : stop ]) ), axis = 0)
            else:
                # last repetition -> take the remaining points [start:end]
                Checkpoints_z = np.concatenate( ( Checkpoints_z, offset + amplitude[i] * np.sin(coeff * v_span[ start : ]) ), axis = 0 )
    
    # if the number of amplitudes is lower than the number of repetitions, then repeat them in loop
    else:
        Checkpoints_z = np.array([])    # initialize Checkpoints_z as empty
        k = 0                           # counter for the amplitude vector   

        for i in range (0, n_reps):
            shift = math.floor( tot_checkpoints / n_reps )
            start = i * shift
            stop  = start + shift

            if i < n_reps - 1:
                # not last repetition -> apply the batch [start:stop]
                if k < len(amplitude) - 1:
                    # not last batch of amplitude -> take it and increase the counter
                    Checkpoints_z = np.concatenate( ( Checkpoints_z, offset + amplitude[k] * np.sin(coeff * v_span[ start : stop ]) ), axis = 0)
                    k = k + 1
                else:
                    # last batch of amplitude -> take it and reset the counter to restart
                    Checkpoints_z = np.concatenate( ( Checkpoints_z, offset + amplitude[k] * np.sin(coeff * v_span[ start : stop]) ), axis = 0 )
                    k = 0
            else:
                # last repetition -> take the remaining points [start:end]
                    Checkpoints_z = np.concatenate( ( Checkpoints_z, offset + amplitude[k] * np.sin(coeff * v_span[ start : ]) ), axis = 0 )


    return Checkpoints_z


# generates a track by means of a succession of splines starting from a set of points
def spline_assembler(npoints, splines):
    '''
    npoints = number of Checkpoint points for each spline
    tpoints = linspace between 0 and 1 of length npoints
    splines = array of arrays. Each element of the array is used to generate a spline, since it contains (sorted by column) the succession of points used to generate the spline.
    The order is: t(parameter), x, y, z, tg_val_init
    So: splines = [ [spline1] , [spline2] ... ]
        spline1 = [ [t extremities of the param], [starting x point for spline1], [starting y point for spline1] ... ]
    '''

    # define a linspace to extract the same number of x(t), y(t), z(t) parametrized coordinates once that the spline is generated
    tpoints = np.linspace(0,1, npoints)

    # initialize the vector of points
    x_f, y_f, z_f = [],[],[]
    first_iteration = 1

    # repeat the process for each spline
    for idx_sp in range(0,len(splines)):
        # extract the parameters related to the spline that we're about to generate
        spline = splines[idx_sp]    
        t = spline[0]
        x_st = spline[1]
        y_st = spline[2]
        z_st = spline[3]
        tg_st = spline[4]

        if idx_sp < len(splines) - 1:
            # if it's not the last spline, then the end of the spline is the same as the start of the following spline
            tg_end = splines[idx_sp + 1][4]
            x_end = splines[idx_sp + 1][1]
            y_end = splines[idx_sp + 1][2]
            z_end = splines[idx_sp + 1][3]
        else:
            # if it's the last spline, then the end of the spline is the same as the start
            tg_end = splines[0][4]
            x_end = splines[0][1]
            y_end = splines[0][2]
            z_end = splines[0][3]

        # define the points needed for the interpolation (start and end of the spline)
        x = [x_st, x_end]
        y = [y_st, y_end]
        z = [z_st, z_end]

        # assemble the points stacking them by columns
        points = np.c_[x,y,z]

        # Create a tridimensional cubic spline imposing the values of the first derivative
        cs = CubicSpline(t, points, bc_type=((1, tg_st), (1, tg_end)))

        # extract the x-y-z coordinates parametrized in t (here p stands for parametrized)
        xp = cs(tpoints)[:,0]
        yp = cs(tpoints)[:,1]
        zp = cs(tpoints)[:,2]

        if first_iteration:
            # the first iteration must include the first element to close the loop
            x_f.extend(xp)
            y_f.extend(yp)
            z_f.extend(zp)
            first_iteration = 0
        else:
            # the following iterations must not include the first element in order to avoid overlapping
            x_f.extend(xp[1:])
            y_f.extend(yp[1:])
            z_f.extend(zp[1:])


    # build the overall path (made by npoints*nsplines elements)
    Checkpoints_x = x_f
    Checkpoints_y = y_f
    Checkpoints_z = z_f

    return Checkpoints_x, Checkpoints_y, Checkpoints_z



# This function finds the parameter s of the point on the curve r(s) which provides minimum distance between the path and the vehicle
def find_s_of_closest_point_on_global_path(x_y_z_state, s_vals_global_path, x_vals_original_path, y_vals_original_path, z_vals_original_path, previous_index, estimated_ds):
    # finds the minimum distance between two consecutive points of the path
    min_ds = np.min(np.diff(s_vals_global_path))
    # find a likely number of idx steps of s_vals_global_path that the vehicle might have travelled
    estimated_index_jumps = math.ceil(estimated_ds / min_ds)
    # define the minimum number of steps that the vehicle could have traveled, if it was moving
    minimum_index_jumps = math.ceil(0.1 / min_ds)

    # in case the vehicle is still, ensure a minimum search space to account for localization error
    if estimated_index_jumps < minimum_index_jumps:
        estimated_index_jumps = minimum_index_jumps

    # define the search space span
    Delta_indexes = estimated_index_jumps * 3

    # takes the previous index (where the vehicle was) and defines a search space around it
    start_i = previous_index - Delta_indexes
    finish_i = previous_index + Delta_indexes

    # check if start_i is negative and finish_i is bigger than the path, in which case the search space is wrapped around
    if start_i < 0:
        s_search_vector = np.concatenate((s_vals_global_path[start_i:], s_vals_global_path[: finish_i]), axis=0)
        x_search_vector = np.concatenate((x_vals_original_path[start_i:], x_vals_original_path[: finish_i]), axis=0)
        y_search_vector = np.concatenate((y_vals_original_path[start_i:], y_vals_original_path[: finish_i]), axis=0)
        z_search_vector = np.concatenate((z_vals_original_path[start_i:], z_vals_original_path[: finish_i]), axis=0)

    elif finish_i > s_vals_global_path.size:
        s_search_vector = np.concatenate((s_vals_global_path[start_i:], s_vals_global_path[: finish_i - s_vals_global_path.size]), axis=0)
        x_search_vector = np.concatenate((x_vals_original_path[start_i:], x_vals_original_path[: finish_i - s_vals_global_path.size]), axis=0)
        y_search_vector = np.concatenate((y_vals_original_path[start_i:], y_vals_original_path[: finish_i - s_vals_global_path.size]), axis=0)
        z_search_vector = np.concatenate((z_vals_original_path[start_i:], z_vals_original_path[: finish_i - s_vals_global_path.size]), axis=0)
    else:
        s_search_vector = s_vals_global_path[start_i: finish_i]
        x_search_vector = x_vals_original_path[start_i: finish_i]
        y_search_vector = y_vals_original_path[start_i: finish_i]
        z_search_vector = z_vals_original_path[start_i: finish_i]

    # remove the last value to avoid ambiguity since first and last value may be the same
    distances = np.zeros(s_search_vector.size)
    for ii in range(0, s_search_vector.size):
        # evaluate the distance between the vehicle (x_y_z_state[0:3]) and each point of the search vector (i.e. of the path)
        distances[ii] = math.dist([x_search_vector[ii], y_search_vector[ii], z_search_vector[ii]], x_y_z_state[0:3])

    # retrieve the index of the minimum distance element
    local_index = np.argmin(distances)

    # check if the found minimum is on the boarder (indicating that the real minimum is outside of the search vector)
    # this offers some protection against failing the local search but it doesn't fix all of the possible problems
    # for example if path loops back (like a bean shape)
    # then you can still get an error (If you have lane boundary information then you colud put a check on the actual value of the min)
    if local_index == 0 or local_index == s_search_vector.size-1:
        print('search vector was not long enough, doing search on full path')
        distances_2 = np.zeros(s_vals_global_path.size)
        for ii in range(0, s_vals_global_path.size):
            distances_2[ii] = math.dist([x_vals_original_path[ii], y_vals_original_path[ii], z_vals_original_path[ii]], x_y_z_state[0:3])
        index = np.argmin(distances_2)
    else:
        index = np.where(s_vals_global_path == s_search_vector[local_index])
        # extract an int from the "where" operand
        index = index[0]
        index = index[0]


    s = float(s_vals_global_path[index])
    return s, index



# This function takes the actual s parameter of the path and provide the cheby coeff of the local path (i.e. the path of lenght [a,b])
def evaluate_local_path_Chebyshev_coefficients_high_order_cheby(s, x_of_s, y_of_s, z_of_s, Ds_forecast, s_vals_global_path, loop_path, Cheby_data_points):
    # obtain the s values of the local path
    s_subpath_cheby_for_xyz_data_generation, s_subpath_for_fitting_operation = produce_s_local_path(s, Ds_forecast, Cheby_data_points, s_vals_global_path, loop_path)

    # from the x-y points of the global path take just the subset that is needed for the local path
    x_data_points_Cheby = x_of_s(s_subpath_cheby_for_xyz_data_generation)
    y_data_points_Cheby = y_of_s(s_subpath_cheby_for_xyz_data_generation)
    z_data_points_Cheby = z_of_s(s_subpath_cheby_for_xyz_data_generation)

    # fit the data points with a chebychev polynomial
    coeffx = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, x_data_points_Cheby, 19)
    coeffy = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, y_data_points_Cheby, 19)
    coeffz = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, z_data_points_Cheby, 19)

    # find the min s and max s of the local path
    a = s_subpath_for_fitting_operation.min()
    b = s_subpath_for_fitting_operation.max()
    
    return coeffx, coeffy, coeffz, a, b



# produces the s vectors of the local path
def produce_s_local_path(s,Ds_forecast,Cheby_data_points,s_vals_global_path,loop_path):
    # allow for some track behind the vehicle
    Ds_backwards = 0.05 * Ds_forecast
                                                         #start value      stop value       #number of equally spaced points
    s_subpath_cheby_for_xyz_data_generation = np.linspace(s - Ds_backwards, s + Ds_forecast, Cheby_data_points)
    s_subpath_for_fitting_operation = np.linspace(s - Ds_backwards, s + Ds_forecast, Cheby_data_points)

    # if there is a loop path enable, wrap around in case the subpath exceeds the path limits
    if loop_path:
        s_subpath_cheby_for_xyz_data_generation[s_subpath_cheby_for_xyz_data_generation > s_vals_global_path.max()] = s_subpath_cheby_for_xyz_data_generation[s_subpath_cheby_for_xyz_data_generation > s_vals_global_path.max()] - s_vals_global_path.max()
        s_subpath_cheby_for_xyz_data_generation[s_subpath_cheby_for_xyz_data_generation < s_vals_global_path.min()] = s_vals_global_path.max() + s_subpath_cheby_for_xyz_data_generation[s_subpath_cheby_for_xyz_data_generation < s_vals_global_path.min()] 
    else:
        # cap upper value
        s_subpath_cheby_for_xyz_data_generation[s_subpath_cheby_for_xyz_data_generation > s_vals_global_path[-1]] = \
        s_vals_global_path[-1]
        # cap lower value
        s_subpath_cheby_for_xyz_data_generation[s_subpath_cheby_for_xyz_data_generation < s_vals_global_path[0]] = \
        s_vals_global_path[0]
        # set both s_vectors to be equal
        s_subpath_for_fitting_operation = s_subpath_cheby_for_xyz_data_generation
        
    return s_subpath_cheby_for_xyz_data_generation, s_subpath_for_fitting_operation

