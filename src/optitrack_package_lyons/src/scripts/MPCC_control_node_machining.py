#!/usr/bin/env python3
import numpy as np
import math
from scipy import interpolate, integrate
import sys
import rospy
import os
from functions_for_MPCC_node_running import evaluate_spline_quantities_cheby, evaluate_local_path,\
                                                  s_eval_a_posteriori_numerically, unpack_parameters_cheby,\
                                                  find_s_of_closest_point_on_global_path,\
                                                  evaluate_spline_quantities_machining_MPCC_cheby,\
                                                  evaluate_local_path_Chebyshev_coefficients,\
                                                  produce_track
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import shelve
from datetime import datetime
from matplotlib.patches import Rectangle


from custom_msgs_optitrack.msg import custom_opti_pose_stamped_msg
#import forcespro (add path before you add it)
sys.path.insert(0, '/home/lorenzo/OneDrive/PhD/ubuntu_stuff/Forces_pro_extracted/forces_pro_client')
import forcespro.nlp


#Unfortunatly there seems to be no function "get latest value on a topic" so global
#values are defined and updated by callback functions in the relative subscribers
#for now they will be used as initial values cause they are not updated

                 # x y eta
state = np.array([float(0), float(0), float(0)])  # initialize state variable
velocity = 0.0  # initialize global variables
opti_time_stamp_secs = 0
safety_value = 0

# callbacks that update these evaluate_spline_quantities_cheby
def velocity_subscriber_callback(msg):
    # need global keyword to overwrite global variable
    global velocity
    velocity = msg.data

def state_subscriber_callback(msg):
    # need global keyword to overwrite global variable
    global state, opti_time_stamp_secs
    # state[0] = msg.x
    # state[1] = msg.y
    # state[2] = msg.rotation
    state[0] = msg.pose.position.x
    state[1] = msg.pose.position.y
    state[2] = msg.pose.orientation.y
    stamp = msg.header.stamp
    opti_time_stamp_secs = stamp.to_sec()


def safety_value_subscriber_callback(msg):
    # need global keyword to overwrite global variable
    global safety_value
    safety_value = msg.data

dyn_ob_traj_x = np.zeros(15)
dyn_ob_traj_y = np.zeros(15)
dyn_ob_traj_r = np.zeros(15)
def occupancy_xyr_2_subscriber_callback(msg):
    global dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r
    dyn_ob_traj = msg.data
    dyn_ob_traj_x = dyn_ob_traj[0:15]
    dyn_ob_traj_y = dyn_ob_traj[15:30]
    dyn_ob_traj_r = dyn_ob_traj[30:]

def MPCC_controller():
    #set up publishers and subscribers 
    car_number = 4
    # choose what solver you want to build
    # 1 = cheby solver
    # 2 = machining cheby solver
    # 3 = large Ds formulation
    # 4 = Forward Euler
    solver_choice = 1

    # this takes the machining solver
    if solver_choice == 1:
        solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_Cheby_from_python"
        solver_name = 'Cheby_full_'
    elif solver_choice == 2:
        solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_machining_MPCC_Cheby_path"
        solver_name = 'Machining_'
    elif solver_choice == 3:
        solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_Cheby_Large_DS_formulation"
        solver_name = 'Cheby_large_DS'
    elif solver_choice == 4:
        solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_Forward_euler"
        solver_name = 'Cheby_forward_Euler'
    else:
        print('invalid solver choice')

    solver = forcespro.nlp.Solver.from_directory(solver_location)


    throttle_publisher = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size = 1)
    steering_publisher = rospy.Publisher('steering_' + str(car_number), Float32, queue_size = 1)
    velocity_subscriber = rospy.Subscriber('velocity_' + str(car_number), Float32, velocity_subscriber_callback)
    state_subscriber = rospy.Subscriber('Optitrack_data_topic_' + str(car_number), PoseStamped, state_subscriber_callback)
    safety_value_subscriber = rospy.Subscriber('safety_value', Float32, safety_value_subscriber_callback)
    occupancy_xyr_2_subscriber = rospy.Subscriber('occupancy_xyr_2', Float32MultiArray, occupancy_xyr_2_subscriber_callback)
    occupancy_xyr_4_publisher = rospy.Publisher('occupancy_xyr_4', Float32MultiArray, queue_size=10)
    rospy.init_node('MPCC_node_' + str(car_number), anonymous=False)
    rate = rospy.Rate(10)



    # setu up N
    # setu up N
    N = 15  # must match the number of stages in the solver

    # generate parameters
    V_target = 0.8  # in terms of being scaled down the proportion is vreal life[km/h] = v[m/s]*42.0000  (assuming the 30cm jetracer is a 3.5 m long car)
    L = 0.175  # length of the jetracer [m]
    # numerical values taken from BEP paper, so linearized around th = 0.129 --> V=1 m/s
    C = 1.54 / 1.63  # longitudinal damping coefficient divided by the mass
    a_th = 60 / 1.63  # motor curve coefficient divided by the mass
    b_th = 1.54 / 1.63  # motor curve offset (actually not used at the moment)
    dt = 0.1 # so first number is the prediction horizon in seconds -this is the dt of the solver so it will think that the control inputs are changed every dt seconds
    # xo = -0.05
    # yo = 1.65
    xo = 10
    yo = 10
    # ro = 0.043     #a scaled down pedestrian (circle 1 mt wide) is r = 0.043
    ro = 0.11  # something like another vehicle
    # so to avoid going through the pedestrian dt should be less than dt*v<r
    # -->   dt<r/v
    # put them all together
    # define dynamic obstacle position and radius as a function of time
    # x_dyn_0 = 2
    # y_dyn_0 = -1
    # r_dyn_0 = 0.15

    l_shift = 0  # shift of centre of lane to the right
    l_width = 1.5  # width of lane
    if solver_choice == 1 or solver_choice == 2 or solver_choice == 3:
        #j = q1 * (s_dot - V_target) ** 2 + q2 * err_lat_squared + q3 * u[0] ** 2 + q4 * u[1] ** 2
        q1 = 10  # relative weight of s_dot following
        q2 = 0.1  # relative weight of lat error
        q3 = 1  # relative weight of steering change (Weighting the two inputs identically to be fair
                  # vs machining formulation)
        q4 = q3  # relative weight of ref_velocity change

    elif solver_choice == 2:
        # j = q1 * (x[3] - V_target) ** 2 + q2 * err_lat_squared + q3 * err_lag_squared + q4 * u[0] ** 2 + q4 * u[1] ** 2
        q1 = 10  # relative weight of s_dot following
        q2 = 1  # relative weight of lat error
        q3 = 0.1  # relative weight of lag error
        q4 = 0.1  # relative weight of inputs  (weighted the same here)







    # s_dot formulation
    # J = q1 * (s_dot - V_target) ** 2 + q2 * err_lat_squared + q3 * Du[0] ** 2 + q4 * Du[1] ** 2
    # problem_related_parameters = np.array(
    #     [V_target, L, C, a_th, b_th, dt, xo, yo, ro, *dyn_ob_traj_x, *dyn_ob_traj_y, *dyn_ob_traj_r, l_shift, l_width,
    #      q1, q2, q3, q4])

    # generate path coefficients
    # ellyps
    # Checkpoints_x = 0.7 * np.array([-2, 2, 2, -2, -2])  # circle (scale by 0.7 in the lab)
    # Checkpoints_y = 0.7 * np.array([-1, -1, +1, +1, -1])
    # # Checkpoints_x = 1 * np.array([0, 2, 2, 0, 0]) #circle (scale by 0.7 in the lab)
    # Checkpoints_y = 1 * np.array([0, 0, 2, 2, 0])
    # Checkpoints_x = 0.5 * np.array([0, 2, 2, 0, 0]) #circle (scale by 0.7 in the lab)
    # Checkpoints_y = 0.5 * np.array([0, 0, 2, 2, 0])

    #Checkpoints_x = 10 * np.array([0, 1, 2, 3, 4])  # straight line
    #Checkpoints_y = 1 * np.array([0, 0, 0, 0, 0])

    # # generating a curved arc of a given radius with a lot of control points
    # n_checkppoints = 10
    # R = 1  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.82
    # theta_init2 = np.pi * -0.5
    # theta_end2 = np.pi * 0.5
    # theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkppoints)
    # theta_init4 = np.pi * 0.5
    # theta_end4 = np.pi * 1.5
    # theta_vec4 = np.linspace(theta_init4, theta_end4, n_checkppoints)
    # Checkpoints_x1 = np.linspace(- 1.5 * R, 1.5 * R, n_checkppoints)
    # Checkpoints_y1 = np.zeros(n_checkppoints) - R
    # Checkpoints_x2 = 1.5 * R + R * np.cos(theta_vec2)
    # Checkpoints_y2 = R * np.sin(theta_vec2)
    # Checkpoints_x3 = np.linspace(1.5 * R, -1.5*R, n_checkppoints)
    # Checkpoints_y3 = R * np.ones(n_checkppoints)
    # Checkpoints_x4 = -1.5* R + R * np.cos(theta_vec4)
    # Checkpoints_y4 = R * np.sin(theta_vec4)
    #
    # Checkpoints_x = [*Checkpoints_x1[0:n_checkppoints - 1], *Checkpoints_x2[0:n_checkppoints - 1],
    #                  *Checkpoints_x3[0:n_checkppoints - 1], *Checkpoints_x4[0:n_checkppoints]]
    # Checkpoints_y = [*Checkpoints_y1[0:n_checkppoints - 1], *Checkpoints_y2[0:n_checkppoints - 1],
    #                  *Checkpoints_y3[0:n_checkppoints - 1], *Checkpoints_y4[0:n_checkppoints]]


    # #double donught
    # n_checkppoints = 10
    # R = 1  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.82
    # theta_init1 = np.pi * -0.5
    # theta_end1 = np.pi * 0.0
    # theta_vec1 = np.linspace(theta_init1, theta_end1, n_checkppoints)
    # theta_init2 = np.pi * 1
    # theta_end2 = np.pi * -1
    # theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkppoints)
    # theta_init3 = np.pi * 0
    # theta_end3 = np.pi * 1.5
    # theta_vec3 = np.linspace(theta_init3, theta_end3, n_checkppoints)
    #
    # Checkpoints_x1 = - R + R * np.cos(theta_vec1)
    # Checkpoints_y1 = - R + R * np.sin(theta_vec1)
    # Checkpoints_x2 = + R + R * np.cos(theta_vec2)
    # Checkpoints_y2 = - R + R * np.sin(theta_vec2)
    # Checkpoints_x3 = - R + R * np.cos(theta_vec3)
    # Checkpoints_y3 = - R + R * np.sin(theta_vec3)
    #
    # Checkpoints_x = [*Checkpoints_x1[0:n_checkppoints - 1], *Checkpoints_x2[0:n_checkppoints - 1],
    #                  *Checkpoints_x3[0:n_checkppoints]]
    # Checkpoints_y = [*Checkpoints_y1[0:n_checkppoints - 1], *Checkpoints_y2[0:n_checkppoints - 1],
    #                  *Checkpoints_y3[0:n_checkppoints]]
    #
    #
    #
    #
    # #straight line
    # # Checkpoints_x = np.linspace(0, 100, n_checkppoints)
    # # Checkpoints_y = np.zeros(n_checkppoints)

    #generate track
    choice = 'savoiardo'
    # choice = 'double_donught'
    # choice = 'straight_line'
    n_checkppoints = 10

    Checkpoints_x, Checkpoints_y = produce_track(choice, n_checkppoints)


    # this is a bit strange so work on improving this thing
    loop_path = True
    tck, u = interpolate.splprep([Checkpoints_x, Checkpoints_y], s=0, k=3)

    # evaluate arc length given a certain discretization
    spline_discretization = 1000
    t_vec = np.linspace(0, u[-1], spline_discretization)
    xy_vals = interpolate.splev(t_vec, tck)

    x_vals_original_path = xy_vals[0]
    y_vals_original_path = xy_vals[1]

    # evaluate path parameters by numerically integrating
    s_vals_global_path = np.zeros(spline_discretization)
    # define infinitesimal arc length function
    ds = lambda t: np.sqrt(np.dot(interpolate.splev(t, tck, der=1, ext=0), interpolate.splev(t, tck, der=1, ext=0)))

    for ii in range(1, spline_discretization):
        ds_precision = integrate.quad(ds, t_vec[ii - 1], t_vec[ii])
        s_vals_global_path[ii] = s_vals_global_path[ii - 1] + ds_precision[0]

    # generate splines  x(s) and y(s) where s is now the arc length value (starting from 0)
    x_of_s = interpolate.CubicSpline(s_vals_global_path, x_vals_original_path)
    y_of_s = interpolate.CubicSpline(s_vals_global_path, y_vals_original_path)

    #set initial index for closest point on the path search
    previous_index = 0

    #this is needed to plot the figure in real time
    plt.ion()

    #initiate data storage variables
    output_array_stacked = np.array([0, 0, 0, 0, 0, 0, 0])
    time_stamp_stacked = np.array([0])
    safety_value_stacked = np.array([0])
    params_i_array_stacked = np.array(np.zeros(82))
    exitflag_stacked = np.array([0])
    total_time_stacked = np.array([0])
    solver_time_stacked = np.array([0])
    Full_plotting_time_stacked = np.array([0])
    #add problem every time instant

    # #set initial position of dynamic obstacle
    # s_0_dyn_obst = 2
    # s_dot_dyn_obst = 0.5
    # r_enlarge_rate_dyn_obst = 0.1
    # r_0_dyn_obst = 0.15
    #
    while not rospy.is_shutdown():
        # get clock estimated for timing purpouses
        start_clock_time = rospy.get_rostime()


    #     # evaluate dynamic obstacles' occupancy
    #     dyn_ob_traj_x = np.zeros(N)
    #     dyn_ob_traj_y = np.zeros(N)
    #     dyn_ob_traj_r = np.zeros(N)
    #     for ii in range(N):
    #         dyn_ob_traj_x[ii] = x_of_s(s_0_dyn_obst + s_dot_dyn_obst * dt * ii)
    #         dyn_ob_traj_y[ii] = y_of_s(s_0_dyn_obst + s_dot_dyn_obst * dt * ii)
    #         dyn_ob_traj_r[ii] = r_0_dyn_obst + r_enlarge_rate_dyn_obst * dt * ii  # enalrge radius to simulate uncertainty
    #
    #     # keep the dynamic obstacle moving
    #     s_0_dyn_obst = s_0_dyn_obst + s_dot_dyn_obst * dt
    #     if s_0_dyn_obst > s_vals_global_path[-1]:
    #         s_0_dyn_obst = s_0_dyn_obst - s_vals_global_path[-1]

        problem_related_parameters = np.array([V_target, L, C, a_th, b_th, dt, xo, yo, ro,
                                               *dyn_ob_traj_x, *dyn_ob_traj_y, *dyn_ob_traj_r, l_shift,
                                               l_width, q1, q2, q3, q4])



        #define initial position
        #note that the initial s value will be over written with the measured one
        #                  x = [x y  rotation velocity s]
        #Get current state (in real application the state is obtained from sensors)
        #here it is updated automatically by the state listeners

        #measure the closest point on the global path
        estimated_ds = velocity * dt  # esitmated ds from previous time instant (velocity is measured now so not accounting for acceleration, but this is only for the search of the s initial s value, so no need to be accurate)
        s, current_index = find_s_of_closest_point_on_global_path(np.array([state[0], state[1]]), s_vals_global_path, x_vals_original_path, y_vals_original_path, previous_index, estimated_ds)
        previous_index = current_index  # update index along the path to know where to search in next iteration

        #define initial condition for the solver
        xinit = np.array([state[0], state[1], state[2], velocity, s])

        # define initial guess for solver (chosen as zero input so static initial state)
        x0 = np.concatenate(([0, 0], xinit), axis=0)
        x0_array = np.tile(x0, N)  # concatenate for each stage

        # evaluate local path
        # estimated needed path parameter length
        Ds_forecast = V_target * dt * N * 1.5
        Cheby_data_points = 100

        a = s  # be carefull between resetting to 0 or keeping measured value
        b = a + Ds_forecast

        x_y_state = xinit[0: 2]

        coeffx, coeffy = evaluate_local_path_Chebyshev_coefficients(s, x_of_s, y_of_s, Ds_forecast, s_vals_global_path, loop_path,
                                                          Cheby_data_points)

        # assemble local path related parameters
        params_i = np.concatenate((np.array([a]), np.array([b]), coeffx, coeffy, problem_related_parameters), axis=0)
        all_params_array = np.tile(params_i, N)

        all_params_array = np.tile(params_i, N) # give as a stacked vector of parameters (see codeoptions.nlp.stack_parambounds in solver generation phase)

        # produce the problem as a dictionary for the solver
        problem = {"x0": x0_array, "xinit": xinit, "all_parameters": all_params_array}
        # add parametric bounds (only s changes online)
        problem["lb"] = np.tile([0.10, -20.0 / 180.0 * np.pi, a], N)
        problem["ub"] = np.tile([0.25, 20.0 / 180.0 * np.pi, b], N)

        # call solver
        output, exitflag, info = solver.solve(problem)
        output_array = np.array(list(output.values()))


        #check if solver converged
        if exitflag != 1:
            print('Ouch solver did not converge (exitflag different than 1)')


        # publish input values
        throttle_val = Float32(output_array[0, 0])
        steering_val = Float32(output_array[0, 1] / (20.0 / 180.0 * np.pi))
        throttle_publisher.publish(throttle_val)
        steering_publisher.publish(steering_val)

        #publish occupancy for other vehicles
        r_enlarge_rate_dyn_obst = 0
        r_0_dyn_obst = 0.1
        dyn_ob_message_4 = Float32MultiArray()
        dyn_ob_message_4.data = np.zeros(45)
        dyn_ob_traj_r_4 = np.linspace(r_0_dyn_obst, r_0_dyn_obst + r_enlarge_rate_dyn_obst * dt * N, N)


        # simulating broken down vehicle
        # dummy_x = state[0] * np.ones(N)
        # dummy_y = state[0] * np.ones(N)
        # dyn_ob_message_4.data = [*dummy_x, *dummy_y, *dyn_ob_traj_r_4]

        #moving vehicle
        dyn_ob_message_4.data = [*output_array[:, 2], *output_array[:, 3], *dyn_ob_traj_r_4]

        occupancy_xyr_4_publisher.publish(dyn_ob_message_4)





        stop_clock_time = rospy.get_rostime()
        total_time = stop_clock_time.secs-start_clock_time.secs+(stop_clock_time.nsecs-start_clock_time.nsecs)/1000000000
        #rospy.loginfo("Elapsed time = %f [seconds]", total_time)

        #prepare data to be stored (ADD TIME VECTOR)
        output_array_stacked = np.vstack([output_array_stacked, output_array])
        time_stamp_stacked = np.vstack([time_stamp_stacked, np.array([opti_time_stamp_secs])])
        safety_value_stacked = np.vstack([safety_value_stacked, np.array([safety_value])])
        params_i_array_stacked = np.vstack([params_i_array_stacked, params_i])
        exitflag_stacked = np.vstack([exitflag_stacked, exitflag])
        total_time_stacked = np.vstack([total_time_stacked, total_time])
        solver_time_stacked = np.vstack([solver_time_stacked, info.solvetime])


        #####
        # plot x-y state prediction

        # extract x and y prediction
        x_vec = output_array[:, 2]
        y_vec = output_array[:, 3]
        # evaluating local path as solver will do it (inside continuous dynamics)
        dummy_x = [0, 0, 0, 0, 0]
        x_path_from_solver_function = np.zeros(Cheby_data_points)
        y_path_from_solver_function = np.zeros(Cheby_data_points)

        # this must match what is done inside the function "evaluate_local_path_Chebyshev_coefficients_manuel"
        # s_subpath_cheby = np.linspace(s, s + Ds_forecast, Cheby_data_points)
        #
        # for ii in range(0, Cheby_data_points):
        #     dummy_x[4] = s_subpath_cheby[ii]
        #     Cx_out, Cy_out, s_dot, x_Cdev, y_Cdev = evaluate_spline_quantities_machining_MPCC_cheby(dummy_x, params_i)
        #     x_path_from_solver_function[ii] = Cx_out
        #     y_path_from_solver_function[ii] = Cy_out
        #
        # # Plot figure
        # for ii in range(0, N):
        #     dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r
        #     xo_now = dyn_ob_traj_x[ii]
        #     yo_now = dyn_ob_traj_y[ii]
        #     ro_now = dyn_ob_traj_r[ii]
        #     circle1 = plt.Circle((xo_now, yo_now), ro_now, color='c', alpha=0.5 * (1 - ii / N))
        #     plt.gca().add_patch(circle1)
        # circle1 = plt.Circle((xo, yo), ro, color='r')
        # plt.gca().add_patch(circle1)
        # rect1 = Rectangle((x_vec[0]+(L/1.5)/2 * np.sin(state[2]), y_vec[0]-(L/1.5)/2 * np.cos(state[2])), L, (L/1.5), angle=state[2]/np.pi*180, color='green')
        # plt.gca().add_patch(rect1)
        # plt.plot(x_vals_original_path, y_vals_original_path, label="Original global path")
        # plt.scatter(x_vals_original_path[current_index], y_vals_original_path[current_index], label="current closest point on path")
        # plt.plot(x_path_from_solver_function, y_path_from_solver_function, label="Local path from inside solver function")
        # plt.plot(x_vec, y_vec, label="xy_state_prediction")
        # plt.ylabel('x')
        # plt.ylabel('y')
        # plt.axis('equal')
        # plt.legend()
        # plt.title(solver_name)
        # # plt.xlim(x_vec[0]-2.5, x_vec[0]+2.5)
        # # plt.ylim(y_vec[0]-2, y_vec[0]+2)
        # plt.xlim(-3, +3)
        # plt.ylim(-2, +2)
        # plt.show()
        # plt.pause(0.001)
        # plt.clf()
        #
        # #####

        stop_clock_time_plots = rospy.get_rostime()
        total_time_plots = stop_clock_time_plots.secs - start_clock_time.secs + (
                    stop_clock_time_plots.nsecs - start_clock_time.nsecs) / 1000000000
        #rospy.loginfo("Elapsed time with plots= %f [seconds]", total_time_plots)
        Full_plotting_time_stacked = np.vstack([Full_plotting_time_stacked, total_time_plots])
        
        rate.sleep()


    #plot computation times
    plt.figure(2)
    plt.plot(total_time_stacked[1:], label="total_time")
    plt.plot(solver_time_stacked[1:], label="solver_time")
    plt.plot(Full_plotting_time_stacked[1:], label="Full iteration time with plotting")
    plt.ylabel('iteration')
    plt.ylabel('seconds')
    plt.legend()
    plt.show()
    plt.pause(5)


    
    #data storage
    #save the problem to a file for later use
    now = datetime.now() # current date and time
    date = now.strftime("%m_%d_%Y")
    time = now.strftime("%H_%M_%S")

    #get time and date
    filename = '/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/MPCC_Lyons/Data_storage_lyons/'+ solver_name + date + time + '.db'
    d = shelve.open(filename, writeback=False)

    try:
        d['MPCC_results'] = {'problem_related_parameters':[problem_related_parameters], 'params_i_array_stacked': params_i_array_stacked[1:,:],
         'output_array_stacked': output_array_stacked[1:,:], 'time_stamp_stacked':time_stamp_stacked[1:,:],
         'safety_value_stacked':safety_value_stacked[1:,:],'exitflag_stacked':exitflag_stacked[1:,:],'total_time_stacked':total_time_stacked[1:,:],
         'solver_time_stacked':solver_time_stacked[1:,:],'s_vals_global_path':[s_vals_global_path], 'x_vals_original_path':[x_vals_original_path],
         'y_vals_original_path':[y_vals_original_path]}
        #d['MPCC_results'] = { 'constant_problem_parameters': constant_problem_parameters, 'other_parameters': np.array([omega]),'output_array_stacked': output_array_stacked, 'Theta_array_1_2_5': Theta_array_1_2_5,'safety_value_list':safety_value_list}
    finally:
        d.close()


if __name__ == '__main__':
    try:
        MPCC_controller()
    except rospy.ROSInterruptException:
        pass
