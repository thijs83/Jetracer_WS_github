#!/usr/bin/env python3

# this script simulates the output from the optitrack, so you can use it for tests
# the 4 vehicles are all alligne spced 0.5 m apart

import rospy
from std_msgs.msg import String, Float32, Float32MultiArray
#from custom_msgs_optitrack.msg import custom_opti_pose_stamped_msg
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy import interpolate, integrate
from functions_for_MPCC_node_running import find_s_of_closest_point_on_global_path, produce_track
from matplotlib.patches import Rectangle



#set up global variables and relative callbacks
safety_value = 0
def safety_value_subscriber_callback(msg):
    # need global keyword to overwrite global variable
    global safety_value
    safety_value = msg.data

state = np.array([float(0), float(0), float(0)])  # initialize state variable
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
    opti_time_stamp_secs = float(stamp.to_sec())


def forwards_integrate():
    car_number = 3

    #set up subscriber nodes
    safety_value_subscriber = rospy.Subscriber('safety_value', Float32, safety_value_subscriber_callback)
    state_subscriber = rospy.Subscriber('Optitrack_data_topic_' + str(car_number), PoseStamped,
                                        state_subscriber_callback)

    #set up publisher handles
    occupancy_xyr_4_publisher = rospy.Publisher('occupancy_xyr_4', Float32MultiArray, queue_size=10)

    # initialize current node and set publishing rate
    rospy.init_node('dynamic_obstacle_integrator', anonymous=False)
    dt_controller = 0.1
    rate = rospy.Rate(1 / dt_controller)

    # create empty messages
    #occupancy of dynamic obstacle
    dyn_ob_message = Float32MultiArray()
    dyn_ob_message.data = np.zeros(90)

    #initiate dynamic obstacle
    #set initial position of dynamic obstacle
    N = 30
    dyn_ob_traj_x = np.zeros(N) + 10
    dyn_ob_traj_y = np.zeros(N) + 10
    dyn_ob_traj_r = np.zeros(N)
    s_0_dyn_obst_reset = 3.5
    s_0_dyn_obst = s_0_dyn_obst_reset
    s_dot_dyn_obst_selection = [0.3, 0.3]
    s_dot_dyn_obst = s_dot_dyn_obst_selection[0]
    reset_count = 0
    r_enlarge_rate_dyn_obst = 0.01
    r_0_dyn_obst = 0.15


    # generate track
    # choice = 'savoiardo'
    # choice = 'double_donut'
    # choice = 'straight_line'
    # choice = 'savoiardo_saturate_steering'
    # choice = 'circle'
    # choice = 'racetrack_saturate_steering'
    choice = 'racetrack_Lab'
    n_checkppoints = 30

    Checkpoints_x, Checkpoints_y = produce_track(choice, n_checkppoints)


    # this is a bit strange so work on improving this thing
    loop_path = True
    tck, u = interpolate.splprep([Checkpoints_x, Checkpoints_y], s=0, k=3)

    # evaluate arc length given a certain discretization
    spline_discretization = 100
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

    previous_index = 0

    while not rospy.is_shutdown():
        #only activates if safety is off
        if safety_value != 0:
            # evaluate dynamic obstacles' occupancy
            #determine closest dynamic obstacle

            for ii in range(N):
                s_dyn_pred_i = s_0_dyn_obst + s_dot_dyn_obst * dt_controller * ii
                if s_dyn_pred_i > s_vals_global_path[-1]:
                    s_dyn_pred_i = s_dyn_pred_i - s_vals_global_path[-1]
                dyn_ob_traj_x[ii] = x_of_s(s_dyn_pred_i)
                dyn_ob_traj_y[ii] = y_of_s(s_dyn_pred_i)
                dyn_ob_traj_r[
                    ii] = r_0_dyn_obst + r_enlarge_rate_dyn_obst * dt_controller * ii  # enalrge radius to simulate uncertainty

            # keep the dynamic obstacle moving
            s_0_dyn_obst = s_0_dyn_obst + s_dot_dyn_obst * dt_controller
            #reset after you performed a lap
            if s_0_dyn_obst > s_vals_global_path[-1]:
                s_0_dyn_obst = s_0_dyn_obst - s_vals_global_path[-1]

        #print('s = ', s, '       s_0_dyn_obst', s_0_dyn_obst)
        # check if reset condition is met
        estimated_ds = 1 * dt_controller
        s, current_index = find_s_of_closest_point_on_global_path(np.array([state[0], state[1]]), s_vals_global_path,
                                                                  x_vals_original_path, y_vals_original_path,
                                                                  previous_index, estimated_ds)
        previous_index = current_index


        if s - s_0_dyn_obst > 0.5 and s - s_0_dyn_obst < 0.5 * s_vals_global_path[-1]:
            s_0_dyn_obst = s + s_0_dyn_obst_reset  # reset in front of the ego vehicle
            s_dot_dyn_obst = s_dot_dyn_obst_selection[np.mod(reset_count, len(s_dot_dyn_obst_selection))]
            reset_count = reset_count + 1
            print('resetting to initial value, V dyn =', s_dot_dyn_obst)




        #fill in the messages and publish them
        dyn_ob_message.data = [*dyn_ob_traj_x, *dyn_ob_traj_y, *dyn_ob_traj_r]
        occupancy_xyr_4_publisher.publish(dyn_ob_message)

        rate.sleep()

if __name__ == '__main__':
    try:
        forwards_integrate()
    except rospy.ROSInterruptException:
        pass
