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

throttle_3 = 0
steering_3 = 0
def throttle_3_subscriber_callback(msg):
    # need global keyword to overwrite global variable
    global throttle_3
    throttle_3 = msg.data
def steering_3_subscriber_callback(msg):
    # need global keyword to overwrite global variable
    global steering_3
    steering_3 = msg.data




def continuous_dynamics_Jetracer(t, z):
    u = z[0:2]
    x = z[2:6]
    L  =0.175  # length of the jetracer [m]
    #numerical values taken from BEP paper, so linearized around th = 0.129 --> V=1 m/s
    c = 1.54/1.63  # longitudinal damping coefficient divided by the mass
    a_th = 60/1.63  # motor curve coefficient divided by the mass
    b_th = 6/1.63
    xdot1 = x[3] * np.cos(x[2])  # x_dot = v * cos(eta)
    xdot2 = x[3] * np.sin(x[2])  # y_dot = v * sin(eta)
    xdot3 = x[3] * np.tan(u[1]) / L  # v * tan(delta) / L
    xdot4 = -c * (x[3]) + u[0] * a_th - b_th # vdot = -C*v+a_th*throttle+b_throttle  (linear motor torque eaten by linear viscosity)
    #adding zeros to the derivatives of u since they are constant (in the timestep)
    zdot = np.array([0, 0, xdot1, xdot2, xdot3, xdot4])
    return zdot


def forwards_integrate():
    #set up subscriber nodes
    safety_value_subscriber = rospy.Subscriber('safety_value', Float32, safety_value_subscriber_callback)

    throttle_subscriber_3 = rospy.Subscriber('throttle_3', Float32, throttle_3_subscriber_callback)
    steering_subscriber_3 = rospy.Subscriber('steering_3', Float32, steering_3_subscriber_callback)

    #set up publisher handles
    test_publisher_1 = rospy.Publisher('Optitrack_data_topic_1', PoseStamped, queue_size=10)
    test_publisher_2 = rospy.Publisher('Optitrack_data_topic_2', PoseStamped, queue_size=10)
    test_publisher_3 = rospy.Publisher('Optitrack_data_topic_3', PoseStamped, queue_size=10)
    test_publisher_4 = rospy.Publisher('Optitrack_data_topic_4', PoseStamped, queue_size=10)

    test_velocity_publisher_1 = rospy.Publisher('velocity_1', Float32, queue_size=10)
    test_velocity_publisher_2 = rospy.Publisher('velocity_2', Float32, queue_size=10)
    test_velocity_publisher_3 = rospy.Publisher('velocity_3', Float32, queue_size=10)
    test_velocity_publisher_4 = rospy.Publisher('velocity_4', Float32, queue_size=10)

    occupancy_xyr_4_publisher = rospy.Publisher('occupancy_xyr_4', Float32MultiArray, queue_size=10)

    # initialize current node and set publishing rate
    rospy.init_node('Optitrack_tests', anonymous=False)
    dt_int = 0.01
    dt_controller = 0.1
    rate = rospy.Rate(1/dt_int)

    # create empty messages
    message_1 = PoseStamped()
    message_2 = PoseStamped()
    message_3 = PoseStamped()
    message_4 = PoseStamped()

    velocity_message_1 = Float32()
    velocity_message_2 = Float32()
    velocity_message_3 = Float32()
    velocity_message_4 = Float32()

    #occupancy of dynamic obstacle
    dyn_ob_message = Float32MultiArray()
    dyn_ob_message.data = np.zeros(45)

    #initiate dynamic obstacle
    #set initial position of dynamic obstacle
    N = 30
    dyn_ob_traj_x = np.zeros(N) + 10
    dyn_ob_traj_y = np.zeros(N) + 10
    dyn_ob_traj_r = np.zeros(N)
    # s_0_dyn_obst_reset = 8
    s_0_dyn_obst_reset = 2
    s_0_dyn_obst = s_0_dyn_obst_reset
    s_dot_dyn_obst_selection = [0.25, 0.25]
    s_dot_dyn_obst = s_dot_dyn_obst_selection[0]
    reset_count = 0
    r_enlarge_rate_dyn_obst = 0.01
    r_0_dyn_obst = 0.11


    # generate track
    # choice = 'savoiardo'
    # choice = 'double_donut'
    # choice = 'straight_line'
    # choice = 'savoiardo_saturate_steering'
    # choice = 'circle'
    # choice = 'racetrack_saturate_steering'
    # choice = 'racetrack_Lab'
    choice = 'gain_sweep_track_2'
    n_checkppoints = 30

    Checkpoints_x, Checkpoints_y = produce_track(choice, n_checkppoints)

    # initiate states
    state_3_reset = [Checkpoints_x[1], Checkpoints_y[1], 0, 0]
    # # state_3_reset = [-0.2, -0.341659, 0, 0]
    # state_3 = state_3_reset
    state_3 = [Checkpoints_x[1], Checkpoints_y[1], np.pi/2, 0]


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
        #perform forwards integration
        t0 = 0
        t_bound = dt_int
        #only activates if safety is off
        if safety_value == 0:
            y0 = np.array([0.1, 0] + state_3) #set throttle to 0.1 as default (corrisponds to 0 input)
        else:
            delta = - steering_3 * (20.0 / 180.0 * np.pi) # must convert back to emulate what jetracer does
            y0 = np.array([throttle_3, delta] + state_3)

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
            s_0_dyn_obst = s_0_dyn_obst + s_dot_dyn_obst * dt_int
            #reset after you performed a lap
            if s_0_dyn_obst > s_vals_global_path[-1]:
                s_0_dyn_obst = s_0_dyn_obst - s_vals_global_path[-1]

        RK45_output = integrate.RK45(continuous_dynamics_Jetracer, t0, y0, t_bound)
        dyn_ob_message.data = [*dyn_ob_traj_x, *dyn_ob_traj_y, *dyn_ob_traj_r]

        RK45_output.step()

        z_next = RK45_output.y
        # print("{:.2f}".format(z_next[0]), "{:.2f}".format(z_next[1]), "{:.2f}".format(z_next[2]),
        #       "{:.2f}".format(z_next[3]), "{:.2f}".format(z_next[4]), "{:.2f}".format(z_next[5]))
        state_3 = z_next[2:]
        state_3 = state_3.tolist()
        estimated_ds = state_3[3] * dt_int
        s, current_index = find_s_of_closest_point_on_global_path(np.array([state_3[0], state_3[1]]), s_vals_global_path,
                                                                  x_vals_original_path, y_vals_original_path,
                                                                  previous_index, estimated_ds)
        previous_index = current_index
        #print('s = ', s, '       s_0_dyn_obst', s_0_dyn_obst)
        #check if reset condition is met
        if s - s_0_dyn_obst > 0.5 and s - s_0_dyn_obst < 0.5 * s_vals_global_path[-1]:
            # state_3 = state_3_reset
            # s_0_dyn_obst = s_0_dyn_obst_reset
            s_0_dyn_obst = s + s_0_dyn_obst_reset  # reset in front of the ego vehicle
            s_dot_dyn_obst = s_dot_dyn_obst_selection[np.mod(reset_count, len(s_dot_dyn_obst_selection))]
            reset_count = reset_count + 1
            print('resetting to initial value, V dyn =', s_dot_dyn_obst)




        #fill in the messages and publish them
        message_1.header.stamp = rospy.Time.now()
        message_1.pose.position.x = 0.0
        message_1.pose.position.y = 0.8
        message_1.pose.orientation.y = 3.14 * (-1/8)

        message_2.header.stamp = rospy.Time.now()
        message_2.pose.position.x = -1.5
        message_2.pose.position.y = 0.8
        message_2.pose.orientation.y = 3.14 * 1

        message_3.header.stamp = rospy.Time.now()
        message_3.pose.position.x = state_3[0]
        message_3.pose.position.y = state_3[1]
        message_3.pose.orientation.y = state_3[2]

        message_4.header.stamp = rospy.Time.now()
        message_4.pose.position.x=0.0
        message_4.pose.position.y=0.8
        message_4.pose.orientation.y= 3.14 * (-1/8)

        #velocity
        velocity_message_1 = 1
        velocity_message_2 = 1
        velocity_message_3 = state_3[3]
        velocity_message_4 = 1


        #publish the messages
        test_publisher_1.publish(message_1)
        test_publisher_2.publish(message_2)
        test_publisher_3.publish(message_3)
        test_publisher_4.publish(message_4)

        test_velocity_publisher_1.publish(velocity_message_1)
        test_velocity_publisher_2.publish(velocity_message_2)
        test_velocity_publisher_3.publish(velocity_message_3)
        test_velocity_publisher_4.publish(velocity_message_4)

        occupancy_xyr_4_publisher.publish(dyn_ob_message)

        rate.sleep()

if __name__ == '__main__':
    try:
        forwards_integrate()
    except rospy.ROSInterruptException:
        pass
