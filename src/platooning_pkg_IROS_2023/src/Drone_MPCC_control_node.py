#!/usr/bin/env python3

# MPCC Control Node
#This node
# 1. It is subscribing to the following topics:
#   - vx, vy, optitrack_state, safety_value
# 2. It is publishing the following topics:
#   - throttle_x, throttle_y, comptime

# The node is doing the following:
# 1. It is computing the control inputs for the drone
# 2. It is publishing the control inputs to the drone
# 3. It is publishing the computation time of the solver


import numpy as np
from pathlib import Path
from scipy import interpolate
from scipy.spatial.transform import Rotation
import rospy
import os
from pathlib import Path
from functions_for_Drone_MPCC_node_running import find_s_of_closest_point_on_global_path,\
                                            evaluate_local_path_Chebyshev_coefficients_high_order_cheby,\
                                            produce_s_local_path,\
                                            produce_track

from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from datetime import datetime
import csv
import forcespro.nlp
from helper_functions.utility_functions import model_attributes
from helper_functions.functions_for_drone_solver_generation import Functions_for_solver_generation

# for dynamic paramters reconfigure (setting param values from rqt_reconfigure GUI)
from dynamic_reconfigure.server import Server
from dynamic_reconfigure_pkg.cfg import Drone_MPCC_dynamic_reconfigureConfig
import rospkg
from custom_msgs_optitrack.msg import pointmass_opti_pose_stamped_msg



class MPCC_controller_class(Functions_for_solver_generation):
    def __init__(self, vehicle_number,dt):

        # set up variables
        self.vehicle_number = vehicle_number
        self.dt = dt
        self.solver_choice = 2  # default choice

        # to handle warmstarting 
        self.first_run = True 

        # initialize the state obtained by optitrack (x,y,z,vx,vy,vz)
        self.opti_state = [-6, -4, 0, 0, 0, 0]

        # define selected solver
        self.set_solver_type()

        # set up constant problem parameters 
        self.setup_constant_parameters()

        # initialize path relative variable necessary for local search of closest point on path
        self.previous_index = 1

        # define rviz related topics
        self.set_up_topics_for_rviz()

        # for data time stamp initialize sensor data
        self.start_elapsed_time = rospy.get_rostime()
        self.safety_value = 0
        self.data_folder_name = 'Data_recordings'
        self.file = 0
        self.writer = 0
        self.start_elapsed_time = 0
        self.setup_data_recording()

        ## SELECT THE TRACK ##
        # z-Sinusoidal Tracks
        #track_choice = 'savoiardo'
        #track_choice = 'double_donut'
        #track_choice = 'savoiardo_saturate_steering'
        #track_choice = 'circle'
        #track_choice = 'racetrack_saturate_steering'
        #track_choice = 'gain_sweep_track'
        #track_choice = 'racetrack_Lab'

        # Spline tracks
        #track_choice = 'spline_circle'
        track_choice = 'spline_race_track'

        # Loop of the path
        self.loop_path = True

        # produce track related fixed quantities
        self.generate_track(track_choice)

        # set publisher
        self.throttle_x_publisher = rospy.Publisher('throttle_x_' + str(vehicle_number), Float32, queue_size=1)
        self.throttle_y_publisher = rospy.Publisher('throttle_y_' + str(vehicle_number), Float32, queue_size=1)
        self.throttle_z_publisher = rospy.Publisher('throttle_z_' + str(vehicle_number), Float32, queue_size=1)
        self.comptime_publisher = rospy.Publisher('comptime_' + str(vehicle_number), Float32, queue_size=1)

        # set subscriber for utility variables
        self.optitrack_state_subscriber = rospy.Subscriber('Optitrack_data_topic_' + str(vehicle_number), pointmass_opti_pose_stamped_msg, self.opti_state_subscriber_callback)
        self.safety_value_subscriber = rospy.Subscriber('safety_value', Float32, self.safety_value_subscriber_callback)

        #for dynamic parameter change using rqt_reconfigure GUI
        srv = Server(Drone_MPCC_dynamic_reconfigureConfig, self.reconfig_callback)

        # produce and send out global path message to rviz, which contains information about the track (i.e. the global path)
        rgba = [160, 189, 212, 0.35]
        marker_type = 4
        self.global_path_message = self.produce_marker_array_rviz(self.x_vals_global_path, self.y_vals_global_path, self.z_vals_global_path, rgba, marker_type)
        self.rviz_global_path_publisher.publish(self.global_path_message)

        # produce and send out lane boundaries message to rviz
        rgba = [100.0, 255.0, 100.0, 0.15]
        # steps (i.e. how often the lane boundaries are plotted)
        step = 5
        rviz_global_lane_bound_message = self.produce_and_publish_rviz_lane_boundaries(self.s_vals_global_path, rgba, step)
        self.rviz_global_lane_bound_publisher.publish(rviz_global_lane_bound_message)

        # produce and send out gates message to rviz
        rgba = [255.0, 0.0, 0.0, 0.35]
        rviz_gates_message = self.produce_and_publish_rviz_gates(rgba)
        self.rviz_gates_publisher.publish(rviz_gates_message)


    ### in the following section are present all the callback functions for the subscribed topics mentioned in __init__ of MPC_Controller_class ###

    def opti_state_subscriber_callback(self, msg):
        self.opti_state = [msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz]



    def safety_value_subscriber_callback(self, msg):
        self.safety_value = msg.data



    # callback to update the parameters modified dynamically by means of the gui
    def reconfig_callback(self, config, level):
        print('_________________________________________________')
        print('  reconfiguring parameters from dynamic_reconfig ')
        print('‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾')

        self.V_target = config['V_target__s_dot_target']
        self.q1 = config['q1']
        self.q2 = config['q2']
        self.q3 = config['q3']
        self.q4 = config['q4']
        self.l_width = config['l_width']
        self.warm_start = config['warm_start']
        #self.dt = config['dt']
        self.solver_choice = config['solver_choice']
        self.save_data = config['save_data']
        self.data_folder_name = config['data_folder_name']
        self.minimal_plotting = config['minimal_plotting']

        if self.save_data == False:
            try:
                #close file when recording is switched off
                self.file.close()
            except:
                #print('')
                pass

        # assemble parameters in order for changes to take effect on parameter_block 1 and 2
        self.set_solver_type()
        self.assemble_fixed_parameters()
        #self.rate = rospy.Rate(1 / self.dt)

        # re-send the global path message
        rgba = [160, 189, 212, 0.35]
        marker_type = 4
        self.global_path_message = self.produce_marker_array_rviz(self.x_vals_global_path, self.y_vals_global_path, self.z_vals_global_path, rgba, marker_type)
        self.rviz_global_path_publisher.publish(self.global_path_message)

        # re-send the lane boundaries message
        rgba = [100.0, 255.0, 100.0, 0.15]
        # steps (i.e. how often the lane boundaries are plotted)
        step = 5
        rviz_global_lane_bound_message = self.produce_and_publish_rviz_lane_boundaries(self.s_vals_global_path, rgba, step)
        self.rviz_global_lane_bound_publisher.publish(rviz_global_lane_bound_message)

        # re-send the gates message
        rgba = [255.0, 0.0, 0.0, 0.35]
        rviz_gates_message = self.produce_and_publish_rviz_gates(rgba)
        self.rviz_gates_publisher.publish(rviz_gates_message)


        print('‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾')
        return config
    
    ### end of the callbacks section



    ### in the following section are present all the functions called by def__init__ of MPC_Controller_class ###

    def set_solver_type(self):
        print('Setting solver type')

        self.model_attributes_obj = model_attributes(self.solver_choice)
        print('Solver choice: ', self.model_attributes_obj.solver_name)
        # get the right solver from the solver folder
        rospack = rospkg.RosPack()
        gp_pkg_path = rospack.get_path('mpcc_pkg')
        path_to_solver_folder = os.path.join(Path(gp_pkg_path).parents[2],'MPCC_generate_solvers/Solvers')
        path_to_solver = os.path.join(path_to_solver_folder,self.model_attributes_obj.solver_name)
        if os.path.isdir(path_to_solver) == False:
            print('+++++++++++++++++++++++++++++++++++++++++++++++++')
            print('Warning! the provided solver location is invalid')
            print('+++++++++++++++++++++++++++++++++++++++++++++++++')
        
        self.solver = forcespro.nlp.Solver.from_directory(path_to_solver)



    def setup_constant_parameters(self):
        # set up N (prediction horizon)
        self.N = 30             # must match the number of stages in the solver

        # Solver related
        self.warm_start = False

        # initial solution is zero [ th_x, th_y, th_z, slack, x, y, z, vx, vy, vz, s]
        warm_start_solution_0 = np.concatenate(([0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]), axis=0)

        # initialize previous solution and warm start solution both as a vector of N times (prediction horizon) the initial solution
        self.previous_solution = np.tile(warm_start_solution_0, self.N)
        self.warm_start_solution = np.tile(warm_start_solution_0, self.N)

        # generate parameters 
        self.V_target = 1
        self.l_width = 0.6      # width of lane
        self.slack_p = 1000     # controls the cost of the slack variable

        # J = self.q1 * (V_vehicle - V_target) ** 2 + self.q2 * err_cont_squared + self.q3 * err_lag_squared + self.q4 * u[0] ** 2 + self.q4 * u[1] ** 2 + self.q4 * u[2]
        self.q1 = 1             # relative weight of s_dot following
        self.q2 = 10            # relative weight of conturing error
        self.q3 = 1             # relative weight of lag error
        self.q4 = 1             # relative weight of inputs  (weighted the same here)

        # assemble fixed parameters
        self.assemble_fixed_parameters()



    # solve the LQR problem to define the terminal cost and assemble the fixed parameters
    def assemble_fixed_parameters(self):
        # solve LQR problem to define the terminal cost (if V_target changes put this inside the while loop)

        m = 1.63                # mass of the point
        fri = 3                 # friction

        # define A and B matrices
        # A = np.array([[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1], [0, 0, 0, -fri/m, 0, 0], [0, 0, 0, 0, -fri/m, 0], [0, 0, 0, 0, 0, -fri/m]])
        # B = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [1/m, 0, 0], [0, 1/m, 0], [0, 0, 1/m]])

        # define Q and R matrices, LQR problem is: min integral( x'Qx + u'Ru )
        # Q = np.identity(A.shape[0])
        # R = np.identity(B.shape[1])

        # solve the LQR problem
        #K, S, E = control.lqr(A, B, 2 * Q, 2 * R)
        #S = S

        #self.fixed_params_block_1 = [self.V_target, self.L, C, a_th, b_th, self.dt, self.xo, self.yo, self.ro]
        #self.fixed_params_block_2 = [self.l_width, self.q1, self.q2, self.q3, self.q4,
        #                        S[0, 0], S[0, 1], S[0, 2], S[1, 0], S[1, 1], S[1, 2], S[2, 0], S[2, 1], S[2, 2],
        #                        self.slack_p]

        self.fixed_params = [self.V_target, self.dt, self.l_width, self.q1, self.q2, self.q3, self.q4,
                                     self.slack_p ]
        


    def set_up_topics_for_rviz(self):
        self.rviz_MPC_path_publisher = rospy.Publisher('rviz_MPC_path_' + str(self.vehicle_number), MarkerArray, queue_size=10)
        self.rviz_global_path_publisher = rospy.Publisher('rviz_global_path_' + str(self.vehicle_number), MarkerArray, queue_size=10)
        self.rviz_local_path_publisher = rospy.Publisher('rviz_local_path_' + str(self.vehicle_number), MarkerArray, queue_size=10)
        self.rviz_global_lane_bound_publisher = rospy.Publisher('rviz_global_lane_bound_' + str(self.vehicle_number), MarkerArray, queue_size=10)
        self.rviz_local_lane_bound_publisher = rospy.Publisher('rviz_local_lane_bound_' + str(self.vehicle_number), MarkerArray, queue_size=10)
        self.rviz_curvature_centre_publisher = rospy.Publisher('rviz_curvature_centre_' + str(self.vehicle_number), MarkerArray, queue_size=10)
        self.rviz_gates_publisher = rospy.Publisher('rviz_gates_' + str(self.vehicle_number), MarkerArray, queue_size=10)

        # MPC path is the path followed by the vehicle in the open loop prediction (prediction horizon)
        # global path is the overall path structure (i.e. the track)
        # local path is the path evaluated by the chebychev polynomials of extremes [a,b], is the portion of path plotted by rviz
        # global lane bound are the lane boundaries plotted for all the path
        # local lane bound are the lane boundaries plotted for the local path and seen by the forces pro solver (i.e. the ones used for the constraints) 
        # curvature centre is the centre of the circle that best fits the local path



    # define the settings for the data file which will store the data
    def setup_data_recording(self):
        date_time = datetime.now()
        date_time_str = date_time.strftime("%m_%d_%Y_%H_%M_%S")
        rospack = rospkg.RosPack()
        mpcc_pkg_path = rospack.get_path('mpcc_pkg')
        print(mpcc_pkg_path)
        file_name = mpcc_pkg_path + '/src/' + self.data_folder_name + '/recording_' + date_time_str + '.csv'
        file = open(file_name, 'w')
        writer = csv.writer(file)
        start_elapsed_time = rospy.get_rostime()
        self.file = file
        self. writer = writer
        self.start_elapsed_time = start_elapsed_time
        return



    # according to the track choice it generates the track related fixed quantities
    def generate_track(self, track_choice):
        # choice = 'savoiardo'
        # choice = 'double_donut'
        # choice = 'straight_line'
        # choice = 'savoiardo_saturate_steering'
        # choice = 'circle'
        # choice = 'racetrack_saturate_steering'
        # choice = 'racetrack_Lab'
        # choice = 'gain_sweep_track_2'
        
        # number of checkpoints to be used to define each spline of the track
        n_checkpoints = 1000

        # cartesian coordinates (x-y-z) of the points defining the track and the gates
        Checkpoints_x, Checkpoints_y, Checkpoints_z, gates_coordinates = produce_track(track_choice, n_checkpoints)

        # associate these points to the global path x-y-z variables
        self.x_vals_global_path = Checkpoints_x
        self.y_vals_global_path = Checkpoints_y
        self.z_vals_global_path = Checkpoints_z

        # from the x-y-z points obtain the sum of the arc lenghts between each point, i.e. the path seen as a function of s
        spline_discretization = len(Checkpoints_x)
        self.s_vals_global_path = np.zeros(spline_discretization)
        for i in range(1,spline_discretization):
            self.s_vals_global_path[i] = self.s_vals_global_path[i-1] + np.sqrt((self.x_vals_global_path[i]-self.x_vals_global_path[i-1])**2+
                                                                                (self.y_vals_global_path[i]-self.y_vals_global_path[i-1])**2+
                                                                                (self.z_vals_global_path[i]-self.z_vals_global_path[i-1])**2)
        # s_vals_global_path = sum of the arc lenght along the original path, which are gonna be used to re-parametrize the path

        # generate splines x(s) and y(s) where s is now the arc length value (starting from 0)
        self.x_of_s = interpolate.CubicSpline(self.s_vals_global_path, self.x_vals_global_path)
        self.y_of_s = interpolate.CubicSpline(self.s_vals_global_path, self.y_vals_global_path)
        self.z_of_s = interpolate.CubicSpline(self.s_vals_global_path, self.z_vals_global_path)

        # find the s value of the gates on the path starting from the x-y-z coordinates of the gates
        self.gates_s = self.find_s_gate_position(gates_coordinates) 



    # find the s value of the gates on the path starting from the x-y-z coordinates of the gates
    def find_s_gate_position(self, gates_xyz):
        # Initialize the gates_s vector
        gates_s = np.zeros(len(gates_xyz)) 
        
        # retrieve the x(s), y(s), z(s) in array shape (rather than cubic spline shape)
        x_s = self.x_of_s(self.s_vals_global_path)
        y_s = self.y_of_s(self.s_vals_global_path)
        z_s = self.z_of_s(self.s_vals_global_path)

        # evauate the x-y-z coordinates according to every s value of the global path and check if they match with the x-y-z coordinates of the gates
        for ii in range(len(self.s_vals_global_path)):
            # for every point of the path check every gate
            for jj in range(len(gates_xyz)):
                # if the coordinates match, store the s value of the path in the proper position of gates_s
                if (x_s[ii] == gates_xyz[jj,0] and y_s[ii] == gates_xyz[jj,1] and z_s[ii] == gates_xyz[jj,2]):
                    gates_s[jj] = self.s_vals_global_path[ii]

        return gates_s



    # produce the marker_array to visualize things in rviz (global path, local path, predictions)
    def produce_marker_array_rviz(self, x, y, z, rgba, marker_type):
        marker_array = MarkerArray()              # definition of an array of markers
        marker = Marker()                         # single marker within the marker_array

        marker.header.frame_id = "map"            # map frame, used when markers or data should be positioned in relation to a global map.
        marker.header.stamp = rospy.Time.now()    # associate a timestamp to the frame

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3 ; Line_strip: 4
        marker.type = marker_type
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025

        # Set the color
        marker.color.r = rgba[0] / 256
        marker.color.g = rgba[1] / 256
        marker.color.b = rgba[2] / 256
        marker.color.a = rgba[3]

        # Set the pose of the marker
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        points_list = []
        for i in range(len(x)):
            p = Point()
            p.x = x[i]
            p.y = y[i]
            p.z = z[i]
            points_list = points_list + [p]

        marker.points = points_list

        # append the created marker to marker_array
        marker_array.markers.append(marker)

        return  marker_array



    # prepare and publish the lane boundaries and for the rviz visualization
    def produce_and_publish_rviz_lane_boundaries(self, s_vals_path, rgba, step = 5):

        # defining the points for the boundary check
        length = len(s_vals_path)                            # n points that make the global path
        x_lane, y_lane, z_lane, vec_quat = [], [], [], []    # initialize the lane vectors

        # fill the lane vectors
        for i in range(0,length,step):
            if (i*step < length):
                actual_s = s_vals_path[i*step]
                actual_x, actual_y, actual_z, tan, normal = self.compute_tangent_and_normal(actual_s)

                # compute the quaternion for the pose
                quaternion = self.compute_quaternion(tan,normal)

                x_lane.append(actual_x)
                y_lane.append(actual_y)
                z_lane.append(actual_z)
                vec_quat.append(quaternion)

        ## FOR RVIZ ##

        # marker type (circle)
        marker_type = 3

        # produce marker array for the lane boundaries and publish it
        rviz_lane_bound_message = self.produce_tridimensional_lanes(x_lane, y_lane, z_lane, vec_quat, rgba, marker_type, self.l_width)

        return rviz_lane_bound_message



    # we need to retrieve the pose of the circle, in order to do so we have to define the tangent vector and the normal vector for each point
    def compute_tangent_and_normal(self, actual_s):
        # need to predefine the forecast here, if you change Cheby_data_points be aware to change it also inside produce_chebyshev_coeffs
        local_Ds_forecast = 1 * 0.1 * 30 * 1.1
        local_Cheby_data_points = 100
        # we need to produce the coefficients related to the path section in which we are
        coeffx, coeffy, coeffz, a, b = evaluate_local_path_Chebyshev_coefficients_high_order_cheby(actual_s, self.x_of_s, self.y_of_s, self.z_of_s, local_Ds_forecast,
                                                                                    self.s_vals_global_path, self.loop_path,
                                                                                    local_Cheby_data_points)
        
        # compute the vector of parameters
        param = np.array([a, b, *coeffx, *coeffy, *coeffz, *self.fixed_params])

        # compute the coordinates, their first derivative (useful for the tangent), their second derivative (useful for the normal), since we don't need the s_dot we can place 1 instead of the velocities
        x_Cout, y_Cout, z_Cout, s_dot, k, x_Cdev, y_Cdev, z_Cdev, x_Cdev2, y_Cdev2, z_Cdev2 = self.evaluate_spline_quantities(0, 0, 0, 0, 0, 0, actual_s, param)

        # retrieve the position
        actual_x = x_Cout
        actual_y = y_Cout
        actual_z = z_Cout    

        # compute the tangent
        tan = [x_Cdev, y_Cdev, z_Cdev] / np.linalg.norm([x_Cdev, y_Cdev, z_Cdev])

        # compute the normal
        normal = [x_Cdev2, y_Cdev2, z_Cdev2] / np.linalg.norm([x_Cdev2, y_Cdev2, z_Cdev2])

        return actual_x, actual_y, actual_z, tan, normal



    # by taking the tangent and the normal retrieve the quaternion useful for the pose
    def compute_quaternion(self, tan, normal):
        
        # compute the binormal
        bin = np.cross(tan, normal)

        # create a rotation matrix from the tangent, normal and binormal
        rotation_matrix = np.column_stack((tan, normal, bin))

        # to have the cylinder oriented in the useful direction we should rotate on y axis by 90°
        rotation_matrix = np.dot(rotation_matrix, np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]))

        # convert the rotation matrix to a rotation object for easier manipulation and convert in quaternion
        quaternion = Rotation.from_matrix(rotation_matrix).as_quat()

        return quaternion



    # takes the vectors of positions and orientations for the lane boundaries markers and produces the relative marker object
    def produce_tridimensional_lanes(self, x, y, z, quat, rgba, marker_type, width, thickness = 0.0001):
        marker_array = MarkerArray()                  # definition of an array of markers

        for i in range(0,len(x)):
            marker = Marker()                         # single marker within the marker_array
            marker.header.frame_id = "map"            # map frame, used when markers or data should be positioned in relation to a global map.
            marker.header.stamp = rospy.Time.now()    # associate a timestamp to the frame

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3 ; Line_strip: 4
            marker.type = marker_type
            marker.id = i

            # Set the scale of the marker
            marker.scale.x = width
            marker.scale.y = width
            marker.scale.z = thickness                # we want a circle, so z is very small

            # Set the color
            marker.color.r = rgba[0] / 256
            marker.color.g = rgba[1] / 256
            marker.color.b = rgba[2] / 256
            marker.color.a = rgba[3]

            # Set the pose of the marker
            marker.pose.position.x = x[i]
            marker.pose.position.y = y[i]
            marker.pose.position.z = z[i]
            marker.pose.orientation.x = quat[i][0]
            marker.pose.orientation.y = quat[i][1]
            marker.pose.orientation.z = quat[i][2]
            marker.pose.orientation.w = quat[i][3]

            # append the created marker to marker_array
            marker_array.markers.append(marker)

        return  marker_array
    


    # prepare and publish the gates for the rviz visualization
    def produce_and_publish_rviz_gates(self, rgba):

        # defining the points for the boundary check
        length = len(self.gates_s)                            
        x_gate, y_gate, z_gate, vec_quat = [], [], [], []    # initialize the lane vectors

        # fill the lane vectors
        for i in range(0,length):
            actual_s = self.gates_s[i]
            actual_x, actual_y, actual_z, tan, normal = self.compute_tangent_and_normal(actual_s)

            # compute the quaternion for the pose
            quaternion = self.compute_quaternion(tan,normal)

            x_gate.append(actual_x)
            y_gate.append(actual_y)
            z_gate.append(actual_z)
            vec_quat.append(quaternion)


        ## FOR RVIZ ##
        # marker type (circle)
        marker_type = 3

        # dimensions of the gates
        width = 0.8
        thickness = 0.05

        # produce marker array for the lane boundaries and publish it
        rviz_gate_message = self.produce_tridimensional_lanes(x_gate, y_gate, z_gate, vec_quat, rgba, marker_type, width, thickness)

        return rviz_gate_message
    
    ### end of init function's section ###



    # The MPCC control loop (this is one iteration only so this need to be called multiple times in a while loop)
    def MPCC_control_loop(self):

        # set the frequency rate of the loop equal to 1/dt when the function is called (i.e. the loop is executed 1/dt times per second)

        # get clock estimated for timing purpouses
        start_clock_time = rospy.get_rostime()

        # updates the local path at runtime. Dyn obst is updated by the subscriber callback
        a, b, coeffx, coeffy, coeffz, self.s = self.produce_chebyshev_coeffs() # this automatically reads the current state and global path parameters

        # assemble all parameters
        params_i = np.array([a, b, *coeffx, *coeffy, *coeffz, *self.fixed_params])

        # give as a stacked vector of parameters (see codeoptions.nlp.stack_parambounds in solver generation phase)
        all_params_array = np.tile(params_i, self.N) 

        problem = self.produce_problem_for_forces(all_params_array)
        
        # call solver
        try:
            output, exitflag, info = self.solver.solve(problem)
        except:
            print('Failure in calling the solver, trying to reset the solver choice')
            rospy.sleep(1) # wait this amount of time so that the reconfig callback can finish executing

            # choose the solver again and solve
            self.set_solver_type()
            params_i = np.array([a, b, *coeffx, *coeffy, *coeffz, *self.fixed_params])
            all_params_array = np.tile(params_i, self.N)
            problem = self.produce_problem_for_forces(all_params_array)
            output, exitflag, info = self.solver.solve(problem)

        
        start_clock_time_after_solver = rospy.get_rostime()

        # retrieve the output values
        output_array = np.array(list(output.values()))

        # check if solver converged
        if exitflag != 1:
            print('Ouch solver did not converge exitflag = ', exitflag)
        else:  
            pass
            # converged, so update warm start (it will only be used if warm_start = true)
            # always update warm start solution, then if warm_start is True it will be used, if not it wont be used

            # take first solution of the MPC as a starting point to compute the vector to be given to the warm start
            self.previous_solution = output_array[0]

            # concatenate in previous_solution for the warm start the whole solution obtained by using a for loop
            for cc in range(1, self.N):
                self.previous_solution = np.concatenate((self.previous_solution, output_array[cc]))

        # publish the control inputs
        self.publish_control_inputs(output_array)

        if self.minimal_plotting == False:
            # prepare stuff for rviz
            self.produce_and_publish_rviz_visualization(output_array, self.Cheby_data_points, params_i,
                                                        self.s, self.Ds_forecast, self.s_vals_global_path,
                                                        self.loop_path, self.x_vals_global_path, self.y_vals_global_path, self.z_vals_global_path)

        # evaluating elapsed time (considering both seconds and nanoseconds for a more precise measurement)
        stop_clock_time = rospy.get_rostime()
        total_time = stop_clock_time.secs - start_clock_time.secs + (
                    stop_clock_time.nsecs - start_clock_time.nsecs) / 1000000000
        self.comptime_publisher.publish(total_time)


        # store data if necessary
        if self.save_data == True:
            try:
                # write new row
                self.write_new_data_row(total_time, info.solvetime, params_i, output_array)
            except:
                # if exception rises it means that there's no data file in which to write, so it defines it
                print('creating new data file')
                self.setup_data_recording()
                self.writer.writerow(['s global path', 'x global path', 'y global path', 'z global path'])
                self.writer.writerow([self.s_vals_global_path , self.x_vals_global_path, self.y_vals_global_path, self.z_vals_global_path])
                self.writer.writerow(
                    ['elapsed time', 'opti x', 'opti y', 'opti z', 'opti vx', 'opti vy', 'opti vz', 
                     'safety_value', 'throttle_x', 'throttle_y', 'throttle_z', 'total comp time', 'solver comp time', 'params_i', 'solver solution'])
                self.write_new_data_row(total_time, info.solvetime, params_i, output_array)

            



    ### in the following section are present all the functions called by MPCC_control_loop of MPC_Controller_class ###

    # Produces chebyshev coefficients and returns a,b,coeffx,coeffy (cheby params) and s (parameter of the curve with minimum distance between the vehicle and the path)
    def produce_chebyshev_coeffs(self):
        # initial guess for the estimated travelled distance along the path
        estimated_ds = 0.5 # 0.5 meters

        # measure the closest point on the global path, retturning the respective s parameter and its index
        s, self.current_index = find_s_of_closest_point_on_global_path(np.array([self.opti_state[0], self.opti_state[1], self.opti_state[2]]), self.s_vals_global_path,
                                                                  self.x_vals_global_path, self.y_vals_global_path, self.z_vals_global_path,
                                                                  self.previous_index, estimated_ds)
        
        # update index along the path to know where to search in next iteration
        self.previous_index = self.current_index  

        # evaluate local path
        # forward path to be visualized in rviz, derived considering a little more (1.1) than the possible travelled distance along all the prediction horizon (N)  
        self.Ds_forecast = self.V_target * self.dt * self.N * 1.1
        self.Cheby_data_points = 100 

        # a and b are the extremes of path parameter values
        coeffx, coeffy, coeffz, a, b = evaluate_local_path_Chebyshev_coefficients_high_order_cheby(s, self.x_of_s, self.y_of_s, self.z_of_s, self.Ds_forecast,
                                                                                     self.s_vals_global_path, self.loop_path,
                                                                                     self.Cheby_data_points)
        return a, b, coeffx, coeffy, coeffz, s


    # produce the problem to be solved by forces pro
    def produce_problem_for_forces(self, all_params_array):
        # define initial condition for the solver
        # x = x y z vx vy vz s
        xinit = np.array([self.opti_state[0], self.opti_state[1], self.opti_state[2], self.opti_state[3], self.opti_state[4], self.opti_state[5], self.s])
            
        if self.warm_start and self.first_run==False:
            # produce the problem as a dictionary for the solver
            problem = {"reinitialize": True, "xinit": xinit, "all_parameters": all_params_array}
        else:
            # produce first guess as if perfectly on path
            s_0 = self.s + self.V_target * np.array(range(self.N)) * self.dt
            s_0_path_gen = self.s + self.V_target * np.array(range(self.N)) * self.dt

            # if loop path is true, when the vehicle reaches the end of the path it starts again from the beginning
            if self.loop_path:
                s_0_path_gen[s_0_path_gen > self.s_vals_global_path.max()] = s_0_path_gen[s_0_path_gen > self.s_vals_global_path.max()] - self.s_vals_global_path.max()
                s_0_path_gen[s_0_path_gen < self.s_vals_global_path.min()] = self.s_vals_global_path.max() + s_0_path_gen[s_0_path_gen < self.s_vals_global_path.min()] 

            # initial x(s), y(s), z(s) values (actual + predictions over N)
            pos_x_0 = self.x_of_s(s_0_path_gen)
            pos_y_0 = self.y_of_s(s_0_path_gen)
            pos_z_0 = self.z_of_s(s_0_path_gen)

            # vectors of velocities (actual + prediction over N)
            vx_0 = np.diff(pos_x_0)/self.dt
            vx_0 = [self.opti_state[3], *vx_0] # add first vx as current state
            vy_0 = np.diff(pos_y_0)/self.dt
            vy_0 = [self.opti_state[4], *vy_0] # add first vy as current state
            vz_0 = np.diff(pos_z_0)/self.dt
            vz_0 = [self.opti_state[5], *vz_0] # add first vz as current state

            z_length = self.model_attributes_obj.n_inputs + self.model_attributes_obj.n_states
            x0_array = np.zeros(z_length*self.N)

            # initialize the first guess as zero control action and static state (this does violate dynamic constraint)
            for kk in range(self.N):
                x0_array[z_length * kk:z_length * (kk+1)] = [*np.zeros(self.model_attributes_obj.n_inputs), pos_x_0[kk], pos_y_0[kk], pos_z_0[kk], vx_0[kk], vy_0[kk], vz_0[kk], s_0[kk]]

            # produce the problem as a dictionary for the solver
            problem = {"x0": x0_array, "xinit": xinit, "all_parameters": all_params_array}

            # disable first_run variable (needed for warm starting)
            self.first_run==False

        #retrieve a, b values (extremes of the local path parameters)
        a = all_params_array[0]
        b = all_params_array[1]

        # lower and upper bound for the optimization problem at runtime (a,b needs to be updated)
        # th_x, th_y, th_z, slack, s
        problem["lb"] = np.tile([-10, -10, -10, 0, a], self.N)    # tile repeats the values N times
        problem["ub"] = np.tile([ 10, 10, 10, 100, b], self.N)

        return problem



    def publish_control_inputs(self, output_array):

        # retrieve from output_array throttle values for the first step of the prediction horizon
        throttle_x_val = Float32(output_array[0, 0])
        throttle_y_val = Float32(output_array[0, 1])
        throttle_z_val = Float32(output_array[0, 2])
        # slack = Float32(output_array[0, 3])

        # for data storage purpouses
        self.throttle_x = throttle_x_val.data
        self.throttle_y = throttle_y_val.data
        self.throttle_z = throttle_z_val.data

        # publish the three throttles
        self.throttle_x_publisher.publish(throttle_x_val)
        self.throttle_y_publisher.publish(throttle_y_val)
        self.throttle_z_publisher.publish(throttle_z_val)



    # prepare and publish predictions made by the solver, local path, vehicle, lane boundaries and curvature center for the rviz visualization
    def produce_and_publish_rviz_visualization(self, output_array, Cheby_data_points, params_i, s, Ds_forecast, s_vals_global_path, loop_path,
                            x_vals_global_path, y_vals_global_path, z_vals_global_path):

        # extract x,y,z predictions along the prediction horizon
        x_vec = output_array[:, 4]
        y_vec = output_array[:, 5]
        z_vec = output_array[:, 6]
        # s_vec = output_array[:, 7]
        
        # evaluating local path as solver will do it (inside continuous dynamics)
        # x = [x y z vx vy vz s]
        dummy_x = [0, 0, 0, 0, 0, 0, 0]
        x_path_from_solver_function = np.zeros(Cheby_data_points)
        y_path_from_solver_function = np.zeros(Cheby_data_points)
        z_path_from_solver_function = np.zeros(Cheby_data_points)

        s_subpath_cheby_for_xyz_data_generation, s_subpath_for_fitting_operation = produce_s_local_path(s, Ds_forecast,
                                                                                                       Cheby_data_points,
                                                                                                       s_vals_global_path,
                                                                                                       loop_path)

        for ii in range(0, Cheby_data_points):
            # associate to the s field of dummy_x the values found by produce_s_local_path
            dummy_x[6] = s_subpath_for_fitting_operation[ii]

            x_Cout, y_Cout, z_Cout, s_dot, k, x_Cdev, y_Cdev, z_Cdev,  x_Cdev2, y_Cdev2, z_Cdev2 = self.evaluate_spline_quantities(dummy_x[0], dummy_x[1], dummy_x[2], dummy_x[3], dummy_x[4], dummy_x[5], dummy_x[6], params_i)

            # x(s),y(s), z(s) coordinates of the predictions made by the MPC
            x_path_from_solver_function[ii] = x_Cout
            y_path_from_solver_function[ii] = y_Cout
            z_path_from_solver_function[ii] = z_Cout


        # adding centre of curvature to make sure the path is stable
        x_centre_of_curvature_by_solver = np.zeros(self.N)
        y_centre_of_curvature_by_solver = np.zeros(self.N)
        z_centre_of_curvature_by_solver = np.zeros(self.N)

        for ii in range(0, self.N):
            # state_i = [x y z vx vy vz s]
            state_i = output_array[ii, 4:11]
            
            x_Cout, y_Cout, z_Cout, s_dot, k, x_Cdev, y_Cdev, z_Cdev, x_Cdev2, y_Cdev2, z_Cdev2 = self.evaluate_spline_quantities(state_i[0],state_i[1],state_i[2],state_i[3],state_i[4], state_i[5], state_i[6], params_i)

            x_centre_of_curvature_by_solver[ii] = x_Cout + (1/k**2) * x_Cdev2
            y_centre_of_curvature_by_solver[ii] = y_Cout + (1/k**2) * y_Cdev2
            z_centre_of_curvature_by_solver[ii] = z_Cout + (1/k**2) * z_Cdev2


        ## FOR RVIZ ##

        # open loop prediction mean #
        # set colors for the predictions
        rgba = [0, 166, 214, 1.0]
        marker_type = 4

        # produce marker for MPCC path prediction and publish it
        rviz_MPCC_path_message = self.produce_marker_array_rviz(x_vec, y_vec, z_vec, rgba, marker_type)
        self.rviz_MPC_path_publisher.publish(rviz_MPCC_path_message)


        # local path #
        # set colors for the local path
        rgba = [250, 150, 100, 0.5]

        # produce marker for local path and publish it
        rviz_local_path_message = self.produce_marker_array_rviz(x_path_from_solver_function, y_path_from_solver_function, z_path_from_solver_function, rgba, marker_type)
        self.rviz_local_path_publisher.publish(rviz_local_path_message)


        # lanes #
        # set colors for the lanes
        rgba = [250, 150, 100, 0.15]

        # steps (i.e. how often the lane boundaries are plotted)
        step = 2

        # produce and send out local lane boundaries message to rviz
        rviz_local_lane_bound_message = self.produce_and_publish_rviz_lane_boundaries(s_subpath_for_fitting_operation, rgba, step)
        self.rviz_local_lane_bound_publisher.publish(rviz_local_lane_bound_message)

        # centre of curvature #
        # publish centre of curvature
        rgba = [208.0, 78.0, 199.0, 0.5]
        marker_type = 4
        rviz_curvature_centre_publisher_message = self.produce_marker_array_rviz(x_centre_of_curvature_by_solver, y_centre_of_curvature_by_solver, z_centre_of_curvature_by_solver, rgba, marker_type)
        self.rviz_curvature_centre_publisher.publish(rviz_curvature_centre_publisher_message)



    # defines a new data line and places in it the actual data to store them
    def write_new_data_row(self, total_time, solvetime,  params_i, output_array):
        # updating elapsed time
        stop_clock_time = rospy.get_rostime()
        elapsed_time = stop_clock_time.secs - self.start_elapsed_time.secs + (
                stop_clock_time.nsecs - self.start_elapsed_time.nsecs) / 1000000000
        
        #filling the new data line
        data_line = [elapsed_time, self.opti_state[0], self.opti_state[1], self.opti_state[2], self.opti_state[3], self.opti_state[4], self.opti_state[5],
                     self.safety_value, self.throttle_x, self.throttle_y, self.throttle_z, total_time, solvetime, params_i.tolist(), output_array.tolist()]
        
        #adding the new data line
        self.writer.writerow(data_line)
        


# Main
if __name__ == '__main__':
    try:
        rospy.init_node('MPCC_node', anonymous=False)
        dt = 0.1 # control loop time
        vehicle_number_1 = 1
        vehicle_controller = MPCC_controller_class(vehicle_number_1, dt)

        rate = rospy.Rate(1 / dt)
        while not rospy.is_shutdown():
            vehicle_controller.MPCC_control_loop()
            rate.sleep() # removing sleep from here will make the control loop run as fast as possible

    except rospy.ROSInterruptException:
        pass
