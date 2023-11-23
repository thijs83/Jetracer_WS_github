#!/usr/bin/env python3
import numpy as np
import math
from scipy import interpolate, integrate
from scipy.spatial.transform import Rotation
import sys
import rospy
import control
import os
from pathlib import Path
from functions_for_MPCC_node_running import find_s_of_closest_point_on_global_path,\
                                            evaluate_local_path_Chebyshev_coefficients_high_order_cheby,\
                                            produce_s_local_path,\
                                            produce_track,\
                                            load_GP_parameters_from_file




from std_msgs.msg import Float32, Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from datetime import datetime
import csv
# for dynamic paramters reconfigure (setting param values from rqt_reconfigure GUI)
from dynamic_reconfigure.server import Server
from dynamic_reconfigure_pkg.cfg import GP_MPCC_dynamic_reconfigureConfig
import rospkg
from custom_msgs_optitrack.msg import custom_opti_pose_stamped_msg
from Helper_functions.utility_functions import generate_paths, model_attributes
paths = generate_paths()
#paths.add_forces_path_to_sys_path()
import forcespro.nlp
from GP_MPCC_generate_solvers.functions_for_GP_solver_generation import Functions_for_solver_generation
from Helper_functions.GP_fittinig_functions import load_GP_models_and_save_forces_params


#fory dynamic parameter change using rqt_reconfigure GUI
class MPC_GUI_manager:
    def __init__(self, vehicles_list):
        #fory dynamic parameter change using rqt_reconfigure GUI
        self.vehicles_list = vehicles_list
        srv = Server(GP_MPCC_dynamic_reconfigureConfig, self.reconfig_callback)


    def reconfig_callback(self, config, level):
        print('_________________________________________________')
        print('  reconfiguring parameters from dynamic_reconfig ')
        print('‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾')
        
        for i in range(len(self.vehicles_list)):
            self.vehicles_list[i].V_target = config['V_target']
            self.vehicles_list[i].q1 = config['q1']
            self.vehicles_list[i].q2 = config['q2']
            self.vehicles_list[i].q3 = config['q3']
            self.vehicles_list[i].q4 = config['q4']
            self.vehicles_list[i].l_width = config['l_width']
            self.vehicles_list[i].l_shift = config['l_shift']
            self.vehicles_list[i].safety_filter_p = config['safety_filter_p']
            self.vehicles_list[i].max_steer_right = config['max_steer_right']
            self.vehicles_list[i].max_steer_left = config['max_steer_left']
            self.vehicles_list[i].warm_start = config['warm_start']
            self.vehicles_list[i].dt = config['dt']
            self.vehicles_list[i].solver_choice = config['solver_choice']
            self.vehicles_list[i].save_data = config['save_data']
            self.vehicles_list[i].data_folder_name = config['data_folder_name']
            self.vehicles_list[i].minimal_plotting = config['minimal_plotting']

            if self.vehicles_list[i].save_data == False:
                try:
                    #close file when recording is switched off
                    self.vehicles_list[i].file.close()
                except:
                    #print('')
                    pass


            # assemble parameters in order for changes to take effect on parameter_block1 and 2
            self.vehicles_list[i].set_solver_type()

            # if self.model_attributes_obj.regulate_kernel_exponent == True:
            #     #must re-evaluate KXX matrix used in The GP
            #     save_forces_params = True
            #     exponential_kernel_gamma = self.safety_filter_p
            #     load_GP_models_and_save_forces_params(self.model_attributes_obj.path_to_GP_model, save_forces_params,exponential_kernel_gamma)

            self.vehicles_list[i].set_GP_paramters() # if switching between uncertainty yes/no you have to regenerate the GP parameters accordingly
            self.vehicles_list[i].assemble_fixed_parameters()
            self.rate = rospy.Rate(1 / self.vehicles_list[0].dt)


            print('‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾')

        return config











class MPCC_controller_class(Functions_for_solver_generation):
    def __init__(self, car_number):
        

        
        #set up variables
        self.car_number = car_number
        self.solver_choice = 11  # setting default (Note that if running GUI this value will be overwritten)
        
        # initialize state variables
        self.vx = 0
        self.vy = 0
        self.omega = 0
        self.opti_state = [0, 0, 0]


        # intialize uncertainty state variables
        self.sigmaVx = 0
        self.sigmaVy = 0
        self.sigmaW = 0

        # The actual value is set inside the callback to opti state
        self.sigmaX = 0
        self.sigmaY = 0
        self.sigmaXY = 0
        self.sigmaT = 0
        self.sigmaTX = 0
        self.sigmaTY = 0

        # previous steer in case of delay compensation
        self.prev_steer = 0

        # set up default safety filter conservativeness (it will be overwritten when calling GUI)
        self.safety_filter_p = 10

        # define selected solver
        self.set_solver_type()

        #set up constant problem parameters 
        self.setup_constant_parameters()

        # initialize path relative variables
        self.previous_index = 1

        # initialize dynamic obstacle realeted
        self.dyn_ob_traj_x = np.zeros(30) + 10
        self.dyn_ob_traj_y = np.zeros(30) + 10
        self.dyn_ob_traj_r = np.zeros(30) + 0.15

        #define rviz related topics
        self.set_up_topics_for_rviz()

        #produce an object to access the lane boundary definition functions (only really need for visualization)
        #self.Functions_for_solver_generation_obj = Functions_for_solver_generation()



        #for data time stamp initialize sensor data
        self.start_elapsed_time = rospy.get_rostime()
        self.safety_value = 0
        self.current = 0
        self.voltage = 0
        self.IMU_acceleration = [0, 0, 0]
        self.encoder_velocity = 0
        #self.data_folder_name = 'Data'
        #self.file = 0
        #self. writer = 0
        #self.start_elapsed_time = 0
        #self.setup_data_recording()

        # Track related
        #track_choice = 'savoiardo'
        #track_choice = 'double_donut'
        #track_choice = 'racetrack_Lab'
        #track_choice = 'racetrack_saturate_steering'
        track_choice = 'racetrack_Lab_safety_GP'

        self.loop_path = True

        # produce track related fixed quantities
        self.generate_track(track_choice)


        # set up publisher and subscribers
        self.throttle_publisher = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=1)
        self.steering_publisher = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=1)
        self.vx_subscriber = rospy.Subscriber('vx_' + str(car_number), Float32, self.vx_subscriber_callback)
        self.vy_subscriber = rospy.Subscriber('vy_' + str(car_number), Float32, self.vy_subscriber_callback)
        self.omega_subscriber = rospy.Subscriber('omega_' + str(car_number), Float32, self.omega_subscriber_callback)
        self.optitrack_state_subscriber = rospy.Subscriber('Optitrack_data_topic_' + str(car_number), custom_opti_pose_stamped_msg, self.opti_state_subscriber_callback)
        self.safety_value_subscriber = rospy.Subscriber('safety_value', Float32, self.safety_value_subscriber_callback)
        self.occupancy_xyr_4_subscriber = rospy.Subscriber('occupancy_xyr_4', Float32MultiArray, self.occupancy_xyr_4_subscriber_callback)
        self.comptime_publisher = rospy.Publisher('comptime_' + str(car_number), Float32, queue_size=1)
        self.new_GP_params = rospy.Subscriber("new_GP_params_" + str(car_number), Bool, self.new_GP_params_callback)



        #for data collection
        self.sub_cur = rospy.Subscriber("current_" + str(car_number), Float32, self.sub_cur_callback)
        self.sub_vol = rospy.Subscriber("voltage_" + str(car_number), Float32, self.sub_vol_callback)
        self.sub_acc = rospy.Subscriber("IMU_" + str(car_number), Float32MultiArray, self.sub_acc_callback)
        self.sub_vel = rospy.Subscriber("velocity_" + str(car_number), Float32, self.sub_vel_callback)



        # send out global path message to rviz
        rgba = [160, 189, 212, 0.25]
        global_path_message = self.produce_marker_array_rviz(self.x_vals_original_path, self.y_vals_original_path, rgba)
        self.rviz_global_path_publisher.publish(global_path_message)




    def run_one_MPCC_control_loop(self):

        # self.rate = rospy.Rate(1 / self.dt)
        # #self.rate = rospy.Rate(1)

        # while not rospy.is_shutdown():
            # get clock estimated for timing purpouses
        # moved the control loop outside

        start_clock_time = rospy.get_rostime()
        # at runtime, local path and dynamic obstacle need to be updated. Dyn obst is updated by the subscriber callback

        a, b, coeffx, coeffy, self.s = self.produce_chebyshev_coeffs() # this automatically reads the current state and global path parameters

        # assemble all parameters  (thiscould be done cleaner but get everything working firs)
        params_i = np.array([a, b, *coeffx, *coeffy, *self.fixed_params_block_1, *self.dyn_ob_traj_x, *self.dyn_ob_traj_y, *self.dyn_ob_traj_r, *self.fixed_params_block_2, *self.GP_parameters])
        all_params_array = np.tile(params_i, self.N)  # give as a stacked vector of parameters (see codeoptions.nlp.stack_parambounds in solver generation phase)

        problem = self.produce_problem_for_forces(all_params_array)
        
        # call solver
        try:
            output, exitflag, info = self.solver.solve(problem)
        except:
            print('Failure in calling the solver, trying to reset the solver choice')
            print('uncertainties',self.model_attributes_obj.uncertainties)
            rospy.sleep(0.2) # wait this amount of time so that the reconfig callback can finish executing
            # this is a bit of a hack but ok
            self.set_solver_type()
            self.setup_constant_parameters()
            # assemble all parameters, needed to update the GP parameters inside all param array
            params_i = np.array([a, b, *coeffx, *coeffy, *self.fixed_params_block_1, *self.dyn_ob_traj_x, *self.dyn_ob_traj_y, *self.dyn_ob_traj_r, *self.fixed_params_block_2, *self.GP_parameters])
            all_params_array = np.tile(params_i, self.N)  # give as a stacked vector of parameters (see codeoptions.nlp.stack_parambounds in solver generation phase)
            problem = self.produce_problem_for_forces(all_params_array)
            output, exitflag, info = self.solver.solve(problem)

        
        start_clock_time_after_solver = rospy.get_rostime()
        output_array = np.array(list(output.values()))

        



        # check if solver converged
        if exitflag != 1:
            print('Ouch solver did not converge exitflag = ', exitflag)
        else:  # converged so update warm start (it will only be used if warm_start = true)
            #always update warm start solution, then if warm_start is True it will be used, if not it wont be used
            self.previous_solution = output_array[0]
            for cc in range(1, self.N):
                self.previous_solution = np.concatenate((self.previous_solution, output_array[cc]))

        self.publish_control_inputs(output_array)

        #update previous steer (it will not be used if not needed)
        self.prev_steer = output_array[0,1]


        if self.minimal_plotting == False:
            # prepare stuff for rviz
            self.produce_and_publish_rviz_visualization(output_array, self.Cheby_data_points, params_i,
                                                        self.s, self.Ds_forecast, self.s_vals_global_path,
                                                        self.loop_path, self.x_vals_original_path, self.y_vals_original_path)


        stop_clock_time = rospy.get_rostime()
        total_time = stop_clock_time.secs - start_clock_time.secs + (
                    stop_clock_time.nsecs - start_clock_time.nsecs) / 1000000000
        self.comptime_publisher.publish(total_time)
        # rospy.loginfo("Elapsed time = %f [seconds]", total_time)

        # store data if necessary
        if self.save_data == True:
            try:
                # write new row
                self.write_new_data_row(total_time, info.solvetime, params_i, output_array)
            except:
                print('creating new data file')
                self.setup_data_recording()
                self.writer.writerow(['s global path', 'x global path', 'y global path'])
                self.writer.writerow([s_vals_global_path, x_vals_original_path, y_vals_original_path])
                self.writer.writerow(
                    ['elapsed time', 'opti x', 'opti y', 'opti theta', 'safety_value', 'throttle', 'steering',
                        'current',
                        'voltage', 'IMU[0]', 'IMU[1]', 'IMU[2]', 'endocder velocity', 'vx', 'vy', 'omega', 'total comp time','solver comp time','params_i',
                        'solver solution'])
                self.write_new_data_row(total_time, info.solvetime, params_i, output_array)

            # self.rate.sleep() # removing this means the controller will run as fast as it can

    def set_solver_type(self):

        print('setting solver type')
        self.model_attributes_obj = model_attributes(self.solver_choice)

        print('Solver choice: ', self.model_attributes_obj.solver_name)
        if os.path.isdir(self.model_attributes_obj.path_to_solver) == False:
            print('+++++++++++++++++++++++++++++++++++++++++++++++++')
            print('Warning! the provided solver location is invalid')
            print('+++++++++++++++++++++++++++++++++++++++++++++++++')
        
        self.solver = forcespro.nlp.Solver.from_directory(self.model_attributes_obj.path_to_solver)



    def setup_constant_parameters(self):
        # setu up N
        self.N = 30  # must match the number of stages in the solver

        #max steering angles for constrints
        self.max_steer = 17

        # Solver related
        self.warm_start = False
        warm_start_solution_0 = np.concatenate(([0, 0, 0], [0, 0, 0, 0, 0, 0, 0]), axis=0)
        self.previous_solution = np.tile(warm_start_solution_0, self.N)
        self.warm_start_solution = np.tile(warm_start_solution_0, self.N)

        # generate parameters
        self.V_target = 1  # in terms of being scaled down the proportion is vreal life[km/h] = v[m/s]*42.0000  (assuming the 30cm jetracer is a 3.5 m long car)
        self.L = 0.175  # length of the jetracer [m]

        self.dt = 0.1  # so first number is the prediction horizon in seconds -this is the dt of the solver so it will think that the control inputs are changed every dt seconds
        # xo = -0.05
        # yo = 1.65
        self.xo = 10
        self.yo = 10
        # ro = 0.043     #a scaled down pedestrian (circle 1 mt wide) is r = 0.043
        self.ro = 0.15
        # so to avoid going through the pedestrian dt should be less than dt*v<r
        # -->   dt<r/v
        self.l_shift = 0  # shift of centre of lane to the right
        self.l_width = 0.6  # width of lane
        self.slack_p_1 = 1000  # controls the cost of the slack variable
        self.slack_p_2 = 1
        self.slack_p_3 = (self.ro * 0.66) ** 2


        if self.solver_choice == 1:  # machinig formulation
            # j = self.q1 * (x[3] - V_target) ** 2 + self.q2 * err_lat_squared + self.q3 * err_lag_squared + self.q4 * u[0] ** 2 + self.q4 * u[1] ** 2
            self.q1 = 1  # relative weight of s_dot following
            self.q2 = 5  # relative weight of lat error
            self.q3 = 1  # relative weight of lag error -- this is the extra term that needs to be tuned
            self.q4 = 1  # relative weight of inputs  (weighted the same here)

        else:  # CAMPCC
            # j = self.q1 * (s_dot - V_target) ** 2 + self.q2 * err_lat_squared + self.q3 * u[0] ** 2 + self.q4 * u[1] ** 2
            self.q1 = 1  # relative weight of s_dot following
            self.q2 = 0.5  # relative weight of lat error
            self.q3 = 0.1  # relative weight of steering (Weighting the two inputs identically to be fair
            self.q4 = 1  # relative weight of throttle



        # Set GP paramters
        self.set_GP_paramters()


        # assemble fixed parameters. This is separate so you can call it again when you reset values from dynamic_reconfig
        self.assemble_fixed_parameters()


    def set_GP_paramters(self):
        # load GP parmaters from a specified file
        #abs_path_parameters_folder = "/home/lorenzo/OneDrive/PhD/Code/GPs_for_macchinine/Codice_Lyons/Saved_models_and_data_simulations/"
        # x_data_vec_vx, x_data_vec_vy, x_data_vec_w, outputscale_Delta_vx, lengthscales_Delta_vx, right_vec_block_Delta_vx, \
        # outputscale_Delta_vy, lengthscales_Delta_vy, right_vec_block_Delta_vy, \
        # outputscale_Delta_w, lengthscales_Delta_w, right_vec_block_Delta_w,\
        # central_mat_Delta_vx_vector, central_mat_Delta_vy_vector, central_mat_Delta_w_vector = load_GP_parameters_from_file(abs_path_parameters_folder)
        
        if self.model_attributes_obj.dynamic_model == 'GP' or self.model_attributes_obj.dynamic_model == 'Dyn_bike_GP':
            x_data_vec_vx, x_data_vec_vy, x_data_vec_w, outputscale_Delta_vx, lengthscales_Delta_vx, right_vec_block_Delta_vx,\
            outputscale_Delta_vy, lengthscales_Delta_vy, right_vec_block_Delta_vy, \
            outputscale_Delta_w, lengthscales_Delta_w, right_vec_block_Delta_w,\
            central_mat_Delta_vx_vector, central_mat_Delta_vy_vector, central_mat_Delta_w_vector,\
            x_data_vec_vx_cov, x_data_vec_vy_cov, x_data_vec_w_cov = load_GP_parameters_from_file(self.model_attributes_obj.path_to_GP_model)
            
            
            # base GP parameters
            self.GP_parameters = [*x_data_vec_vx, *x_data_vec_vy, *x_data_vec_w,
                    outputscale_Delta_vx, *lengthscales_Delta_vx, *right_vec_block_Delta_vx,
                    outputscale_Delta_vy, *lengthscales_Delta_vy, *right_vec_block_Delta_vy,
                    outputscale_Delta_w, *lengthscales_Delta_w, *right_vec_block_Delta_w]
            
            
            # if using uncertainties in solver also add central matrices
            if self.model_attributes_obj.uncertainties == True or self.model_attributes_obj.safety_filter == True:
                # add cholesky matrix for uncertainty evaluation
                self.GP_parameters = [*self.GP_parameters,*central_mat_Delta_vx_vector, *central_mat_Delta_vy_vector, *central_mat_Delta_w_vector]
                
                if self.model_attributes_obj.orthogonally_decoupled == True:
                    # add inducing points for uncertainty evaluation
                    self.GP_parameters = [*self.GP_parameters,*x_data_vec_vx_cov, *x_data_vec_vy_cov, *x_data_vec_w_cov]

            if self.model_attributes_obj.safety_filter == True or self.model_attributes_obj.regulate_kernel_exponent == True:
                self.GP_parameters = [*self.GP_parameters, self.safety_filter_p] #set default value of safety filter conservativeness to 10

            # set values related to GP data size, this is needed for unpacking gp parameters inside some visualization functions
            self.n_data = right_vec_block_Delta_vx.shape[0]  # number of data points
            self.m_features = int(x_data_vec_vx.shape[0]/right_vec_block_Delta_vx.shape[0]) # number of features 


        else: # the dynamic model is not a GP
            self.GP_parameters = []



    def assemble_fixed_parameters(self):
        # solve LQR problem to define the terminal cost (if V_target changes put this inside the while loop)
        # linearized longitudinal vehicle model
        # numerical values taken from BEP paper, so linearized around th = 0.129 --> V=1 m/s
        C = 1.54 / 1.63  # longitudinal damping coefficient divided by the mass
        a_th = 60 / 1.63  # motor curve coefficient divided by the mass
        b_th = 1.54 / 1.63  # motor curve offset (actually not used at the moment)


        A = np.array([[0, self.V_target, 0], [0, 0, 0], [0, 0, -C]])
        B = np.array([[0, 0], [0, self.V_target / self.L], [a_th, 0]])
        Q = np.array([[self.q2, 0, 0], [0, 0, 0], [0, 0, self.q1]])
        R = np.array([[self.q3, 0], [0, self.q4]])
        K, S, E = control.lqr(A, B, 2 * Q, 2 * R)
        S = S

        self.fixed_params_block_1 = [self.V_target, self.L, C, a_th, b_th, self.dt, self.xo, self.yo, self.ro]
        self.fixed_params_block_2 = [self.l_shift, self.l_width, self.q1, self.q2, self.q3, self.q4,
                                S[0, 0], S[0, 1], S[0, 2], S[1, 0], S[1, 1], S[1, 2], S[2, 0], S[2, 1], S[2, 2],
                                self.slack_p_1, self.slack_p_2, self.slack_p_3]



    def generate_track(self, track_choice):
        # generate track
        # choice = 'savoiardo'
        # choice = 'double_donut'
        # choice = 'straight_line'
        # choice = 'savoiardo_saturate_steering'
        # choice = 'circle'
        # choice = 'racetrack_saturate_steering'
        # choice = 'racetrack_Lab'
        # choice = 'gain_sweep_track_2'
        n_checkppoints = 30

        Checkpoints_x, Checkpoints_y = produce_track(track_choice, n_checkppoints)

        # this is a bit strange so work on improving this thing
        tck, u = interpolate.splprep([Checkpoints_x, Checkpoints_y], s=0, k=3)

        # evaluate arc length given a certain discretization
        spline_discretization = 1000
        t_vec = np.linspace(0, u[-1], spline_discretization)
        xy_vals = interpolate.splev(t_vec, tck)

        self.x_vals_original_path = xy_vals[0]
        self.y_vals_original_path = xy_vals[1]

        # evaluate path parameters by numerically integrating
        self.s_vals_global_path = np.zeros(spline_discretization)
        # define infinitesimal arc length function
        ds = lambda t: np.sqrt(np.dot(interpolate.splev(t, tck, der=1, ext=0), interpolate.splev(t, tck, der=1, ext=0)))

        for ii in range(1, spline_discretization):
            ds_precision = integrate.quad(ds, t_vec[ii - 1], t_vec[ii])
            self.s_vals_global_path[ii] = self.s_vals_global_path[ii - 1] + ds_precision[0]

        # generate splines  x(s) and y(s) where s is now the arc length value (starting from 0)
        self.x_of_s = interpolate.CubicSpline(self.s_vals_global_path, self.x_vals_original_path)
        self.y_of_s = interpolate.CubicSpline(self.s_vals_global_path, self.y_vals_original_path)


    def produce_chebyshev_coeffs(self):
        # measure the closest point on the global path
        estimated_ds = self.vx * self.dt  # esitmated ds from previous time instant (velocity is measured now so not accounting for acceleration, but this is only for the search of the s initial s value, so no need to be accurate)
        s, self.current_index = find_s_of_closest_point_on_global_path(np.array([self.opti_state[0], self.opti_state[1]]), self.s_vals_global_path,
                                                                  self.x_vals_original_path, self.y_vals_original_path,
                                                                  self.previous_index, estimated_ds)
        self.previous_index = self.current_index  # update index along the path to know where to search in next iteration


        # define initial guess for solver (chosen as zero input so static initial state)
        # u = [throttle steer slack]

        # evaluate local path
        # estimated needed path parameter length
        self.Ds_forecast = self.V_target * self.dt * self.N * 1.2
        self.Cheby_data_points = 100

        a = s  # be careful between resetting to 0 or keeping measured value
        b = a + self.Ds_forecast


        # this takes really low time, like less than 0.001 seconds
        # start_clock_time_cheby_fitting = rospy.get_rostime()
        
        coeffx, coeffy = evaluate_local_path_Chebyshev_coefficients_high_order_cheby(s, self.x_of_s, self.y_of_s, self.Ds_forecast,
                                                                                     self.s_vals_global_path, self.loop_path,
                                                                                     self.Cheby_data_points)
        return a, b, coeffx, coeffy, s

    def produce_problem_for_forces(self, all_params_array):
        # define initial condition for the solver
        # x = x y theta vx vy w s
        if self.model_attributes_obj.uncertainties == False:
            if self.model_attributes_obj.delay_comp == False:
                xinit = np.array([self.opti_state[0], self.opti_state[1], self.opti_state[2], self.vx, self.vy, self.omega, self.s])
            else:
                xinit = np.array([self.opti_state[0], self.opti_state[1], self.opti_state[2], self.vx, self.vy, self.omega, self.s,self.prev_steer])
        else:
            if self.model_attributes_obj.delay_comp == False:
                xinit = np.array([self.opti_state[0], self.opti_state[1], self.opti_state[2], self.vx, self.vy, self.omega, self.s,
                                self.sigmaVx, self.sigmaVy, self.sigmaW,
                                self.sigmaX,self.sigmaY,self.sigmaXY,self.sigmaT, self.sigmaTX, self.sigmaTY]) 
            else:
                xinit = np.array([self.opti_state[0], self.opti_state[1], self.opti_state[2], self.vx, self.vy, self.omega, self.s,
                                self.sigmaVx, self.sigmaVy, self.sigmaW,
                                self.sigmaX,self.sigmaY,self.sigmaXY,self.sigmaT, self.sigmaTX, self.sigmaTY, self.prev_steer]) 

            
        if self.warm_start:
            n_stage_vars = int(len(self.previous_solution)/self.N)
            warm_start_solution = np.concatenate((self.previous_solution[n_stage_vars:], self.previous_solution[-n_stage_vars:]))
            #x0_array = self.warm_start_solution
            #warm_start_solution[3:10] = xinit  # force the first stage state to be consistent with xinit
            x0_array = warm_start_solution
        else:
            x0 = np.concatenate(([0.15, 0, 0], xinit), axis=0)
            x0_array = np.tile(x0, self.N)  # concatenate for each stage



        # produce the problem as a dictionary for the solver
        problem = {"x0": x0_array, "xinit": xinit, "all_parameters": all_params_array}
        # add parametric bounds (only s changes online)
        # this must match the indexes decalred when generating the solver (throttle steer slack path_parameter)

        #retrieve a, b values (extreemes of path paramter values)
        a = all_params_array[0]
        b = all_params_array[1]

        # steering conversion has been moved to inside integrator function
        problem["lb"] = np.tile([0, -1, 0, a], self.N)
        problem["ub"] = np.tile([0.3, 1, 100, b], self.N)

        return problem

    def publish_control_inputs(self, output_array):
        # publish throttle values
        throttle_val = Float32(output_array[0, 0])
        # publish steering (saturating first)


        #temporary override to give zeros
        steering = output_array[0, 1]
        #steering = 0.0



        #steering = np.max([steering, -self.max_steer/180*np.pi])
        #steering = np.min([steering, self.max_steer/180*np.pi])
        #steering_val = Float32(steering / np.pi * 180 / self.max_steer) # convert to degrees)
        steering_val = Float32(steering)
        #if -output_array[0, 1] < 0:
        #    steering_val = Float32(-output_array[0, 1] / (self.max_steer_left / 180.0 * np.pi))
        #else:
        #    steering_val = Float32(-output_array[0, 1] / (self.max_steer_right / 180.0 * np.pi))



        # for data storage purpouses
        self.throttle = throttle_val.data
        self.steering = steering_val.data

        self.throttle_publisher.publish(throttle_val)
        self.steering_publisher.publish(steering_val)


    def setup_data_recording(self):
        date_time = datetime.now()
        date_time_str = date_time.strftime("%m_%d_%Y_%H_%M_%S")
        rospack = rospkg.RosPack()
        gp_pkg_path = rospack.get_path('gp_mpcc_pkg')
        print(gp_pkg_path)
        file_name = gp_pkg_path + '/src/' + self.data_folder_name + '/recording_' + date_time_str + '.csv'
        file = open(file_name, 'w')
        writer = csv.writer(file)
        start_elapsed_time = rospy.get_rostime()

        self.file = file
        self. writer = writer
        self.start_elapsed_time = start_elapsed_time
        return

    def write_new_data_row(self, total_time, solvetime ,  params_i, output_array):
        # store data here
        stop_clock_time = rospy.get_rostime()
        elapsed_time = stop_clock_time.secs - self.start_elapsed_time.secs + (
                stop_clock_time.nsecs - self.start_elapsed_time.nsecs) / 1000000000
        data_line = [elapsed_time, self.opti_state[0], self.opti_state[1], self.opti_state[2],
                     self.safety_value, self.throttle, self.steering, self.current,
                     self.voltage, self.IMU_acceleration[0], self.IMU_acceleration[1], self.IMU_acceleration[2], self.encoder_velocity,
                     self.vx, self.vy, self.omega, total_time, solvetime, params_i.tolist(), output_array.tolist()]
        self.writer.writerow(data_line)





    def produce_and_publish_rviz_visualization(self, output_array, Cheby_data_points, params_i,s,Ds_forecast,s_vals_global_path,loop_path,
                            x_vals_original_path, y_vals_original_path):

        plot_counter = 0
        # plot x-y state prediction

        # extract x and y prediction
        x_vec = output_array[:, 3]
        y_vec = output_array[:, 4]
        # evaluating local path as solver will do it (inside continuous dynamics)
        dummy_x = [0, 0, 0, 0, 0, 0, 0]
        x_path_from_solver_function = np.zeros(Cheby_data_points)
        y_path_from_solver_function = np.zeros(Cheby_data_points)
        x_left_lane_boundary = np.zeros(Cheby_data_points)
        y_left_lane_boundary = np.zeros(Cheby_data_points)
        x_right_lane_boundary = np.zeros(Cheby_data_points)
        y_right_lane_boundary = np.zeros(Cheby_data_points)


        #
        s_subpath_cheby_for_xy_data_generation, s_subpath_for_fitting_operation = produce_s_local_path(s, Ds_forecast,
                                                                                                       Cheby_data_points,
                                                                                                       s_vals_global_path,
                                                                                                       loop_path)
        # s_subpath_cheby = s_subpath_for_fitting_operation

        for ii in range(0, Cheby_data_points):
            # dummy_x[4] = s_subpath_for_fitting_operation[ii]
            # #Cx_out, Cy_out, s_dot, x_Cdev, y_Cdev = evaluate_spline_quantities_machining_MPCC_cheby(dummy_x, params_i)
            # Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby_high_order(dummy_x, params_i)
            dummy_x[6] = s_subpath_for_fitting_operation[ii]
            
            Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = self.evaluate_spline_quantities_GP(dummy_x, params_i)


            #local path
            x_path_from_solver_function[ii] = Cx_out
            y_path_from_solver_function[ii] = Cy_out
            #lane boundaries l_shift to right
            #enforce strictly that norm should be 1 just to make the plots nicer
            norm = np.sqrt(x_Cdev ** 2 + y_Cdev ** 2)
            #chek if curvature boundary applies

            V_x_left = (-self.l_shift + self.l_width/2) * x_Cdev / norm
            V_y_left = (-self.l_shift + self.l_width/2) * y_Cdev / norm
            V_x_right = (self.l_shift + self.l_width/2) * x_Cdev / norm
            V_y_right = (self.l_shift + self.l_width/2) * y_Cdev / norm

            x_left_lane_boundary[ii] = Cx_out - V_y_left
            y_left_lane_boundary[ii] = Cy_out + V_x_left
            x_right_lane_boundary[ii] = Cx_out + V_y_right
            y_right_lane_boundary[ii] = Cy_out - V_x_right


        # plot boundaries as seen by the solver
        x_left_lane_boundary_by_solver = np.zeros(self.N)
        y_left_lane_boundary_by_solver = np.zeros(self.N)
        x_right_lane_boundary_by_solver = np.zeros(self.N)
        y_right_lane_boundary_by_solver = np.zeros(self.N)
        s_prediction = output_array[:, -1]


        for ii in range(0, self.N):
            state_i = output_array[ii, 3:10]
            #Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby_high_order(state_i, params_i)

            Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = self.evaluate_spline_quantities_GP(state_i, params_i)

            # norm = np.sqrt(x_Cdev ** 2 + y_Cdev ** 2)
            # # chek if curvature boundarie applies
            # V_x_left = (-self.l_shift + self.l_width / 2) * x_Cdev / norm
            # V_y_left = (-self.l_shift + self.l_width / 2) * y_Cdev / norm
            # V_x_right = (self.l_shift + self.l_width / 2) * x_Cdev / norm
            # V_y_right = (self.l_shift + self.l_width / 2) * y_Cdev / norm

            # chek if curvature boundarie applies
            V_x_left = (-self.l_shift + self.l_width / 2) * x_Cdev
            V_y_left = (-self.l_shift + self.l_width / 2) * y_Cdev
            V_x_right = (self.l_shift + self.l_width / 2) * x_Cdev
            V_y_right = (self.l_shift + self.l_width / 2) * y_Cdev


            # x_left_lane_boundary_by_solver[ii] = Cx_out
            # y_left_lane_boundary_by_solver[ii] = Cy_out
            x_left_lane_boundary_by_solver[ii] = Cx_out - V_y_left
            y_left_lane_boundary_by_solver[ii] = Cy_out + V_x_left
            x_right_lane_boundary_by_solver[ii] = Cx_out + V_y_right
            y_right_lane_boundary_by_solver[ii] = Cy_out - V_x_right

            # if ii == 0:
            #     print('s_dot = ', s_dot , 'v', state_i[3])


        #  For rviz
        #open loop prediction mean
        rgba = [0, 166, 214, 1.0]
        rviz_MPCC_path_message = self.produce_marker_array_rviz(x_vec, y_vec, rgba)
        self.rviz_MPC_path_publisher.publish(rviz_MPCC_path_message)

        #open loop prediction uncertainty region
        rgba_cov = [0, 166, 214, 0.1]
        sigma_interval = 3 # 3 sigma interval --> 99.7% confidence region
        if self.model_attributes_obj.uncertainties == True:
            rviz_MPCC_uncertainty_region_message = self.produce_ellipse_marker(output_array, sigma_interval, rgba_cov)
            self.rviz_MPC_uncertainty_region_publisher.publish(rviz_MPCC_uncertainty_region_message)

        rgba = [112, 173, 219, 0.25]
        rviz_local_path_message = self.produce_marker_array_rviz(x_path_from_solver_function, y_path_from_solver_function,rgba)
        self.rviz_local_path_publisher.publish(rviz_local_path_message)

        rgba = [57.0, 81.0, 100.0, 1.0]
        rviz_left_lane_bound_message = self.produce_marker_array_rviz(x_left_lane_boundary, y_left_lane_boundary,rgba)
        self.rviz_left_lane_publisher.publish(rviz_left_lane_bound_message)

        rviz_right_lane_publisher_message = self.produce_marker_array_rviz(x_right_lane_boundary, y_right_lane_boundary,rgba)
        self.rviz_right_lane_publisher.publish(rviz_right_lane_publisher_message)




    def set_up_topics_for_rviz(self):

        self.rviz_MPC_path_publisher = rospy.Publisher('rviz_MPC_path_' + str(self.car_number), MarkerArray, queue_size=10)
        self.rviz_MPC_uncertainty_region_publisher = rospy.Publisher('rviz_MPC_uncertainty_region_' + str(self.car_number), MarkerArray, queue_size=10)
        self.rviz_global_path_publisher = rospy.Publisher('rviz_global_path_' + str(self.car_number), MarkerArray, queue_size=10)
        self.rviz_local_path_publisher = rospy.Publisher('rviz_local_path_' + str(self.car_number), MarkerArray, queue_size=10)
        self.rviz_left_lane_publisher = rospy.Publisher('rviz_left_lane_' + str(self.car_number), MarkerArray, queue_size=10)
        self.rviz_right_lane_publisher = rospy.Publisher('rviz_right_lane_' + str(self.car_number), MarkerArray, queue_size=10)

    def produce_marker_array_rviz(self, x, y, rgba):
        marker_array = MarkerArray()
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3 ; LINE_STRIP: 4
        marker.type = 4
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
        #marker.pose.position.x = x[i]
        #marker.pose.position.y = y[i]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        points_list = []
        for i in range(len(x)):
            p = Point()
            p.x = x[i]
            p.y = y[i]
            points_list = points_list + [p]

        marker.points = points_list

        # assign to array
        marker_array.markers.append(marker)




        return  marker_array
    
    def produce_ellipse_marker(self, output_array, sigma_interval, rgba):
        #                 0   1     2   3 4   5   6  7  8 9    10     11      12     13     14     15      16      17      18
        #output array = [th steer slack x y theta vx vy w s sigmaVx sigmaVy sigmaW sigmaX sigmaY sigmaXY sigmaT sigmaTX sigma TY]


        marker_array = MarkerArray()
        # print('---')
        for i in range(1, output_array.shape[0],4):
            #rebuild covariance partix of x y position
            covar_mat = np.array([[output_array[i,13],output_array[i,15]],[output_array[i,15],output_array[i,14]]])



            #solving to find the rotation that will diagonalize the covariance matrix
            angle = 0.5 * np.arctan2((2*covar_mat[0,1]),(covar_mat[0,0]-covar_mat[1,1]))
            # apply rotation to diagonalize the matrix
            Rot_mat = np.zeros((2,2))
            Rot_mat[0,0:2] = [np.cos(angle), np.sin(angle)]
            Rot_mat[1,0:2] = [-np.sin(angle), np.cos(angle)]
            diag_covar_mat = Rot_mat @ covar_mat @ Rot_mat.T 

            semidiam_x = np.sqrt(diag_covar_mat[0,0])
            semidiam_y = np.sqrt(diag_covar_mat[1,1])

            r = Rotation.from_euler('z', - angle) # minus sign cause we want the rotation that will bring the diagonalized covar back to the rael one

            #produce new marker
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3 ; LINE_STRIP: 4
            marker.type = 3
            marker.id = i

            # Set the scale of the marker
            marker.scale.x = semidiam_x * sigma_interval * 2 # times 2 because this sets the "diameter"
            marker.scale.y = semidiam_y * sigma_interval * 2
            marker.scale.z = 0.001

            # Set the color
            marker.color.r = rgba[0] / 256
            marker.color.g = rgba[1] / 256
            marker.color.b = rgba[2] / 256
            marker.color.a = rgba[3]

            # Set the pose of the marker
            marker.pose.position.x = output_array[i, 3]
            marker.pose.position.y = output_array[i, 4]
            marker.pose.position.z = 0
            marker.pose.orientation.x = r.as_quat()[0]
            marker.pose.orientation.y = r.as_quat()[1]
            marker.pose.orientation.z = r.as_quat()[2]
            marker.pose.orientation.w = r.as_quat()[3]

            # assign to array
            marker_array.markers.append(marker)


        #marker.points = points_list

        




        return  marker_array




        return 




    # define state subscriber callbacks
    def vx_subscriber_callback(self, msg):
        self.vx = msg.data

    def vy_subscriber_callback(self, msg):
        self.vy = msg.data

    def omega_subscriber_callback(self, msg):
        self.omega = msg.data

    def opti_state_subscriber_callback(self, msg):
        self.opti_state = [msg.x, msg.y, msg.rotation]
        # also set up initial covariance matrix, this is a bit temporary as a thing
        angle = msg.rotation

        diag_covar_mat = np.array([[0.001,0],[0,0.001]])
        Rot_mat = np.zeros((2,2))
        Rot_mat[0,0:2] = [np.cos(angle), np.sin(angle)]
        Rot_mat[1,0:2] = [-np.sin(angle), np.cos(angle)]
        covar_mat = Rot_mat @ diag_covar_mat @ Rot_mat.T 

        self.sigmaX = covar_mat[0,0]
        self.sigmaXY = covar_mat[0,1]
        self.sigmaY = covar_mat[1,1]
        self.sigmaT = 0.001 # this is a bit random
        self.sigmaTX = 0
        self.sigmaTY = 0


    def safety_value_subscriber_callback(self, msg):
        self.safety_value = msg.data


    def sub_cur_callback(self, msg):
        self.current = msg.data

    def sub_vol_callback(self, msg):
        self.voltage = msg.data

    def sub_acc_callback(self, msg):
        self.IMU_acceleration = msg.data

    def sub_vel_callback(self, msg):
        self.encoder_velocity = msg.data



    def occupancy_xyr_4_subscriber_callback(self, msg):
        # some legacy shorter vector can also be accepted
        # temporary block out to just test the tracking performance
        dyn_ob_traj = msg.data
        if len(dyn_ob_traj) == 45:
            self.dyn_ob_traj_x[0:15] = dyn_ob_traj[0:15]
            self.dyn_ob_traj_y[0:15] = dyn_ob_traj[15:30]
            self.dyn_ob_traj_r[0:15] = dyn_ob_traj[30:45]
            self.dyn_ob_traj_x[15:31] = dyn_ob_traj[14] + np.zeros(15)
            self.dyn_ob_traj_y[15:31] = dyn_ob_traj[29] + np.zeros(15)
            self.dyn_ob_traj_r[15:31] = dyn_ob_traj[34] + np.zeros(15)
        elif len(dyn_ob_traj) == 90:
            self.dyn_ob_traj_x[0:31] = dyn_ob_traj[0:30]
            self.dyn_ob_traj_y[0:31] = dyn_ob_traj[30:60]
            self.dyn_ob_traj_r[0:31] = dyn_ob_traj[60:90]




    def new_GP_params_callback(self, new_GP_params_msg):
        if new_GP_params_msg.data:
            self.set_GP_paramters()
            print('GP_parameters have been updated inside MPC controller')


















if __name__ == '__main__':
    try:
        rospy.init_node('MPCC_node', anonymous=False)
        global_comptime_publisher = rospy.Publisher('GLOBAL_comptime', Float32, queue_size=1)

        #set up vehicle controllers
        #car 1
        car_number = 1
        vehicle_1_controller = MPCC_controller_class(1)

        #car 3
        vehicle_3_controller = MPCC_controller_class(3)



        # start control loop
        dt = 0.1
        rate = rospy.Rate(1 / dt)
        #NOTE that this rate is the rate to send out ALL control imputs to all vehicles

        vehicle_controllers_list = [vehicle_1_controller, vehicle_3_controller]

        #set up GUI manager
        MPC_GUI_manager_obj = MPC_GUI_manager(vehicle_controllers_list)


        while not rospy.is_shutdown():
            try:
                start_clock_time = rospy.get_rostime()
                for i in range(len(vehicle_controllers_list)):
                    vehicle_controllers_list[i].run_one_MPCC_control_loop()


                stop_clock_time = rospy.get_rostime()
                elapsed_time_global_loop = stop_clock_time.secs - start_clock_time.secs + (stop_clock_time.nsecs - start_clock_time.nsecs) / 1000000000
                global_comptime_publisher.publish(elapsed_time_global_loop)
                rate.sleep()

            except:
                print('MPCC control loop failed, waiting ang trying again')
                #rospy.sleep(0.2)

            # rate.sleep() # commenting this out means that the mpc will run as fast as it can




    except rospy.ROSInterruptException:
        pass
