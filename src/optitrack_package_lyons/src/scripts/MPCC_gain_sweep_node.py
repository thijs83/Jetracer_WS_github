#!/usr/bin/env python3
import numpy as np
from scipy import interpolate, integrate
import sys
import control
import time
import random
from functions_for_MPCC_node_running import evaluate_spline_quantities_cheby, evaluate_local_path,\
                                            evaluate_spline_quantities_cheby_high_order,\
                                            s_eval_a_posteriori_numerically, unpack_parameters_cheby,\
                                            unpack_parameters_cheby_high_order,\
                                            find_s_of_closest_point_on_global_path,\
                                            evaluate_spline_quantities_machining_MPCC_cheby,\
                                            evaluate_local_path_Chebyshev_coefficients,\
                                            evaluate_local_path_Chebyshev_coefficients_high_order_cheby,\
                                            produce_s_local_path,\
                                            produce_track

import matplotlib.pyplot as plt
import shelve
from matplotlib.patches import Rectangle
from tqdm import tqdm

# import forcespro (add path before you add it)
sys.path.insert(0, '/home/lorenzo/OneDrive/PhD/ubuntu_stuff/Forces_pro_extracted/forces_pro_client')
import forcespro.nlp



# choose what solver you want to build
# 1 = cheby solver
# 2 = machining cheby solver
# 3 = large Ds formulation
# 4 = Forward Euler
solver_choice = 2

#sweeping indexes
n_trials = 1  # number of trials for each q2 - q3 combination

# set up simulations parameters
q1 = 1  # s_dot follow
q4 = 0.1  # steering gain in large DS and both input weights in machining

# q2_gain_sweep_vec = np.linspace(0, 12, 25) machining
q2_gain_sweep_vec = np.linspace(0, 20, 11)
q2_gain_sweep_vec[0] = 0.0001

q2_indexes = range(len(q2_gain_sweep_vec))
if solver_choice == 2:
    q3_gain_sweep_vec = np.linspace(0, 3, 10)
    q3_gain_sweep_vec[0] = 0.0001
    q3_indexes = range(len(q3_gain_sweep_vec))
    progress_bar = tqdm(range(len(q2_gain_sweep_vec) * len(q3_gain_sweep_vec) * n_trials), desc='Progress',
                        colour='green')
else:
    q3_gain_sweep_vec = [q4]  # this is now the throttle weight
    q3_indexes = [0]
    progress_bar = tqdm(range(len(q2_gain_sweep_vec) * n_trials), desc='Progress', colour='green')

















# this takes the solver
if solver_choice == 1:
    # solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_Cheby_from_python_slack"
    solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_Cheby_full_slack_TerminalCost"
    solver_name = 'Cheby_full_slack_TerminalCost_'
elif solver_choice == 2:
    # solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_machining_MPCC_Cheby_path_slack_TerminalCost_high_order_cheby_RACING"
    # solver_name = 'Machining_slack__TerminalCost_high_order_cheby_RACING'
    solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_machining_MPCC_Cheby_path_slack_TerminalCost_high_order_cheby"
    solver_name = 'Machining'
elif solver_choice == 3:
    # solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_Cheby_Large_DS_formulation_slack_TerminalCost_largeN_highCheby_RACING"
    # solver_name = 'Cheby_large_DS_slack_TerminalCost_high_order_cheby_RACING'
    solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_Cheby_Large_DS_formulation_slack_TerminalCost_largeN_highCheby"
    solver_name = 'Large_DS'
elif solver_choice == 4:
    solver_location = "/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/FORCES_solver_Forward_euler_slack_TerminalCost"
    solver_name = 'Cheby_forward_Euler_slack_TerminalCost'
else:
    print('invalid solver choice')

print('Solver choice: ', solver_name)
solver = forcespro.nlp.Solver.from_directory(solver_location)


# generate track
# choice = 'savoiardo'
# choice = 'double_donut'
# choice = 'straight_line'
# choice = 'savoiardo_saturate_steering'
# choice = 'circle'
# choice = 'racetrack_saturate_steering'
# choice = 'gain_sweep_track'
choice = 'gain_sweep_track_2'
n_checkppoints = 30
Checkpoints_x, Checkpoints_y = produce_track(choice, n_checkppoints)



# setu up N
N = 30  # must match the number of stages in the solver

# generate parameters
V_target = 0.75  # in terms of being scaled down the proportion is vreal life[km/h] = v[m/s]*42.0000  (assuming the 30cm jetracer is a 3.5 m long car)
L = 0.175  # length of the jetracer [m]
# numerical values taken from BEP paper, so linearized around th = 0.129 --> V=1 m/s
C = 1.54 / 1.63  # longitudinal damping coefficient divided by the mass
a_th = 60 / 1.63  # motor curve coefficient divided by the mass
b_th = 1.54 / 1.63  # motor curve offset (actually not used at the moment)
dt = 0.1  # so first number is the prediction horizon in seconds -this is the dt of the solver so it will think that the control inputs are changed every dt seconds

xo = Checkpoints_x[int(3.5 * n_checkppoints) -1]
yo = Checkpoints_y[int(3.5 * n_checkppoints)- 1]
ro = 0.15
l_shift = 0  # shift of centre of lane to the right
l_width = 0.6  # width of lane
slack_p_1 = 1000  # controls the cost of the slack variable
slack_p_2 = 1
slack_p_3 = (ro * 0.66) ** 2

# set obstacle position
dyn_ob_traj_x = np.zeros(30) + Checkpoints_x[int(1.5 * n_checkppoints) - 1]
dyn_ob_traj_y = np.zeros(30) + Checkpoints_y[int(1.5 * n_checkppoints) - 1]
dyn_ob_traj_r = np.zeros(30) + 0.15


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

# set initial index for closest point on the path search
previous_index = 0



warm_start = True
print('warm starting = ', warm_start)
# warm_start_solution_0 = np.concatenate(([0, 0, 0], [0, 0, 0, 0, 0]), axis=0)
# warm_start_solution = np.tile(warm_start_solution_0, N)




simulation_time_old = 0
# while not rospy.is_shutdown():
time_limit = 100


# reset initial state
initial_checkpoint = int(n_checkppoints/2 - 1)
# state_initial = [Checkpoints_x[initial_checkpoint], Checkpoints_y[initial_checkpoint], 0, 0]
# state = state_initial



#define plotting figures regulations
plot_figures = False
plot_every_n_figures = 10
plot_counter = 1





# this is needed to plot the figure in real time
plt.ion()
# this is just to give the name to the data
exp_counter = 1



#set up some randomness in the initial state
random.seed(a=123123123, version=2)


for k in range(len(q2_gain_sweep_vec)):
    q2 = q2_gain_sweep_vec[k]
    for i in range(len(q3_gain_sweep_vec)):
        q3 = q3_gain_sweep_vec[i]

        # print('Resetting')
        # print('q2 = ', q2)
        # print('q3 = ', q3)


        # solve LQR problem to define the terminal cost (if V_target changes put this inside the while loop)
        A = np.array([[0, V_target, 0], [0, 0, 0], [0, 0, -C]])
        B = np.array([[0, 0], [0, V_target / L], [a_th, 0]])
        Q = np.array([[q2, 0, 0], [0, 0, 0], [0, 0, q1]])
        R = np.array([[q3, 0], [0, q4]])
        K, S, E = control.lqr(A, B, Q, R)

        problem_related_parameters = np.array([V_target, L, C, a_th, b_th, dt, xo, yo, ro,
                                               *dyn_ob_traj_x, *dyn_ob_traj_y, *dyn_ob_traj_r, l_shift,
                                               l_width, q1, q2, q3, q4, S[0, 0], S[0, 1], S[0, 2],
                                               S[1, 0], S[1, 1], S[1, 2], S[2, 0], S[2, 1], S[2, 2], slack_p_1,
                                               slack_p_2, slack_p_3])
        for trial in range(0, n_trials):
            # reset data storage variables
            short_output_array_stacked = np.array([0, 0, 0, 0, 0, 0, 0, 0])
            time_stamp_stacked = np.array([0])
            safety_value_stacked = np.array([0])
            params_i_array_stacked = np.array(np.zeros(159))
            exitflag_stacked = np.array([0])
            total_time_stacked = np.array([0])
            solver_time_stacked = np.array([0])
            Full_plotting_time_stacked = np.array([0])

            state = [Checkpoints_x[initial_checkpoint], Checkpoints_y[initial_checkpoint] + 0.1*(random.randint(-100, +100)/100) ,
                     0 + np.pi * 0.25*(random.randint(-100, +100)/100), 0.75 + 0.25 * (random.randint(-100, +100)/100)]
            previous_index = initial_checkpoint
            warm_start_solution_0 = np.concatenate(([0, 0, 0], [0, 0, 0, 0, 0]), axis=0)
            warm_start_solution = np.tile(warm_start_solution_0, N)


            for t in range(0, time_limit):
                # print('q2 = ', q2, '  q3 = ', q3, 'trial' )

                start_clock_time = time.time()
                # define initial position
                # measure the closest point on the global path
                estimated_ds = state[3] * dt  # esitmated ds from previous time instant (velocity is measured now so not accounting for acceleration, but this is only for the search of the s initial s value, so no need to be accurate)
                s, current_index = find_s_of_closest_point_on_global_path(np.array([state[0], state[1]]), s_vals_global_path,
                                                                          x_vals_original_path, y_vals_original_path,
                                                                          previous_index, estimated_ds)
                previous_index = current_index  # update index along the path to know where to search in next iteration

                # define initial condition for the solver
                xinit = np.array([state[0], state[1], state[2], state[3], s])

                # define initial guess for solver (chosen as zero input so static initial state)
                # u = [throttle steer slack]
                if warm_start:
                    x0_array = warm_start_solution
                else:
                    x0 = np.concatenate(([0, 0, 0], xinit), axis=0)
                    x0_array = np.tile(x0, N)  # concatenate for each stage

                # evaluate local path
                # estimated needed path parameter length
                Ds_forecast = V_target * dt * N * 1.2
                Cheby_data_points = 100

                a = s  # be carefull between resetting to 0 or keeping measured value
                b = a + Ds_forecast


                coeffx, coeffy = evaluate_local_path_Chebyshev_coefficients_high_order_cheby(s, x_of_s, y_of_s, Ds_forecast, s_vals_global_path,
                                                                            loop_path,
                                                                            Cheby_data_points)

                # assemble local path related parameters
                params_i = np.concatenate((np.array([a]), np.array([b]), coeffx, coeffy, problem_related_parameters), axis=0)
                all_params_array = np.tile(params_i, N)

                all_params_array = np.tile(params_i,
                                           N)  # give as a stacked vector of parameters (see codeoptions.nlp.stack_parambounds in solver generation phase)

                # produce the problem as a dictionary for the solver
                problem = {"x0": x0_array, "xinit": xinit, "all_parameters": all_params_array}
                # add parametric bounds (only s changes online)
                # this must match the indexes decalred when generating the solver (throttle steer slack path_parameter)
                problem["lb"] = np.tile([0.10, -20.0 / 180.0 * np.pi, 0, a], N)
                problem["ub"] = np.tile([0.25, 20.0 / 180.0 * np.pi, 100, b], N)

                # call solver
                output, exitflag, info = solver.solve(problem)
                output_array = np.array(list(output.values()))

                # check if solver converged
                if exitflag != 1:
                    # print('Ouch solver did not converge exitflag = ', exitflag)
                    if warm_start == True:
                        # print('Resetting initial guess')
                        # reset initial guess and try again
                        x0 = np.concatenate(([0, 0, 0], xinit), axis=0)
                        warm_start_solution = np.tile(x0, N)

                else:  # converged so update warm start (it will only be used if warm_start = true)
                    warm_start_solution = np.reshape(output_array, (1, N * 8))
                    warm_start_solution = warm_start_solution[0]

                # publish throttle values
                # throttle_val = Float32(output_array[0, 0])
                # steering_val = Float32(-output_array[0, 1] / (20.0 / 180.0 * np.pi))
                # throttle_publisher.publish(throttle_val)
                # steering_publisher.publish(steering_val)

                stop_clock_time = time.time()
                total_time = stop_clock_time - start_clock_time



                # prepare data to be stored (ADD TIME VECTOR)
                short_output_array_stacked = np.vstack([short_output_array_stacked, output_array[0, :]])
                time_stamp_stacked = np.vstack([time_stamp_stacked, np.array([float(t * dt)])])
                safety_value_stacked = np.vstack([safety_value_stacked, np.array([float(1)])])
                params_i_array_stacked = np.vstack([params_i_array_stacked, params_i])
                exitflag_stacked = np.vstack([exitflag_stacked, exitflag])
                total_time_stacked = np.vstack([total_time_stacked, total_time])
                solver_time_stacked = np.vstack([solver_time_stacked, info.solvetime])

                #step 1 time instant forwards by taking it from the output array
                state = output_array[1, 3:7]

                # check if overtake was successful

                # if np.abs(simulation_time - simulation_time_old) > 1:
                #     # change the gains
                #     # then later in post processing you can determine if the overtake was successful or not
                #     if solver_choice == 2:
                #         # if q3 + q3_increment < q3_max:
                #         q3 = q3 + q3_increment
                #         # else:
                #         # q3 = q3_reset #avoid doing this because the files get too large
                #         # q2 = q2 + q2_increment
                #     else:
                #         q2 = q2 + q2_increment
                #
                #     print('updating q2 =', q2, 'updating q3 = ', q3)
                #
                # simulation_time_old = simulation_time

                ###
                if plot_figures and plot_counter == plot_every_n_figures:
                    plot_counter = 0
                    # plot x-y state prediction

                    # extract x and y prediction
                    x_vec = output_array[:, 3]
                    y_vec = output_array[:, 4]
                    # evaluating local path as solver will do it (inside continuous dynamics)
                    dummy_x = [0, 0, 0, 0, 0]
                    x_path_from_solver_function = np.zeros(Cheby_data_points)
                    y_path_from_solver_function = np.zeros(Cheby_data_points)
                    x_left_lane_boundary = np.zeros(Cheby_data_points)
                    y_left_lane_boundary = np.zeros(Cheby_data_points)
                    x_right_lane_boundary = np.zeros(Cheby_data_points)
                    y_right_lane_boundary = np.zeros(Cheby_data_points)

                    # this is what is done inside the function "evaluate_local_path_Chebyshev_coefficients_manuel"
                    s_subpath_cheby_for_xy_data_generation, s_subpath_for_fitting_operation = produce_s_local_path(s,
                                                                                                                   Ds_forecast,
                                                                                                                   Cheby_data_points,
                                                                                                                   s_vals_global_path,
                                                                                                                   loop_path)
                    # s_subpath_cheby = s_subpath_for_fitting_operation

                    for ii in range(0, Cheby_data_points):
                        dummy_x[4] = s_subpath_for_fitting_operation[ii]
                        # Cx_out, Cy_out, s_dot, x_Cdev, y_Cdev = evaluate_spline_quantities_machining_MPCC_cheby(dummy_x, params_i)
                        Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby_high_order(
                            dummy_x, params_i)
                        # local path
                        x_path_from_solver_function[ii] = Cx_out
                        y_path_from_solver_function[ii] = Cy_out
                        # lane boundaries l_shift to right
                        # enforce strictly that norm should be 1 just to make the plots nicer
                        norm = np.sqrt(x_Cdev ** 2 + y_Cdev ** 2)
                        # chek if curvature boundarie applies

                        V_x_left = (-l_shift + l_width / 2) * x_Cdev / norm
                        V_y_left = (-l_shift + l_width / 2) * y_Cdev / norm
                        V_x_right = (l_shift + l_width / 2) * x_Cdev / norm
                        V_y_right = (l_shift + l_width / 2) * y_Cdev / norm

                        x_left_lane_boundary[ii] = Cx_out - V_y_left
                        y_left_lane_boundary[ii] = Cy_out + V_x_left
                        x_right_lane_boundary[ii] = Cx_out + V_y_right
                        y_right_lane_boundary[ii] = Cy_out - V_x_right

                    # plot boundaries as seen by the solver
                    x_left_lane_boundary_by_solver = np.zeros(N)
                    y_left_lane_boundary_by_solver = np.zeros(N)
                    x_right_lane_boundary_by_solver = np.zeros(N)
                    y_right_lane_boundary_by_solver = np.zeros(N)
                    s_prediction = output_array[:, -1]

                    for ii in range(0, N):
                        state_i = output_array[ii, 3:8]
                        Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby_high_order(
                            state_i,
                            params_i)
                        # norm = np.sqrt(x_Cdev ** 2 + y_Cdev ** 2)
                        # # chek if curvature boundarie applies
                        # V_x_left = (-l_shift + l_width / 2) * x_Cdev / norm
                        # V_y_left = (-l_shift + l_width / 2) * y_Cdev / norm
                        # V_x_right = (l_shift + l_width / 2) * x_Cdev / norm
                        # V_y_right = (l_shift + l_width / 2) * y_Cdev / norm

                        # chek if curvature boundarie applies
                        V_x_left = (-l_shift + l_width / 2) * x_Cdev
                        V_y_left = (-l_shift + l_width / 2) * y_Cdev
                        V_x_right = (l_shift + l_width / 2) * x_Cdev
                        V_y_right = (l_shift + l_width / 2) * y_Cdev

                        # x_left_lane_boundary_by_solver[ii] = Cx_out
                        # y_left_lane_boundary_by_solver[ii] = Cy_out
                        x_left_lane_boundary_by_solver[ii] = Cx_out - V_y_left
                        y_left_lane_boundary_by_solver[ii] = Cy_out + V_x_left
                        x_right_lane_boundary_by_solver[ii] = Cx_out + V_y_right
                        y_right_lane_boundary_by_solver[ii] = Cy_out - V_x_right

                        # if ii == 0:
                        #     print('s_dot = ', s_dot , 'v', state_i[3])

                    # Plot figure
                    # print('s_dot = ', (output_array[1,7]-output_array[0, 7]) / dt)
                    for ii in range(0, N):
                        dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r
                        xo_now = dyn_ob_traj_x[ii]
                        yo_now = dyn_ob_traj_y[ii]
                        ro_now = dyn_ob_traj_r[ii]
                        circle1 = plt.Circle((xo_now, yo_now), ro_now, color='c', alpha=0.5 * (1 - ii / N))
                        plt.gca().add_patch(circle1)
                    circle1 = plt.Circle((xo, yo), ro, color='r')
                    plt.gca().add_patch(circle1)
                    rect1 = Rectangle(
                        (x_vec[0] + (L / 1.5) / 2 * np.sin(state[2]), y_vec[0] - (L / 1.5) / 2 * np.cos(state[2])), L,
                        (L / 1.5), angle=state[2] / np.pi * 180, color='green')
                    plt.gca().add_patch(rect1)
                    plt.plot(x_vals_original_path, y_vals_original_path, label="Original global path")
                    plt.scatter(x_vals_original_path[current_index], y_vals_original_path[current_index],
                                label="current closest point on path")
                    plt.plot(x_path_from_solver_function, y_path_from_solver_function,
                             label="Local path from inside solver function")
                    plt.plot(x_vec, y_vec, label="xy_state_prediction")
                    plt.plot(x_left_lane_boundary, y_left_lane_boundary, label='left lane boundaries', color='tan')
                    plt.plot(x_right_lane_boundary, y_right_lane_boundary, label='right lane boundaries', color='tan')
                    plt.plot(x_left_lane_boundary_by_solver, y_left_lane_boundary_by_solver,
                             label='left lane boundaries by solver', color='black')
                    plt.plot(x_right_lane_boundary_by_solver, y_right_lane_boundary_by_solver,
                             label=' right lane boundaries by solver ', color='black')
                    plt.ylabel('x')
                    plt.ylabel('y')
                    plt.axis('equal')
                    plt.legend()
                    plt.title(solver_name + 'time index = ' + str(t) + '  q2 = ' + str(q2) + '   q3 = ' + str(q3))
                    plt.xlim(x_vec[0] - 2, x_vec[0] + 2)
                    plt.ylim(y_vec[0] - 2, y_vec[0] + 2)
                    plt.show()
                    plt.pause(0.1)
                    plt.clf()

                    plot_counter = plot_counter + 1
                    #####
                else:
                    plot_counter = plot_counter + 1

                # stop_clock_time_plots = rospy.get_rostime()
                stop_clock_time_plots = time.time()
                total_time_plots = stop_clock_time_plots - start_clock_time
                # rospy.loginfo("Elapsed time with plots= %f [seconds]", total_time_plots)
                Full_plotting_time_stacked = np.vstack([Full_plotting_time_stacked, total_time_plots])



            #update progress bar
            progress_bar.update(1)

            # data storage
            # save the problem to a file for later use
            # now = datetime.now()  # current date and time
            # date = now.strftime("%m_%d_%Y")
            # time = now.strftime("%H_%M_%S")

            # get time and date

            # filename = '/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/MPCC_Lyons/Ease_of_tuning_tests/' + solver_name +\
            #            '/' + solver_name + '_q2_' + f"{q2:2.3f}" + '_q3_' + f"{q3:2.3f}" + '.db'
            filename = '/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/MPCC_Lyons/Ease_of_tuning_tests/' + solver_name +\
                       '_quick/' + solver_name + '_experiment_' + str(exp_counter) + '.db'
            exp_counter = exp_counter + 1
            d = shelve.open(filename, writeback=False)

            try:
                #plot final trajectory
                plt.plot(x_vals_original_path, y_vals_original_path, label="Original global path")
                plt.plot(short_output_array_stacked[1:, 3], short_output_array_stacked[1:, 4], label='trajectory')
                plt.ylabel('x')
                plt.ylabel('y')
                plt.axis('equal')
                plt.show()
                plt.pause(0.01)
                plt.clf()

                d['MPCC_results'] = {'problem_related_parameters': [problem_related_parameters],
                                     'params_i_array_stacked': params_i_array_stacked[1:, :],
                                     'short_output_array_stacked': short_output_array_stacked[1:, :],
                                     # 'time_stamp_stacked': time_stamp_stacked[1:, :],
                                     # 'safety_value_stacked': safety_value_stacked[1:, :],
                                     # 'exitflag_stacked': exitflag_stacked[1:, :],
                                     # 'total_time_stacked': total_time_stacked[1:, :],
                                     # 'solver_time_stacked': solver_time_stacked[1:, :],
                                     's_vals_global_path': [s_vals_global_path],
                                     'x_vals_original_path': [x_vals_original_path],
                                     'y_vals_original_path': [y_vals_original_path]}

                # d['MPCC_results'] = { 'constant_problem_parameters': constant_problem_parameters, 'other_parameters': np.array([omega]),'output_array_stacked': output_array_stacked, 'Theta_array_1_2_5': Theta_array_1_2_5,'safety_value_list':safety_value_list}
            finally:
                d.close()



        # plot computation times
        # plt.figure(2)
        # plt.plot(total_time_stacked[1:], label="total_time")
        # plt.plot(solver_time_stacked[1:], label="solver_time")
        # plt.plot(Full_plotting_time_stacked[1:], label="Full iteration time with plotting")
        # plt.ylabel('iteration')
        # plt.ylabel('seconds')
        # plt.legend()
        # plt.show()
        # plt.pause(20)




    # rate.sleep()

    # # break loop if conditions are met
    # if q2 > q2_max or q3 > q3_max:
    #     break






# if __name__ == '__main__':
#     try:
#         MPCC_controller()
#     except rospy.ROSInterruptException:
#         pass
