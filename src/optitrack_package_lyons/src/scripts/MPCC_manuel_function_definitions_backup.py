import numpy as np
import math
import sys
sys.path.insert(0, '/home/lorenzo/OneDrive/PhD/ubuntu_stuff/Forces_pro_extracted/forces_pro_client')
import forcespro.nlp
import casadi


def unpack_parameters_manuel(p):
    #NOTE THAT: dynamic obstacle position and radius must now match the number of stages since it is the only way to
    # give force a time dependent function
    a = p[0]
    b = p[1]
    coeffx_vec = p[2:12]
    coeffy_vec = p[12:22]
    V_target = p[22]
    L = p[23]
    C = p[24]
    a_th = p[25]
    b_th = p[26]
    dt = p[27]
    q1 = p[28]
    q2 = p[29]
    q3 = p[30]
    q4 = p[31]
    wx_L = p[32]
    scale_x_L = p[33]
    wy_L = p[34]
    scale_y_L = p[35]
    coefftheta_vec = p[36:46]
    return a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, q1, q2, q3, q4, wx_L, scale_x_L, wy_L, scale_y_L, coefftheta_vec


def evaluate_path_quantities(x, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, q1, q2, q3, q4, wx_L, scale_x_L, wy_L, scale_y_L, coefftheta_vec = unpack_parameters_manuel(p)

    # x = [x, y, eta, v, s]
    s = x[4]
    t = s
    dxdy = 1
    x_T0 = coeffx_vec[0] * (1)
    x_T0_dev = coeffx_vec[0] * (dxdy) * (0)
    x_T0_dev2 = coeffx_vec[0] * ((dxdy) ** 2) * (0)
    x_T1 = coeffx_vec[1] * (t)
    x_T1_dev = coeffx_vec[1] * (dxdy) * (1)
    x_T1_dev2 = coeffx_vec[1] * ((dxdy) ** 2) * (0)
    x_T2 = coeffx_vec[2] * (2 * t ** 2 - 1)
    x_T2_dev = coeffx_vec[2] * (dxdy) * (4 * t)
    x_T2_dev2 = coeffx_vec[2] * ((dxdy) ** 2) * (4)
    x_T3 = coeffx_vec[3] * (4 * t ** 3 - 3 * t)
    x_T3_dev = coeffx_vec[3] * (dxdy) * (12 * t ** 2 - 3)
    x_T3_dev2 = coeffx_vec[3] * ((dxdy) ** 2) * (24 * t)
    x_T4 = coeffx_vec[4] * (8 * t ** 4 - 8 * t ** 2 + 1)
    x_T4_dev = coeffx_vec[4] * (dxdy) * (32 * t ** 3 - 16 * t)
    x_T4_dev2 = coeffx_vec[4] * ((dxdy) ** 2) * (96 * t ** 2 - 16)
    x_T5 = coeffx_vec[5] * (5 * t - 20 * t ** 3 + 16 * t ** 5)
    x_T5_dev = coeffx_vec[5] * (dxdy) * (80 * t ** 4 - 60 * t ** 2 + 5)
    x_T5_dev2 = coeffx_vec[5] * ((dxdy) ** 2) * (320 * t ** 3 - 120 * t)
    x_T6 = coeffx_vec[6] * (18 * t ** 2 - 48 * t ** 4 + 32 * t ** 6 - 1)
    x_T6_dev = coeffx_vec[6] * (dxdy) * (36 * t - 192 * t ** 3 + 192 * t ** 5)
    x_T6_dev2 = coeffx_vec[6] * ((dxdy) ** 2) * (960 * t ** 4 - 576 * t ** 2 + 36)
    x_T7 = coeffx_vec[7] * (56 * t ** 3 - 7 * t - 112 * t ** 5 + 64 * t ** 7)
    x_T7_dev = coeffx_vec[7] * (dxdy) * (168 * t ** 2 - 560 * t ** 4 + 448 * t ** 6 - 7)
    x_T7_dev2 = coeffx_vec[7] * ((dxdy) ** 2) * (336 * t - 2240 * t ** 3 + 2688 * t ** 5)
    x_T8 = coeffx_vec[8] * (160 * t ** 4 - 32 * t ** 2 - 256 * t ** 6 + 128 * t ** 8 + 1)
    x_T8_dev = coeffx_vec[8] * (dxdy) * (640 * t ** 3 - 64 * t - 1536 * t ** 5 + 1024 * t ** 7)
    x_T8_dev2 = coeffx_vec[8] * ((dxdy) ** 2) * (1920 * t ** 2 - 7680 * t ** 4 + 7168 * t ** 6 - 64)
    x_T9 = coeffx_vec[9] * (9 * t - 120 * t ** 3 + 432 * t ** 5 - 576 * t ** 7 + 256 * t ** 9)
    x_T9_dev = coeffx_vec[9] * (dxdy) * (2160 * t ** 4 - 360 * t ** 2 - 4032 * t ** 6 + 2304 * t ** 8 + 9)
    x_T9_dev2 = coeffx_vec[9] * ((dxdy) ** 2) * (8640 * t ** 3 - 720 * t - 24192 * t ** 5 + 18432 * t ** 7)


    Cx_out = x_T0 + x_T1 + x_T2 + x_T3 + x_T4 + x_T5 + x_T6 + x_T7 + x_T8 + x_T9

    x_Cdev = x_T0_dev + x_T1_dev + x_T2_dev + x_T3_dev + x_T4_dev + x_T5_dev + x_T6_dev + x_T7_dev + x_T8_dev + x_T9_dev

    #x_Cdev2 = x_T0_dev2 + x_T1_dev2 + x_T2_dev2 + x_T3_dev2 + x_T4_dev2 + x_T5_dev2 + x_T6_dev2 + x_T7_dev2 + x_T8_dev2 + x_T9_dev2

    y_T0 = coeffy_vec[0] * (1)
    y_T0_dev = coeffy_vec[0] * (dxdy) * (0)
    y_T0_dev2 = coeffy_vec[0] * ((dxdy) ** 2) * (0)
    y_T1 = coeffy_vec[1] * (t)
    y_T1_dev = coeffy_vec[1] * (dxdy) * (1)
    y_T1_dev2 = coeffy_vec[1] * ((dxdy) ** 2) * (0)
    y_T2 = coeffy_vec[2] * (2 * t ** 2 - 1)
    y_T2_dev = coeffy_vec[2] * (dxdy) * (4 * t)
    y_T2_dev2 = coeffy_vec[2] * ((dxdy) ** 2) * (4)
    y_T3 = coeffy_vec[3] * (4 * t ** 3 - 3 * t)
    y_T3_dev = coeffy_vec[3] * (dxdy) * (12 * t ** 2 - 3)
    y_T3_dev2 = coeffy_vec[3] * ((dxdy) ** 2) * (24 * t)
    y_T4 = coeffy_vec[4] * (8 * t ** 4 - 8 * t ** 2 + 1)
    y_T4_dev = coeffy_vec[4] * (dxdy) * (32 * t ** 3 - 16 * t)
    y_T4_dev2 = coeffy_vec[4] * ((dxdy) ** 2) * (96 * t ** 2 - 16)
    y_T5 = coeffy_vec[5] * (5 * t - 20 * t ** 3 + 16 * t ** 5)
    y_T5_dev = coeffy_vec[5] * (dxdy) * (80 * t ** 4 - 60 * t ** 2 + 5)
    y_T5_dev2 = coeffy_vec[5] * ((dxdy) ** 2) * (320 * t ** 3 - 120 * t)
    y_T6 = coeffy_vec[6] * (18 * t ** 2 - 48 * t ** 4 + 32 * t ** 6 - 1)
    y_T6_dev = coeffy_vec[6] * (dxdy) * (36 * t - 192 * t ** 3 + 192 * t ** 5)
    y_T6_dev2 = coeffy_vec[6] * ((dxdy) ** 2) * (960 * t ** 4 - 576 * t ** 2 + 36)
    y_T7 = coeffy_vec[7] * (56 * t ** 3 - 7 * t - 112 * t ** 5 + 64 * t ** 7)
    y_T7_dev = coeffy_vec[7] * (dxdy) * (168 * t ** 2 - 560 * t ** 4 + 448 * t ** 6 - 7)
    y_T7_dev2 = coeffy_vec[7] * ((dxdy) ** 2) * (336 * t - 2240 * t ** 3 + 2688 * t ** 5)
    y_T8 = coeffy_vec[8] * (160 * t ** 4 - 32 * t ** 2 - 256 * t ** 6 + 128 * t ** 8 + 1)
    y_T8_dev = coeffy_vec[8] * (dxdy) * (640 * t ** 3 - 64 * t - 1536 * t ** 5 + 1024 * t ** 7)
    y_T8_dev2 = coeffy_vec[8] * ((dxdy) ** 2) * (1920 * t ** 2 - 7680 * t ** 4 + 7168 * t ** 6 - 64)
    y_T9 = coeffy_vec[9] * (9 * t - 120 * t ** 3 + 432 * t ** 5 - 576 * t ** 7 + 256 * t ** 9)
    y_T9_dev = coeffy_vec[9] * (dxdy) * (2160 * t ** 4 - 360 * t ** 2 - 4032 * t ** 6 + 2304 * t ** 8 + 9)
    y_T9_dev2 = coeffy_vec[9] * ((dxdy) ** 2) * (8640 * t ** 3 - 720 * t - 24192 * t ** 5 + 18432 * t ** 7)

    Cy_out = y_T0 + y_T1 + y_T2 + y_T3 + y_T4 + y_T5 + y_T6 + y_T7 + y_T8 + y_T9

    y_Cdev = y_T0_dev + y_T1_dev + y_T2_dev + y_T3_dev + y_T4_dev + y_T5_dev + y_T6_dev + y_T7_dev + y_T8_dev + y_T9_dev

    #y_Cdev2 = y_T0_dev2 + y_T1_dev2 + y_T2_dev2 + y_T3_dev2 + y_T4_dev2 + y_T5_dev2 + y_T6_dev2 + y_T7_dev2 + y_T8_dev2 + y_T9_dev2


    # theta(s) evaluation
    theta_T0 = coefftheta_vec[0] * (1)
    theta_T1 = coefftheta_vec[1] * (t)
    theta_T2 = coefftheta_vec[2] * (2 * t ** 2 - 1)
    theta_T3 = coefftheta_vec[3] * (4 * t ** 3 - 3 * t)
    theta_T4 = coefftheta_vec[4] * (8 * t ** 4 - 8 * t ** 2 + 1)
    theta_T5 = coefftheta_vec[5] * (5 * t - 20 * t ** 3 + 16 * t ** 5)
    theta_T6 = coefftheta_vec[6] * (18 * t ** 2 - 48 * t ** 4 + 32 * t ** 6 - 1)
    theta_T7 = coefftheta_vec[7] * (56 * t ** 3 - 7 * t - 112 * t ** 5 + 64 * t ** 7)
    theta_T8 = coefftheta_vec[8] * (160 * t ** 4 - 32 * t ** 2 - 256 * t ** 6 + 128 * t ** 8 + 1)
    theta_T9 = coefftheta_vec[9] * (9 * t - 120 * t ** 3 + 432 * t ** 5 - 576 * t ** 7 + 256 * t ** 9)

    theta_out = theta_T0 + theta_T1 + theta_T2 + theta_T3 + theta_T4 + theta_T5 + theta_T6 + theta_T7 + theta_T8 + theta_T9

    return Cx_out, Cy_out, x_Cdev, y_Cdev, theta_out


def continuous_dynamics_Jetracer(x, u, p):
    # define the continuous time derivatives that will be used by RK4
    # using simple ds = V * dt to integrate the path parameter

    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, q1, q2, q3, q4, wx_L, scale_x_L, wy_L, scale_y_L, coefftheta_vec = unpack_parameters_manuel(p)
    #  x = [x y rotation velocity path parameter]
    xdot1 = x[3] * casadi.cos(x[2])  # x_dot = v * cos(eta)
    xdot2 = x[3] * casadi.sin(x[2])  # y_dot = v * sin(eta)
    xdot3 = x[3] * casadi.tan(u[1]) / L  # v * tan(delta) / L
    #xdot4 = -C * x[3] + u[0] * a_th - b_th  # vdot = -C*v+a_th*throttle+b_throttle  (linear motor torque - linear viscosity)
    #linearized model taken from bachelor group
    xdot4 = -C * (x[3] - 1) + (u[0] - 0.129) * a_th
    xdot5 = x[3]  # v Simple s_dot approx taken from machining MPCC formulation

    # assemble derivatives
    xdot = np.array([xdot1, xdot2, xdot3, xdot4, xdot5])
    return xdot

def Jetracer_dynamics(z, p):
    #Evaluate the next state by forwards integrating using Rounge-Kutta order 4
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, q1, q2, q3, q4, wx_L, scale_x_L, wy_L, scale_y_L, coefftheta_vec = unpack_parameters_manuel(p)

    # extract x, u
    u = z[0:2]
    x = z[2:7]

    x_next = forcespro.nlp.integrate(continuous_dynamics_Jetracer, x, u, p,
                                     integrator=forcespro.nlp.integrators.RK4, stepsize=dt)
    return x_next

def Objective(z, p):
    # objective function for the optimization problem

    #  extract state
    u = z[0:2]
    x = z[2:7]
    # extract parameters
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, q1, q2, q3, q4, wx_L, scale_x_L, wy_L, scale_y_L, coefftheta_vec = unpack_parameters_manuel(p)
    #evaluate spline quantities
    Cx_out, Cy_out, x_Cdev, y_Cdev, theta_out = evaluate_path_quantities(x, p)
    #  the tangent vector's norm should already be 1 but just to be sure...
    tangent_vector_norm = np.sqrt(x_Cdev ** 2 + y_Cdev ** 2)
    #  lateral and lag error are defined projecting the vector from the robot's position to the "closest" point on the
    #  path on to the tangent-normal to path reference frame
    err_lag_squared = ((x[0] - Cx_out) * x_Cdev / tangent_vector_norm + (x[1] - Cy_out) * y_Cdev / tangent_vector_norm) ** 2
    err_lat_squared = ((x[0] - Cx_out) * -y_Cdev / tangent_vector_norm + (x[1] - Cy_out) * x_Cdev / tangent_vector_norm) ** 2

    # cost function
    s_to_theta_conversion_factor = np.sqrt((scale_x_L * wx_L * np.sin(wx_L * theta_out)) ** 2 + (scale_y_L * wy_L * np.cos(wy_L * theta_out)) ** 2)
    j = ((x[3] / s_to_theta_conversion_factor) - V_target) ** 2 + q1 * err_lat_squared + q2 * err_lag_squared + q3 * u[0] ** 2 + q4 * u[1] ** 2
    return j














# --  functions needed to run the solver (to set up the problem to be solved) --

def find_s_of_closest_point_on_global_path(x_y_state, s_vals_global_path, x_vals_original_path, y_vals_original_path, previous_index, estimated_ds):
    min_ds = np.min(np.diff(s_vals_global_path))
    estimated_index_jumps = math.ceil(estimated_ds / min_ds)
    minimum_index_jumps = math.ceil(0.1 / min_ds)

    # in case the vehicle is still, ensure a minimum search space to account for localization error
    if estimated_index_jumps < minimum_index_jumps:
        estimated_index_jumps = minimum_index_jumps

    Delta_indexes = estimated_index_jumps * 3

    start_i = previous_index - Delta_indexes
    finish_i = previous_index + Delta_indexes

    #check if start_i is negative and finish_i is positive
    if start_i < 0:
        s_search_vector = np.concatenate((s_vals_global_path[start_i:], s_vals_global_path[: finish_i]), axis=0)
        x_search_vector = np.concatenate((x_vals_original_path[start_i:], x_vals_original_path[: finish_i]), axis=0)
        y_search_vector = np.concatenate((y_vals_original_path[start_i:], y_vals_original_path[: finish_i]), axis=0)

    elif finish_i > s_vals_global_path.size:
        s_search_vector = np.concatenate((s_vals_global_path[start_i:], s_vals_global_path[: finish_i - s_vals_global_path.size]), axis=0)
        x_search_vector = np.concatenate((x_vals_original_path[start_i:], x_vals_original_path[: finish_i - s_vals_global_path.size]), axis=0)
        y_search_vector = np.concatenate((y_vals_original_path[start_i:], y_vals_original_path[: finish_i - s_vals_global_path.size]), axis=0)
    else:
        s_search_vector = s_vals_global_path[start_i: finish_i]
        x_search_vector = x_vals_original_path[start_i: finish_i]
        y_search_vector = y_vals_original_path[start_i: finish_i]

    #remove the last value to avoid ambiguity since first and last value may be the same
    distances = np.zeros(s_search_vector.size)
    for ii in range(0, s_search_vector.size):
        distances[ii] = math.dist([x_search_vector[ii], y_search_vector[ii]], x_y_state[0:2])
    
    local_index = np.argmin(distances)
    #local_index = local_index[0]
    print(local_index)
    index = np.where(s_vals_global_path == s_search_vector[local_index])
    print(index)
    index = index[0]
    print(index)
    index = index[0]
    print(index)

    s = float(s_vals_global_path[index])
    return s, index

def evaluate_local_path_Chebyshev_coefficients_manuel(s, x_of_s, y_of_s, Ds_forecast, s_vals_global_path, loop_path, Cheby_data_points, theta_of_s):

    s_subpath_cheby_for_xy_data_generation = np.linspace(s, s + Ds_forecast, Cheby_data_points)
    s_subpath_for_fitting_operation = np.linspace(s, s + Ds_forecast, Cheby_data_points)

    if loop_path and (s + Ds_forecast) > s_vals_global_path[-1]:

        s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation > s_vals_global_path[-1]] = s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation > s_vals_global_path[-1]] - s_vals_global_path[-1]

    x_data_points_Cheby = x_of_s(s_subpath_cheby_for_xy_data_generation)
    y_data_points_Cheby = y_of_s(s_subpath_cheby_for_xy_data_generation)
    theta_data_points_Cheby = theta_of_s(s_subpath_cheby_for_xy_data_generation)

    #define s values of the local path
    #s_subpath_cheby = np.linspace(s, s + Ds_forecast, Cheby_data_points)

    # evaluate cheby coeffs
    # the number of basis should match the number of parameters allocated for Chebishev coefficients in the solver
    coeffx = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, x_data_points_Cheby, 9)
    coeffy = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, y_data_points_Cheby, 9)
    coefftheta_vec = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, theta_data_points_Cheby, 9)

    return coeffx, coeffy, coefftheta_vec





