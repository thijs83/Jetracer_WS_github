import numpy as np
import sys
import time
sys.path.insert(0, '/home/lorenzo/OneDrive/PhD/ubuntu_stuff/Forces_pro_extracted/forces_pro_client')
import forcespro.nlp
import casadi
import math
from scipy.optimize import minimize, Bounds
import warnings #turn off low rank waring from chebfit
warnings.simplefilter('ignore', np.RankWarning)



def produce_s_local_path(s,Ds_forecast,Cheby_data_points,s_vals_global_path,loop_path):
    Ds_backwards = 0.1 * Ds_forecast
    s_subpath_cheby_for_xy_data_generation = np.linspace(s - Ds_backwards, s + Ds_forecast, Cheby_data_points)
    s_subpath_for_fitting_operation = np.linspace(s - Ds_backwards, s + Ds_forecast, Cheby_data_points)

    if loop_path:
        #now perform wrapping if necessary
        if (s + Ds_forecast) > s_vals_global_path[-1]:
            #wrapping the excess
            s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation > s_vals_global_path[-1]] = \
            s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation > s_vals_global_path[-1]] - \
            s_vals_global_path[-1]
        if (s - Ds_backwards) < s_vals_global_path[0]:
            #wrapping before 0
            s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation < s_vals_global_path[0]] = \
            s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation < s_vals_global_path[0]] + \
            s_vals_global_path[-1]
    else:
        #loop is false
        #cap upper value
        s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation > s_vals_global_path[-1]] = \
        s_vals_global_path[-1]
        #cap lower value
        s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation < s_vals_global_path[0]] = \
        s_vals_global_path[-1]
        #set both s_vectors to be equal
        s_subpath_for_fitting_operation = s_subpath_cheby_for_xy_data_generation
    return s_subpath_cheby_for_xy_data_generation, s_subpath_for_fitting_operation





def evaluate_local_path_Chebyshev_coefficients_high_order_cheby(s, x_of_s, y_of_s, Ds_forecast, s_vals_global_path, loop_path, Cheby_data_points):
    #allow for some track behind the vehicle
    s_subpath_cheby_for_xy_data_generation, s_subpath_for_fitting_operation = produce_s_local_path(s, Ds_forecast, Cheby_data_points, s_vals_global_path, loop_path)

    # # wrapping s values around [0 s_max_global], only for x_y data generation
    # if loop_path and (s + Ds_forecast) > s_vals_global_path[-1]:
    #     s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation > s_vals_global_path[-1]] = s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation > s_vals_global_path[-1]] - s_vals_global_path[-1]
    #


    x_data_points_Cheby = x_of_s(s_subpath_cheby_for_xy_data_generation)
    y_data_points_Cheby = y_of_s(s_subpath_cheby_for_xy_data_generation)

    #define s values of the local path
    #s_subpath_cheby = np.linspace(s, s + Ds_forecast, Cheby_data_points)

    # evaluate cheby coeffs
    # the number of basis should match the number of parameters allocated for Chebishev coefficients in the solver
    # add this to smooth rcond = 0.000001
    # adding weights to enforce better fitting closer to the current state
    weights = np.linspace(2, 1, s_subpath_for_fitting_operation.size)
    # coeffx = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, x_data_points_Cheby, 19, rcond = 0.00000000001, w = weights)
    # coeffy = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, y_data_points_Cheby, 19, rcond = 0.00000000001, w = weights)
    coeffx = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, x_data_points_Cheby, 19)
    coeffy = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, y_data_points_Cheby, 19)
    return coeffx, coeffy






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
    # maybe just get the index from the min operator

    local_index = np.argmin(distances)
    # check if the found minimum is on the boarder (indicating that the real minimum is outside of the search vector)

    #this offers some protection against failing the local search but it doesn't fix all of the possible problems
    #for example if pth loops back (like a bean shape)
    # then you can still get an error (If you have lane boundary information then you colud put a check on the actual value of the min)
    if local_index == 0 or local_index == s_search_vector.size-1:
        # print('search vector was not long enough, doing search on full path')
        distances_2 = np.zeros(s_vals_global_path.size)
        for ii in range(0, s_vals_global_path.size):
            distances_2[ii] = math.dist([x_vals_original_path[ii], y_vals_original_path[ii]], x_y_state[0:2])
        index = np.argmin(distances_2)
    else:
        index = np.where(s_vals_global_path == s_search_vector[local_index])
        # this seemeingly reduntant steps are to extract an int from the "where" operand
        index = index[0]
        index = index[0]


    s = float(s_vals_global_path[index])
    return s, index


def straight(xlims, ylims, n_checkppoints):
    Checkpoints_x = np.linspace(xlims[0], xlims[1], n_checkppoints)
    Checkpoints_y = np.linspace(ylims[0], ylims[1], n_checkppoints)
    return Checkpoints_x, Checkpoints_y

def curve(centre, R,theta_extremes, n_checkppoints):
    theta_init = np.pi * theta_extremes[0]
    theta_end = np.pi * theta_extremes[1]
    theta_vec = np.linspace(theta_init, theta_end, n_checkppoints)
    Checkpoints_x = centre[0] + R * np.cos(theta_vec)
    Checkpoints_y = centre[1] + R * np.sin(theta_vec)
    return Checkpoints_x, Checkpoints_y




def produce_track(choice,n_checkppoints):
    if choice == 'savoiardo':

        R = 0.8  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.82
        theta_init2 = np.pi * -0.5
        theta_end2 = np.pi * 0.5
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkppoints)
        theta_init4 = np.pi * 0.5
        theta_end4 = np.pi * 1.5
        theta_vec4 = np.linspace(theta_init4, theta_end4, n_checkppoints)
        Checkpoints_x1 = np.linspace(- 1.5 * R, 1.5 * R, n_checkppoints)
        Checkpoints_y1 = np.zeros(n_checkppoints) - R
        Checkpoints_x2 = 1.5 * R + R * np.cos(theta_vec2)
        Checkpoints_y2 = R * np.sin(theta_vec2)
        Checkpoints_x3 = np.linspace(1.5 * R, -1.5*R, n_checkppoints)
        Checkpoints_y3 = R * np.ones(n_checkppoints)
        Checkpoints_x4 = -1.5* R + R * np.cos(theta_vec4)
        Checkpoints_y4 = R * np.sin(theta_vec4)

        Checkpoints_x = [*Checkpoints_x2[0:n_checkppoints - 1],
                         *Checkpoints_x3[0:n_checkppoints - 1], *Checkpoints_x4[0:n_checkppoints - 1], *Checkpoints_x1[0:n_checkppoints]]
        Checkpoints_y = [*Checkpoints_y2[0:n_checkppoints - 1],
                         *Checkpoints_y3[0:n_checkppoints - 1], *Checkpoints_y4[0:n_checkppoints -1], *Checkpoints_y1[0:n_checkppoints]]

    elif choice == 'double_donut':

        R = 1  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        theta_init1 = np.pi * -0.5
        theta_end1 = np.pi * 0.0
        theta_vec1 = np.linspace(theta_init1, theta_end1, n_checkppoints)
        theta_init2 = np.pi * 1
        theta_end2 = np.pi * -1
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkppoints)
        theta_init3 = np.pi * 0
        theta_end3 = np.pi * 1.5
        theta_vec3 = np.linspace(theta_init3, theta_end3, n_checkppoints)

        Checkpoints_x1 = - R + R * np.cos(theta_vec1)
        Checkpoints_y1 =  + R * np.sin(theta_vec1)
        Checkpoints_x2 = + R + R * np.cos(theta_vec2)
        Checkpoints_y2 =  + R * np.sin(theta_vec2)
        Checkpoints_x3 = - R + R * np.cos(theta_vec3)
        Checkpoints_y3 = + R * np.sin(theta_vec3)

        Checkpoints_x = [*Checkpoints_x1[0:n_checkppoints - 1], *Checkpoints_x2[0:n_checkppoints - 1],
                         *Checkpoints_x3[0:n_checkppoints]]
        Checkpoints_y = [*Checkpoints_y1[0:n_checkppoints - 1], *Checkpoints_y2[0:n_checkppoints - 1],
                         *Checkpoints_y3[0:n_checkppoints]]

    elif choice == 'straight_line':
        Checkpoints_x = np.linspace(0, 100, n_checkppoints)
        Checkpoints_y = np.zeros(n_checkppoints)

    elif choice == 'savoiardo_saturate_steering':
        R = 0.3  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        theta_init2 = np.pi * -0.5
        theta_end2 = np.pi * 0.5
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkppoints)
        theta_init4 = np.pi * 0.5
        theta_end4 = np.pi * 1.5
        theta_vec4 = np.linspace(theta_init4, theta_end4, n_checkppoints)
        Checkpoints_x1 = np.linspace(- 4.5 * R, 4.5 * R, n_checkppoints)
        Checkpoints_y1 = np.zeros(n_checkppoints) - R
        Checkpoints_x2 = 4.5 * R + R * np.cos(theta_vec2)
        Checkpoints_y2 = R * np.sin(theta_vec2)
        Checkpoints_x3 = np.linspace(4.5 * R, -4.5*R, n_checkppoints)
        Checkpoints_y3 = R * np.ones(n_checkppoints)
        Checkpoints_x4 = -4.5* R + R * np.cos(theta_vec4)
        Checkpoints_y4 = R * np.sin(theta_vec4)

        Checkpoints_x = [*Checkpoints_x2[0:n_checkppoints - 1],
                         *Checkpoints_x3[0:n_checkppoints - 1], *Checkpoints_x4[0:n_checkppoints - 1], *Checkpoints_x1[0:n_checkppoints ]]
        Checkpoints_y = [*Checkpoints_y2[0:n_checkppoints - 1],
                         *Checkpoints_y3[0:n_checkppoints - 1], *Checkpoints_y4[0:n_checkppoints], *Checkpoints_y1[0:n_checkppoints - 1]]

    elif choice == 'racetrack_saturate_steering':

        R = 0.8  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        theta_init2 = np.pi * -0.5
        theta_end2 = np.pi * 0.5
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkppoints)

        theta_init3 = np.pi * 1.5
        theta_end3 = np.pi * 0.5
        theta_vec3 = np.linspace(theta_init3, theta_end3, n_checkppoints)

        theta_init6 = np.pi * 0.5
        theta_end6 = np.pi * 1.0
        theta_vec6 = np.linspace(theta_init6, theta_end6, n_checkppoints)

        theta_init8 = np.pi * -1.0
        theta_end8 = np.pi * 0.0
        theta_vec8 = np.linspace(theta_init8, theta_end8, n_checkppoints)

        theta_init10 = np.pi * 1.0
        theta_end10 = np.pi * 0.0
        theta_vec10 = np.linspace(theta_init10, theta_end10, n_checkppoints)

        theta_init12 = np.pi * -1.0
        theta_end12 = np.pi * -0.5
        theta_vec12 = np.linspace(theta_init12, theta_end12, n_checkppoints)

        # DEFINED STARTING FROM  START POINT AND THEN SHIFT IT LATER IF NEEDED
        Checkpoints_x1 = np.linspace(0, 3*R, n_checkppoints)
        Checkpoints_y1 = np.zeros(n_checkppoints) - R

        Checkpoints_x2 = + 3*R + R * np.cos(theta_vec2)
        Checkpoints_y2 =  R * np.sin(theta_vec2)

        Checkpoints_x3 = 3*R + R * np.cos(theta_vec3)
        Checkpoints_y3 = 2*R +R * np.sin(theta_vec3)

        Checkpoints_x4 = + 3*R + R * np.cos(theta_vec2)
        Checkpoints_y4 = + 4*R + R * np.sin(theta_vec2)

        Checkpoints_x5 = np.linspace(3*R, -3*R, n_checkppoints)
        Checkpoints_y5 = np.zeros(n_checkppoints) + 5*R

        Checkpoints_x6 = - 3*R + 2*R * np.cos(theta_vec6)
        Checkpoints_y6 = + 3*R + 2*R * np.sin(theta_vec6)

        Checkpoints_x7 = np.zeros(n_checkppoints) - 5*R
        Checkpoints_y7 = np.linspace(3 * R, 0, n_checkppoints)

        Checkpoints_x8 = - 4.5 * R + 0.5 * R * np.cos(theta_vec8)
        Checkpoints_y8 = + 0.5 * R * np.sin(theta_vec8)

        Checkpoints_x9 = np.zeros(n_checkppoints) - 4*R
        Checkpoints_y9 = np.linspace(0, 2*R, n_checkppoints)

        Checkpoints_x10 = - 3.5 * R + 0.5 * R * np.cos(theta_vec10)
        Checkpoints_y10 = + 2 * R + 0.5 * R * np.sin(theta_vec10)

        Checkpoints_x11 = np.zeros(n_checkppoints) - 3*R
        Checkpoints_y11 = np.linspace(2 * R, 0, n_checkppoints)

        Checkpoints_x12 = - 2.5 * R + 0.5 * R * np.cos(theta_vec8)
        Checkpoints_y12 = + 0.5 * R * np.sin(theta_vec8)

        Checkpoints_x13 = np.zeros(n_checkppoints) - 2*R
        Checkpoints_y13 = np.linspace(0, 2*R, n_checkppoints)

        Checkpoints_x14 = - 1.5 * R + 0.5 * R * np.cos(theta_vec10)
        Checkpoints_y14 = + 2 * R + 0.5 * R * np.sin(theta_vec10)

        Checkpoints_x15 = np.zeros(n_checkppoints) - 1 * R
        Checkpoints_y15 = np.linspace(2 * R, 0, n_checkppoints)

        Checkpoints_x16 = + R * np.cos(theta_vec12)
        Checkpoints_y16 = + R * np.sin(theta_vec12)

        Checkpoints_x = [*Checkpoints_x1[0:n_checkppoints - 1],
                         *Checkpoints_x2[0:n_checkppoints - 1],
                         *Checkpoints_x3[0:n_checkppoints - 1],
                         *Checkpoints_x4[0:n_checkppoints - 1],
                         *Checkpoints_x5[0:n_checkppoints - 1],
                         *Checkpoints_x6[0:n_checkppoints - 1],
                         *Checkpoints_x7[0:n_checkppoints - 1],
                         *Checkpoints_x8[0:n_checkppoints - 1],
                         *Checkpoints_x9[0:n_checkppoints - 1],
                         *Checkpoints_x10[0:n_checkppoints - 1],
                         *Checkpoints_x11[0:n_checkppoints - 1],
                         *Checkpoints_x12[0:n_checkppoints - 1],
                         *Checkpoints_x13[0:n_checkppoints - 1],
                         *Checkpoints_x14[0:n_checkppoints - 1],
                         *Checkpoints_x15[0:n_checkppoints - 1],
                         *Checkpoints_x16[0:n_checkppoints]]

        Checkpoints_y = [*Checkpoints_y1[0:n_checkppoints - 1],
                         *Checkpoints_y2[0:n_checkppoints - 1],
                         *Checkpoints_y3[0:n_checkppoints - 1],
                         *Checkpoints_y4[0:n_checkppoints - 1],
                         *Checkpoints_y5[0:n_checkppoints - 1],
                         *Checkpoints_y6[0:n_checkppoints - 1],
                         *Checkpoints_y7[0:n_checkppoints - 1],
                         *Checkpoints_y8[0:n_checkppoints - 1],
                         *Checkpoints_y9[0:n_checkppoints - 1],
                         *Checkpoints_y10[0:n_checkppoints - 1],
                         *Checkpoints_y11[0:n_checkppoints - 1],
                         *Checkpoints_y12[0:n_checkppoints - 1],
                         *Checkpoints_y13[0:n_checkppoints - 1],
                         *Checkpoints_y14[0:n_checkppoints - 1],
                         *Checkpoints_y15[0:n_checkppoints - 1],
                         *Checkpoints_y16[0:n_checkppoints]]


    elif choice == 'circle':
        n_checkppoints = 4 * n_checkppoints
        R = 0.3  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.82
        theta_init = np.pi * -0.5
        theta_end = np.pi * 1.5
        theta_vec = np.linspace(theta_init, theta_end, n_checkppoints)
        Checkpoints_x = R * np.cos(theta_vec)
        Checkpoints_y = R * np.sin(theta_vec)

    elif choice == 'gain_sweep_track':
        R = 0.4  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        straight_bit_half_length = 2.3

        theta_init2 = np.pi * -0.5
        theta_end2 = np.pi * 0.5
        theta_vec2 = np.linspace(theta_init2, theta_end2, n_checkppoints)
        theta_init4 = np.pi * 0.5
        theta_end4 = np.pi * 1.5
        theta_vec4 = np.linspace(theta_init4, theta_end4, n_checkppoints)
        Checkpoints_x1 = np.linspace(- straight_bit_half_length, straight_bit_half_length, n_checkppoints)
        Checkpoints_y1 = np.zeros(n_checkppoints) - R
        Checkpoints_x2 = straight_bit_half_length + R * np.cos(theta_vec2)
        Checkpoints_y2 = R * np.sin(theta_vec2)
        Checkpoints_x3 = np.linspace(straight_bit_half_length, -straight_bit_half_length, n_checkppoints)
        Checkpoints_y3 = R * np.ones(n_checkppoints)
        Checkpoints_x4 = -straight_bit_half_length + R * np.cos(theta_vec4)
        Checkpoints_y4 = R * np.sin(theta_vec4)

        Checkpoints_x = [*Checkpoints_x1[0:n_checkppoints - 1], *Checkpoints_x2[0:n_checkppoints - 1],
                         *Checkpoints_x3[0:n_checkppoints - 1], *Checkpoints_x4[0:n_checkppoints]]
        Checkpoints_y = [*Checkpoints_y1[0:n_checkppoints - 1], *Checkpoints_y2[0:n_checkppoints - 1],
                         *Checkpoints_y3[0:n_checkppoints - 1], *Checkpoints_y4[0:n_checkppoints]]

    elif choice == 'gain_sweep_track_2':
        R = 0.49  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48
        straight_bit = 0.5

        Checkpoints_x1, Checkpoints_y1 = straight([0, straight_bit], [0, 0], n_checkppoints)
        Checkpoints_x2, Checkpoints_y2 = curve([straight_bit ,1 * R], R, [-0.5, 0.5], n_checkppoints)
        Checkpoints_x3, Checkpoints_y3 = straight([straight_bit, 0], [2*R, 2*R], n_checkppoints)
        Checkpoints_x4, Checkpoints_y4 = curve([0,3 * R], R, [1.5, 0.5], n_checkppoints)
        Checkpoints_x5, Checkpoints_y5 = straight([0, 10*straight_bit], [8*R, 8*R], 10*n_checkppoints)



        Checkpoints_y = [*Checkpoints_x1[0:n_checkppoints - 1],
                         *Checkpoints_x2[0:n_checkppoints - 1],
                         *Checkpoints_x3[0:n_checkppoints - 1],
                         *Checkpoints_x4[0:n_checkppoints - 1],
                         *Checkpoints_x1[0:n_checkppoints - 1],
                         *Checkpoints_x2[0:n_checkppoints - 1],
                         *Checkpoints_x3[0:n_checkppoints - 1],
                         *Checkpoints_x4[0:n_checkppoints - 1],
                         *Checkpoints_x5[0:]]

        Checkpoints_x = [*Checkpoints_y1[0:n_checkppoints - 1],
                         *Checkpoints_y2[0:n_checkppoints - 1],
                         *Checkpoints_y3[0:n_checkppoints - 1],
                         *Checkpoints_y4[0:n_checkppoints - 1],
                         *Checkpoints_y1[0:n_checkppoints - 1]+4*R,
                         *Checkpoints_y2[0:n_checkppoints - 1]+4*R,
                         *Checkpoints_y3[0:n_checkppoints - 1]+4*R,
                         *Checkpoints_y4[0:n_checkppoints - 1]+4*R,
                         *Checkpoints_y5[0:]]

    elif choice == 'racetrack_Lab':
        R = 0.5  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48

        # DEFINED STARTING FROM  START POINT AND THEN SHIFT IT LATER IF NEEDED


        Checkpoints_x1, Checkpoints_y1 = straight([-1+3*R, 1], [0, 0], n_checkppoints)

        Checkpoints_x2, Checkpoints_y2 = curve([1, 2*R], 2 * R, [-0.5, 0.5], n_checkppoints)

        Checkpoints_x3, Checkpoints_y3 = straight([1, -1], [4*R, 4*R], n_checkppoints)

        Checkpoints_x4, Checkpoints_y4 = curve([-1, 2.5*R], 1.5 * R, [0.5, 1.5], n_checkppoints)

        Checkpoints_x5, Checkpoints_y5 = straight([-1, 1], [R, R], n_checkppoints)

        Checkpoints_x6, Checkpoints_y6 = curve([1, 2 * R], 1 * R, [-0.5, 0.5], n_checkppoints)

        Checkpoints_x7, Checkpoints_y7 = straight([1, -1], [3 * R, 2 * R], n_checkppoints)

        Checkpoints_x8, Checkpoints_y8 = curve([-1, 3 * R], 1 * R, [1.5, 0], n_checkppoints)

        Checkpoints_x9, Checkpoints_y9 = straight([-1 + R, -1 + R], [3 * R, 2 * R], n_checkppoints)

        Checkpoints_x10, Checkpoints_y10 = curve([-1 + 3*R, 2 * R], 2 * R, [-1.0, -0.5], n_checkppoints)

        Checkpoints_x = [*Checkpoints_x1[0:n_checkppoints - 1],
                         *Checkpoints_x2[0:n_checkppoints - 1],
                         *Checkpoints_x3[0:n_checkppoints - 1],
                         *Checkpoints_x4[0:n_checkppoints - 1],
                         *Checkpoints_x5[0:n_checkppoints - 1],
                         *Checkpoints_x6[0:n_checkppoints - 1],
                         *Checkpoints_x7[0:n_checkppoints - 1],
                         *Checkpoints_x8[0:n_checkppoints - 1],
                         *Checkpoints_x9[0:n_checkppoints - 1],
                         *Checkpoints_x10[0:n_checkppoints]]
        y_shift = 2*R  #towards the bottom
        Checkpoints_y = [*Checkpoints_y1[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y2[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y3[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y4[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y5[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y6[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y7[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y8[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y9[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y10[0:n_checkppoints] - y_shift]

    elif choice == 'racetrack_Lab_safety_GP':
        R = 1  # as a reference the max radius of curvature is  R = L/tan(delta) = 0.48

        # DEFINED STARTING FROM  START POINT AND THEN SHIFT IT LATER IF NEEDED
        


        Checkpoints_x1, Checkpoints_y1 = straight([-2, 2], [-1.5, -1.5], n_checkppoints)

        Checkpoints_x2, Checkpoints_y2 = curve([2, -0.5], R, [-0.5, 0], n_checkppoints)

        Checkpoints_x3, Checkpoints_y3 = straight([3, 3], [-0.5, 0.5], n_checkppoints)

        Checkpoints_x4, Checkpoints_y4 = curve([2, 0.5], R, [0, 1], n_checkppoints)

        Checkpoints_x5, Checkpoints_y5 = curve([0, 0.5], R, [0, -1], n_checkppoints)

        Checkpoints_x6, Checkpoints_y6 = curve([-2, 0.5], R, [0, 1], n_checkppoints)

        Checkpoints_x7, Checkpoints_y7 = straight([-3, -3], [0.5, -0.5], n_checkppoints)

        Checkpoints_x8, Checkpoints_y8 = curve([-2, -0.5], R, [-1, -0.5], n_checkppoints)

        #Checkpoints_x9, Checkpoints_y9 = curve([-1.5, -1], 0.5*R, [1, 0], n_checkppoints)

        #Checkpoints_x10, Checkpoints_y10 = curve([-0.5, -1], 0.5*R, [-1, -0.5], n_checkppoints)
        x_shift = 0.5
        Checkpoints_x = [*Checkpoints_x1[0:n_checkppoints - 1] + x_shift,
                         *Checkpoints_x2[0:n_checkppoints - 1]+ x_shift,
                         *Checkpoints_x3[0:n_checkppoints - 1]+ x_shift,
                         *Checkpoints_x4[0:n_checkppoints - 1]+ x_shift,
                         *Checkpoints_x5[0:n_checkppoints - 1]+ x_shift,
                         *Checkpoints_x6[0:n_checkppoints - 1]+ x_shift,
                         *Checkpoints_x7[0:n_checkppoints - 1]+ x_shift,
                         *Checkpoints_x8[0:n_checkppoints]+ x_shift]
        y_shift = 0  #towards the bottom
        Checkpoints_y = [*Checkpoints_y1[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y2[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y3[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y4[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y5[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y6[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y7[0:n_checkppoints - 1] - y_shift,
                         *Checkpoints_y8[0:n_checkppoints] - y_shift]


    else:
        print('Invalid choice of track:')
        print('You selected: ', choice)


    return Checkpoints_x, Checkpoints_y




def load_GP_parameters_from_file(abs_path_parameters_folder):
    print('loading GP_parameters from: ')
    print(abs_path_parameters_folder)

    # X data
    x_data_vec_vx = np.load(abs_path_parameters_folder + "/forces_params_X_data_vx.npy")
    x_data_vec_vy = np.load(abs_path_parameters_folder + "/forces_params_X_data_vy.npy")
    x_data_vec_w = np.load(abs_path_parameters_folder + "/forces_params_X_data_w.npy")
    # covar inducing points for Orthogonally decoupled SVGP
    try:
        x_data_vec_vx_cov = np.load(abs_path_parameters_folder + "/forces_params_X_data_vx_cov.npy")
        x_data_vec_vy_cov = np.load(abs_path_parameters_folder + "/forces_params_X_data_vy_cov.npy")
        x_data_vec_w_cov = np.load(abs_path_parameters_folder + "/forces_params_X_data_w_cov.npy")
    except:
        #return empty vectors if there are no covariance inducing points (thus you are loading a standard SVGP)
        x_data_vec_vx_cov = []
        x_data_vec_vy_cov = []
        x_data_vec_w_cov = []


    # vx
    file_forces_params_Delta_vx_model_params = abs_path_parameters_folder + "/forces_params_Delta_vx_model_params.npy"
    file_forces_params_Delta_vx_right_vec = abs_path_parameters_folder + "/file_forces_params_Delta_vx_right_vec.npy"
    GP_params_Delta_vx = np.load(file_forces_params_Delta_vx_model_params)
    outputscale_Delta_vx = GP_params_Delta_vx[0]
    lengthscales_Delta_vx = GP_params_Delta_vx[1:]
    right_vec_block_Delta_vx = np.load(file_forces_params_Delta_vx_right_vec)
    central_mat_Delta_vx_vector = np.load(abs_path_parameters_folder + "/file_forces_params_central_matrix_vx_vec.npy")

    # vy
    file_forces_params_Delta_vy_model_params = abs_path_parameters_folder + "/forces_params_Delta_vy_model_params.npy"
    file_forces_params_Delta_vy_right_vec = abs_path_parameters_folder + "/file_forces_params_Delta_vy_right_vec.npy"
    GP_params_Delta_vy = np.load(file_forces_params_Delta_vy_model_params)
    outputscale_Delta_vy = GP_params_Delta_vy[0]
    lengthscales_Delta_vy = GP_params_Delta_vy[1:]
    right_vec_block_Delta_vy = np.load(file_forces_params_Delta_vy_right_vec)
    central_mat_Delta_vy_vector = np.load(abs_path_parameters_folder + "/file_forces_params_central_matrix_vy_vec.npy")

    # w
    file_forces_params_Delta_w_model_params = abs_path_parameters_folder + "/forces_params_Delta_w_model_params.npy"
    file_forces_params_Delta_w_right_vec = abs_path_parameters_folder + "/file_forces_params_Delta_w_right_vec.npy"
    GP_params_Delta_w = np.load(file_forces_params_Delta_w_model_params)
    outputscale_Delta_w = GP_params_Delta_w[0]
    lengthscales_Delta_w = GP_params_Delta_w[1:]
    right_vec_block_Delta_w = np.load(file_forces_params_Delta_w_right_vec)
    central_mat_Delta_w_vector = np.load(abs_path_parameters_folder + "/file_forces_params_central_matrix_w_vec.npy")


    return x_data_vec_vx, x_data_vec_vy, x_data_vec_w, outputscale_Delta_vx, lengthscales_Delta_vx, right_vec_block_Delta_vx,\
                       outputscale_Delta_vy, lengthscales_Delta_vy, right_vec_block_Delta_vy, \
                       outputscale_Delta_w, lengthscales_Delta_w, right_vec_block_Delta_w,\
                       central_mat_Delta_vx_vector, central_mat_Delta_vy_vector, central_mat_Delta_w_vector,\
                        x_data_vec_vx_cov, x_data_vec_vy_cov, x_data_vec_w_cov