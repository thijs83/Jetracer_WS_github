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



def unpack_parameters_cheby(p):
    #NOTE THAT: dynamic obstacle position and radius must now match the number of stages since it is the only way to
    # give force a time dependent function
    a = p[0]
    b = p[1]
    coeffx_vec = p[2:12]  # ACTULLY ONLY TRUE HERE FOR SOME REASON (MAYBE CASADI INDEXING??)
                          # python does not inlcude the last index so 1:3 is actually 2:3 (in matlab indexing)
    coeffy_vec = p[12:22]
    V_target = p[22]
    L = p[23]
    C = p[24]
    a_th = p[25]
    b_th = p[26]
    dt = p[27]
    xo = p[28]
    yo = p[29]
    ro = p[30]
    # must match the number of stages
    dyn_ob_traj_x = p[31:46]
    dyn_ob_traj_y = p[46:61]
    dyn_ob_traj_r = p[61:76]
    l_shift = p[76]
    l_width = p[77]
    q1 = p[78]
    q2 = p[79]
    q3 = p[80]
    q4 = p[81]
    S1 = p[82]
    S2 = p[83]
    S3 = p[84]
    S4 = p[85]
    S5 = p[86]
    S6 = p[87]
    S7 = p[88]
    S8 = p[89]
    S9 = p[90]
    return a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9

def unpack_parameters_cheby_high_order(p):
    #NOTE THAT: dynamic obstacle position and radius must now match the number of stages since it is the only way to
    # give force a time dependent function
    a = p[0]
    b = p[1]
    coeffx_vec = p[2:22]  # ACTUALLY ONLY TRUE HERE FOR SOME REASON (MAYBE CASADI INDEXING??)
                          # python does not inlcude the last index so 1:3 is actually 2:3 (in matlab indexing)
    coeffy_vec = p[22:42]
    V_target = p[42]
    L = p[43]
    C = p[44]
    a_th = p[45]
    b_th = p[46]
    dt = p[47]
    xo = p[48]
    yo = p[49]
    ro = p[50]
    # must match the number of stages
    dyn_ob_traj_x = p[51:81]
    dyn_ob_traj_y = p[81:111]
    dyn_ob_traj_r = p[111:141]
    l_shift = p[141]
    l_width = p[142]
    q1 = p[143]
    q2 = p[144]
    q3 = p[145]
    q4 = p[146]
    S1 = p[147]
    S2 = p[148]
    S3 = p[149]
    S4 = p[150]
    S5 = p[151]
    S6 = p[152]
    S7 = p[153]
    S8 = p[154]
    S9 = p[155]
    return a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9



# for improved s_dot formulation

def evaluate_spline_quantities_cheby(x, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)

    # x = [x, y, eta, v, s]
    s = x[4]
    #in matlab the coefficients were referred to the standard base so -1<x<1, but here it seems that they are referred
    # to the range [a b] already

    #t = (2 / (b - a)) * s - ((a + b) / (b - a))
    #dxdy = (2 / (b - a))
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

    x_Cdev2 = x_T0_dev2 + x_T1_dev2 + x_T2_dev2 + x_T3_dev2 + x_T4_dev2 + x_T5_dev2 + x_T6_dev2 + x_T7_dev2 + x_T8_dev2 + x_T9_dev2

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

    y_Cdev2 = y_T0_dev2 + y_T1_dev2 + y_T2_dev2 + y_T3_dev2 + y_T4_dev2 + y_T5_dev2 + y_T6_dev2 + y_T7_dev2 + y_T8_dev2 + y_T9_dev2

    # evaluate interesting quantities(using offset for numerical reasons)
    # k_squared = ((x_Cdev2) ** 2 + (y_Cdev2) ** 2)
    # dummy = (x[0] - Cx_out) * x_Cdev2 + (x[1] - Cy_out) * y_Cdev2
    # projection_ratio = 1 / (1 - k_squared * dummy)

    k = np.sqrt((x_Cdev2) ** 2 + (y_Cdev2) ** 2 + 0.0001) #offset ensures R_max = 100
    #n_offset = (x[0] - Cx_out) * x_Cdev2 + (x[1] - Cy_out) * y_Cdev2
    projection_ratio = 1 / (1 - ((x[0] - Cx_out) * x_Cdev2 + (x[1] - Cy_out) * y_Cdev2))



    s_dot = x[3] * (np.cos(x[2]) * x_Cdev + np.sin(x[2]) * y_Cdev) * projection_ratio
    return Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2

def evaluate_spline_quantities_cheby_high_order(x, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby_high_order(p)

    # x = [x, y, eta, v, s]
    s = x[4]
    #in matlab the coefficients were referred to the standard base so -1<x<1, but here it seems that they are referred
    # to the range [a b] already

    #t = (2 / (b - a)) * s - ((a + b) / (b - a))
    #dxdy = (2 / (b - a))
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
    x_T10 = coeffx_vec[10] * (50 * t ** 2 - 400 * t ** 4 + 1120 * t ** 6 - 1280 * t ** 8 + 512 * t ** 10 - 1)
    x_T10_dev = coeffx_vec[10] * (dxdy) * (100 * t - 1600 * t ** 3 + 6720 * t ** 5 - 10240 * t ** 7 + 5120 * t ** 9)
    x_T10_dev2 = coeffx_vec[10] * ((dxdy) ** 2) * (33600 * t ** 4 - 4800 * t ** 2 - 71680 * t ** 6 + 46080 * t ** 8 + 100)
    x_T11 = coeffx_vec[11] * (220 * t ** 3 - 11 * t - 1232 * t ** 5 + 2816 * t ** 7 - 2816 * t ** 9 + 1024 * t ** 11)
    x_T11_dev = coeffx_vec[11] * (dxdy) * (
                660 * t ** 2 - 6160 * t ** 4 + 19712 * t ** 6 - 25344 * t ** 8 + 11264 * t ** 10 - 11)
    x_T11_dev2 = coeffx_vec[11] * ((dxdy) ** 2) * (
                1320 * t - 24640 * t ** 3 + 118272 * t ** 5 - 202752 * t ** 7 + 112640 * t ** 9)
    x_T12 = coeffx_vec[12] * (
                840 * t ** 4 - 72 * t ** 2 - 3584 * t ** 6 + 6912 * t ** 8 - 6144 * t ** 10 + 2048 * t ** 12 + 1)
    x_T12_dev = coeffx_vec[12] * (dxdy) * (
                3360 * t ** 3 - 144 * t - 21504 * t ** 5 + 55296 * t ** 7 - 61440 * t ** 9 + 24576 * t ** 11)
    x_T12_dev2 = coeffx_vec[12] * ((dxdy) ** 2) * (
                10080 * t ** 2 - 107520 * t ** 4 + 387072 * t ** 6 - 552960 * t ** 8 + 270336 * t ** 10 - 144)
    x_T13 = coeffx_vec[13] * (
                13 * t - 364 * t ** 3 + 2912 * t ** 5 - 9984 * t ** 7 + 16640 * t ** 9 - 13312 * t ** 11 + 4096 * t ** 13)
    x_T13_dev = coeffx_vec[13] * (dxdy) * (
                14560 * t ** 4 - 1092 * t ** 2 - 69888 * t ** 6 + 149760 * t ** 8 - 146432 * t ** 10 + 53248 * t ** 12 + 13)
    x_T13_dev2 = coeffx_vec[13] * ((dxdy) ** 2) * (
                58240 * t ** 3 - 2184 * t - 419328 * t ** 5 + 1198080 * t ** 7 - 1464320 * t ** 9 + 638976 * t ** 11)
    x_T14 = coeffx_vec[14] * (
                98 * t ** 2 - 1568 * t ** 4 + 9408 * t ** 6 - 26880 * t ** 8 + 39424 * t ** 10 - 28672 * t ** 12 + 8192 * t ** 14 - 1)
    x_T14_dev = coeffx_vec[14] * (dxdy) * (
                196 * t - 6272 * t ** 3 + 56448 * t ** 5 - 215040 * t ** 7 + 394240 * t ** 9 - 344064 * t ** 11 + 114688 * t ** 13)
    x_T14_dev2 = coeffx_vec[14] * ((dxdy) ** 2) * (
                282240 * t ** 4 - 18816 * t ** 2 - 1505280 * t ** 6 + 3548160 * t ** 8 - 3784704 * t ** 10 + 1490944 * t ** 12 + 196)
    x_T15 = coeffx_vec[15] * (
                560 * t ** 3 - 15 * t - 6048 * t ** 5 + 28800 * t ** 7 - 70400 * t ** 9 + 92160 * t ** 11 - 61440 * t ** 13 + 16384 * t ** 15)
    x_T15_dev = coeffx_vec[15] * (dxdy) * (
                1680 * t ** 2 - 30240 * t ** 4 + 201600 * t ** 6 - 633600 * t ** 8 + 1013760 * t ** 10 - 798720 * t ** 12 + 245760 * t ** 14 - 15)
    x_T15_dev2 = coeffx_vec[15] * ((dxdy) ** 2) * (
                3360 * t - 120960 * t ** 3 + 1209600 * t ** 5 - 5068800 * t ** 7 + 10137600 * t ** 9 - 9584640 * t ** 11 + 3440640 * t ** 13)
    x_T16 = coeffx_vec[16] * (
                2688 * t ** 4 - 128 * t ** 2 - 21504 * t ** 6 + 84480 * t ** 8 - 180224 * t ** 10 + 212992 * t ** 12 - 131072 * t ** 14 + 32768 * t ** 16 + 1)
    x_T16_dev = coeffx_vec[16] * (dxdy) * (
                10752 * t ** 3 - 256 * t - 129024 * t ** 5 + 675840 * t ** 7 - 1802240 * t ** 9 + 2555904 * t ** 11 - 1835008 * t ** 13 + 524288 * t ** 15)
    x_T16_dev2 = coeffx_vec[16] * ((dxdy) ** 2) * (
                32256 * t ** 2 - 645120 * t ** 4 + 4730880 * t ** 6 - 16220160 * t ** 8 + 28114944 * t ** 10 - 23855104 * t ** 12 + 7864320 * t ** 14 - 256)
    x_T17 = coeffx_vec[17] * (
                17 * t - 816 * t ** 3 + 11424 * t ** 5 - 71808 * t ** 7 + 239360 * t ** 9 - 452608 * t ** 11 + 487424 * t ** 13 - 278528 * t ** 15 + 65536 * t ** 17)
    x_T17_dev = coeffx_vec[17] * (dxdy) * (
                57120 * t ** 4 - 2448 * t ** 2 - 502656 * t ** 6 + 2154240 * t ** 8 - 4978688 * t ** 10 + 6336512 * t ** 12 - 4177920 * t ** 14 + 1114112 * t ** 16 + 17)
    x_T17_dev2 = coeffx_vec[17] * ((dxdy) ** 2) * (
                228480 * t ** 3 - 4896 * t - 3015936 * t ** 5 + 17233920 * t ** 7 - 49786880 * t ** 9 + 76038144 * t ** 11 - 58490880 * t ** 13 + 17825792 * t ** 15)
    x_T18 = coeffx_vec[18] * (
                162 * t ** 2 - 4320 * t ** 4 + 44352 * t ** 6 - 228096 * t ** 8 + 658944 * t ** 10 - 1118208 * t ** 12 + 1105920 * t ** 14 - 589824 * t ** 16 + 131072 * t ** 18 - 1)
    x_T18_dev = coeffx_vec[18] * (dxdy) * (
                324 * t - 17280 * t ** 3 + 266112 * t ** 5 - 1824768 * t ** 7 + 6589440 * t ** 9 - 13418496 * t ** 11 + 15482880 * t ** 13 - 9437184 * t ** 15 + 2359296 * t ** 17)
    x_T18_dev2 = coeffx_vec[18] * ((dxdy) ** 2) * (
                1330560 * t ** 4 - 51840 * t ** 2 - 12773376 * t ** 6 + 59304960 * t ** 8 - 147603456 * t ** 10 + 201277440 * t ** 12 - 141557760 * t ** 14 + 40108032 * t ** 16 + 324)
    x_T19 = coeffx_vec[19] * (
                1140 * t ** 3 - 19 * t - 20064 * t ** 5 + 160512 * t ** 7 - 695552 * t ** 9 + 1770496 * t ** 11 - 2723840 * t ** 13 + 2490368 * t ** 15 - 1245184 * t ** 17 + 262144 * t ** 19)
    x_T19_dev = coeffx_vec[19] * (dxdy) * (
                3420 * t ** 2 - 100320 * t ** 4 + 1123584 * t ** 6 - 6259968 * t ** 8 + 19475456 * t ** 10 - 35409920 * t ** 12 + 37355520 * t ** 14 - 21168128 * t ** 16 + 4980736 * t ** 18 - 19)
    x_T19_dev2 = coeffx_vec[19] * ((dxdy) ** 2) * (
                6840 * t - 401280 * t ** 3 + 6741504 * t ** 5 - 50079744 * t ** 7 + 194754560 * t ** 9 - 424919040 * t ** 11 + 522977280 * t ** 13 - 338690048 * t ** 15 + 89653248 * t ** 17)

    Cx_out = x_T0 + x_T1 + x_T2 + x_T3 + x_T4 + x_T5 + x_T6 + x_T7 + x_T8 + x_T9 + x_T10+ x_T11+x_T12+x_T13+x_T14+x_T15+x_T16+x_T17+x_T18+x_T19

    x_Cdev = x_T0_dev + x_T1_dev + x_T2_dev + x_T3_dev + x_T4_dev + x_T5_dev + x_T6_dev + x_T7_dev + x_T8_dev + x_T9_dev + x_T10_dev+x_T11_dev+x_T12_dev+x_T13_dev+x_T14_dev+x_T15_dev+x_T16_dev+x_T17_dev+x_T18_dev+x_T19_dev

    x_Cdev2 = x_T0_dev2 + x_T1_dev2 + x_T2_dev2 + x_T3_dev2 + x_T4_dev2 + x_T5_dev2 + x_T6_dev2 + x_T7_dev2 + x_T8_dev2 + x_T9_dev2 + x_T10_dev2+x_T11_dev2+x_T12_dev2+x_T13_dev2+x_T14_dev2+x_T15_dev2+x_T16_dev2+x_T17_dev2+x_T18_dev2+x_T19_dev2

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
    y_T10 = coeffy_vec[10] * (50 * t ** 2 - 400 * t ** 4 + 1120 * t ** 6 - 1280 * t ** 8 + 512 * t ** 10 - 1)
    y_T10_dev = coeffy_vec[10] * (dxdy) * (100 * t - 1600 * t ** 3 + 6720 * t ** 5 - 10240 * t ** 7 + 5120 * t ** 9)
    y_T10_dev2 = coeffy_vec[10] * ((dxdy) ** 2) * (33600 * t ** 4 - 4800 * t ** 2 - 71680 * t ** 6 + 46080 * t ** 8 + 100)
    y_T11 = coeffy_vec[11] * (220 * t ** 3 - 11 * t - 1232 * t ** 5 + 2816 * t ** 7 - 2816 * t ** 9 + 1024 * t ** 11)
    y_T11_dev = coeffy_vec[11] * (dxdy) * (
                660 * t ** 2 - 6160 * t ** 4 + 19712 * t ** 6 - 25344 * t ** 8 + 11264 * t ** 10 - 11)
    y_T11_dev2 = coeffy_vec[11] * ((dxdy) ** 2) * (
                1320 * t - 24640 * t ** 3 + 118272 * t ** 5 - 202752 * t ** 7 + 112640 * t ** 9)
    y_T12 = coeffy_vec[12] * (
                840 * t ** 4 - 72 * t ** 2 - 3584 * t ** 6 + 6912 * t ** 8 - 6144 * t ** 10 + 2048 * t ** 12 + 1)
    y_T12_dev = coeffy_vec[12] * (dxdy) * (
                3360 * t ** 3 - 144 * t - 21504 * t ** 5 + 55296 * t ** 7 - 61440 * t ** 9 + 24576 * t ** 11)
    y_T12_dev2 = coeffy_vec[12] * ((dxdy) ** 2) * (
                10080 * t ** 2 - 107520 * t ** 4 + 387072 * t ** 6 - 552960 * t ** 8 + 270336 * t ** 10 - 144)
    y_T13 = coeffy_vec[13] * (
                13 * t - 364 * t ** 3 + 2912 * t ** 5 - 9984 * t ** 7 + 16640 * t ** 9 - 13312 * t ** 11 + 4096 * t ** 13)
    y_T13_dev = coeffy_vec[13] * (dxdy) * (
                14560 * t ** 4 - 1092 * t ** 2 - 69888 * t ** 6 + 149760 * t ** 8 - 146432 * t ** 10 + 53248 * t ** 12 + 13)
    y_T13_dev2 = coeffy_vec[13] * ((dxdy) ** 2) * (
                58240 * t ** 3 - 2184 * t - 419328 * t ** 5 + 1198080 * t ** 7 - 1464320 * t ** 9 + 638976 * t ** 11)
    y_T14 = coeffy_vec[14] * (
                98 * t ** 2 - 1568 * t ** 4 + 9408 * t ** 6 - 26880 * t ** 8 + 39424 * t ** 10 - 28672 * t ** 12 + 8192 * t ** 14 - 1)
    y_T14_dev = coeffy_vec[14] * (dxdy) * (
                196 * t - 6272 * t ** 3 + 56448 * t ** 5 - 215040 * t ** 7 + 394240 * t ** 9 - 344064 * t ** 11 + 114688 * t ** 13)
    y_T14_dev2 = coeffy_vec[14] * ((dxdy) ** 2) * (
                282240 * t ** 4 - 18816 * t ** 2 - 1505280 * t ** 6 + 3548160 * t ** 8 - 3784704 * t ** 10 + 1490944 * t ** 12 + 196)
    y_T15 = coeffy_vec[15] * (
                560 * t ** 3 - 15 * t - 6048 * t ** 5 + 28800 * t ** 7 - 70400 * t ** 9 + 92160 * t ** 11 - 61440 * t ** 13 + 16384 * t ** 15)
    y_T15_dev = coeffy_vec[15] * (dxdy) * (
                1680 * t ** 2 - 30240 * t ** 4 + 201600 * t ** 6 - 633600 * t ** 8 + 1013760 * t ** 10 - 798720 * t ** 12 + 245760 * t ** 14 - 15)
    y_T15_dev2 = coeffy_vec[15] * ((dxdy) ** 2) * (
                3360 * t - 120960 * t ** 3 + 1209600 * t ** 5 - 5068800 * t ** 7 + 10137600 * t ** 9 - 9584640 * t ** 11 + 3440640 * t ** 13)
    y_T16 = coeffy_vec[16] * (
                2688 * t ** 4 - 128 * t ** 2 - 21504 * t ** 6 + 84480 * t ** 8 - 180224 * t ** 10 + 212992 * t ** 12 - 131072 * t ** 14 + 32768 * t ** 16 + 1)
    y_T16_dev = coeffy_vec[16] * (dxdy) * (
                10752 * t ** 3 - 256 * t - 129024 * t ** 5 + 675840 * t ** 7 - 1802240 * t ** 9 + 2555904 * t ** 11 - 1835008 * t ** 13 + 524288 * t ** 15)
    y_T16_dev2 = coeffy_vec[16] * ((dxdy) ** 2) * (
                32256 * t ** 2 - 645120 * t ** 4 + 4730880 * t ** 6 - 16220160 * t ** 8 + 28114944 * t ** 10 - 23855104 * t ** 12 + 7864320 * t ** 14 - 256)
    y_T17 = coeffy_vec[17] * (
                17 * t - 816 * t ** 3 + 11424 * t ** 5 - 71808 * t ** 7 + 239360 * t ** 9 - 452608 * t ** 11 + 487424 * t ** 13 - 278528 * t ** 15 + 65536 * t ** 17)
    y_T17_dev = coeffy_vec[17] * (dxdy) * (
                57120 * t ** 4 - 2448 * t ** 2 - 502656 * t ** 6 + 2154240 * t ** 8 - 4978688 * t ** 10 + 6336512 * t ** 12 - 4177920 * t ** 14 + 1114112 * t ** 16 + 17)
    y_T17_dev2 = coeffy_vec[17] * ((dxdy) ** 2) * (
                228480 * t ** 3 - 4896 * t - 3015936 * t ** 5 + 17233920 * t ** 7 - 49786880 * t ** 9 + 76038144 * t ** 11 - 58490880 * t ** 13 + 17825792 * t ** 15)
    y_T18 = coeffy_vec[18] * (
                162 * t ** 2 - 4320 * t ** 4 + 44352 * t ** 6 - 228096 * t ** 8 + 658944 * t ** 10 - 1118208 * t ** 12 + 1105920 * t ** 14 - 589824 * t ** 16 + 131072 * t ** 18 - 1)
    y_T18_dev = coeffy_vec[18] * (dxdy) * (
                324 * t - 17280 * t ** 3 + 266112 * t ** 5 - 1824768 * t ** 7 + 6589440 * t ** 9 - 13418496 * t ** 11 + 15482880 * t ** 13 - 9437184 * t ** 15 + 2359296 * t ** 17)
    y_T18_dev2 = coeffy_vec[18] * ((dxdy) ** 2) * (
                1330560 * t ** 4 - 51840 * t ** 2 - 12773376 * t ** 6 + 59304960 * t ** 8 - 147603456 * t ** 10 + 201277440 * t ** 12 - 141557760 * t ** 14 + 40108032 * t ** 16 + 324)
    y_T19 = coeffy_vec[19] * (
                1140 * t ** 3 - 19 * t - 20064 * t ** 5 + 160512 * t ** 7 - 695552 * t ** 9 + 1770496 * t ** 11 - 2723840 * t ** 13 + 2490368 * t ** 15 - 1245184 * t ** 17 + 262144 * t ** 19)
    y_T19_dev = coeffy_vec[19] * (dxdy) * (
                3420 * t ** 2 - 100320 * t ** 4 + 1123584 * t ** 6 - 6259968 * t ** 8 + 19475456 * t ** 10 - 35409920 * t ** 12 + 37355520 * t ** 14 - 21168128 * t ** 16 + 4980736 * t ** 18 - 19)
    y_T19_dev2 = coeffy_vec[19] * ((dxdy) ** 2) * (
                6840 * t - 401280 * t ** 3 + 6741504 * t ** 5 - 50079744 * t ** 7 + 194754560 * t ** 9 - 424919040 * t ** 11 + 522977280 * t ** 13 - 338690048 * t ** 15 + 89653248 * t ** 17)

    Cy_out = y_T0 + y_T1 + y_T2 + y_T3 + y_T4 + y_T5 + y_T6 + y_T7 + y_T8 + y_T9 + y_T10 + y_T11+y_T12+y_T13+y_T14+y_T15+y_T16+y_T17+y_T18+y_T19

    y_Cdev = y_T0_dev + y_T1_dev + y_T2_dev + y_T3_dev + y_T4_dev + y_T5_dev + y_T6_dev + y_T7_dev + y_T8_dev + y_T9_dev +y_T10_dev+y_T11_dev+y_T12_dev+y_T13_dev+y_T14_dev+y_T15_dev+y_T16_dev+y_T17_dev+y_T18_dev+y_T19_dev

    y_Cdev2 = y_T0_dev2 + y_T1_dev2 + y_T2_dev2 + y_T3_dev2 + y_T4_dev2 + y_T5_dev2 + y_T6_dev2 + y_T7_dev2 + y_T8_dev2 + y_T9_dev2 +y_T10_dev2+y_T11_dev2+y_T12_dev2+y_T13_dev2+y_T14_dev2+y_T15_dev2+y_T16_dev2+y_T17_dev2+y_T18_dev2+y_T19_dev2

    # evaluate interesting quantities(using offset for numerical reasons)
    # k_squared = ((x_Cdev2) ** 2 + (y_Cdev2) ** 2)
    # dummy = (x[0] - Cx_out) * x_Cdev2 + (x[1] - Cy_out) * y_Cdev2
    # projection_ratio = 1 / (1 - k_squared * dummy)

    k = np.sqrt((x_Cdev2) ** 2 + (y_Cdev2) ** 2 + 0.0001) #offset ensures R_max = 100
    #n_offset = (x[0] - Cx_out) * x_Cdev2 + (x[1] - Cy_out) * y_Cdev2
    projection_ratio = 1 / (1 - ((x[0] - Cx_out) * x_Cdev2 + (x[1] - Cy_out) * y_Cdev2))



    s_dot = x[3] * (np.cos(x[2]) * x_Cdev + np.sin(x[2]) * y_Cdev) * projection_ratio
    return Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2

def continuous_dynamics_Jetracer_cheby_longitudinal_motor(x, u, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)

    xdot1 = x[3] * casadi.cos(x[2])  # x_dot = v * cos(eta)
    xdot2 = x[3] * casadi.sin(x[2])  # y_dot = v * sin(eta)
    xdot3 = x[3] * casadi.tan(u[1]) / L  # v * tan(delta) / L
    xdot4 = -C * x[3] + u[0] * a_th - b_th  # vdot = -C*v+a_th*throttle+b_throttle  (linear motor torque eaten by linear viscosity)
    #xdot4 = u[0]
    Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby(x, p)
    xdot5 = s_dot  # v * (cos(eta) * cos(phi) + sin(eta) * sin(phi)) * projection_ratio
    #xdot5 = u[0]
    # assemble derivatives
    xdot = np.array([xdot1, xdot2, xdot3, xdot4, xdot5])
    return xdot

# not using this from now
def Jetracer_dynamics_circle_approx_cheby(z, p):

    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)

    # extract x, u and subparameters
    u = z[0:2]
    x = z[2:7]

    x_next = forcespro.nlp.integrate(continuous_dynamics_Jetracer_cheby_longitudinal_motor, x, u, p,
                                     integrator=forcespro.nlp.integrators.RK4, stepsize=dt)
    return x_next

def objective_jetracer_cheby(z, p):
    u = z[0:2]
    x = z[2:7]
    # extract parameters
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)
    #evaluate spline quantities
    Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby(x, p)
    err_lat_squared = (x[0] - Cx_out) ** 2 + (x[1] - Cy_out) ** 2
    #err_lat_squared = (x[1]) ** 2  # hardcoded straight path on x axis

    j = q1 * (s_dot - V_target) ** 2 + q2 * err_lat_squared + q3 * u[0] ** 2 + q4 * u[1] ** 2
    return j



# for basic s_dot formulation coming from machining MPCC

def evaluate_spline_quantities_machining_MPCC_cheby(x, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)

    # x = [x, y, eta, v, s]
    s = x[4]
    #in matlab the coefficients were referred to the standard base so -1<x<1, but here it seems that they are referred
    # to the range [a b] already

    #t = (2 / (b - a)) * s - ((a + b) / (b - a))
    #dxdy = (2 / (b - a))
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

    x_Cdev2 = x_T0_dev2 + x_T1_dev2 + x_T2_dev2 + x_T3_dev2 + x_T4_dev2 + x_T5_dev2 + x_T6_dev2 + x_T7_dev2 + x_T8_dev2 + x_T9_dev2

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

    y_Cdev2 = y_T0_dev2 + y_T1_dev2 + y_T2_dev2 + y_T3_dev2 + y_T4_dev2 + y_T5_dev2 + y_T6_dev2 + y_T7_dev2 + y_T8_dev2 + y_T9_dev2

    # evaluate interesting quantities(using offset for numerical reasons)
    # k_squared = ((x_Cdev2) ** 2 + (y_Cdev2) ** 2)
    # dummy = (x[0] - Cx_out) * x_Cdev2 + (x[1] - Cy_out) * y_Cdev2
    # projection_ratio = 1 / (1 - k_squared * dummy)

    # k = np.sqrt((x_Cdev2) ** 2 + (y_Cdev2) ** 2)
    # n_offset = (x[0] - Cx_out) * x_Cdev2 + (x[1] - Cy_out) * y_Cdev2
    projection_ratio = 1 / (1 - ((x[0] - Cx_out) * x_Cdev2 + (x[1] - Cy_out) * y_Cdev2))

    s_dot = x[3] * (np.cos(x[2]) * x_Cdev + np.sin(x[2]) * y_Cdev) * projection_ratio
    return Cx_out, Cy_out, s_dot, x_Cdev, y_Cdev




def continuous_dynamics_Jetracer_machining_MPCC_cheby_longitudinal_motor(x, u, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)

    xdot1 = x[3] * casadi.cos(x[2])  # x_dot = v * cos(eta)
    xdot2 = x[3] * casadi.sin(x[2])  # y_dot = v * sin(eta)
    xdot3 = x[3] * casadi.tan(u[1]) / L  # v * tan(delta) / L
    xdot4 = -C * x[3] + u[0] * a_th - b_th  # vdot = -C*v+a_th*throttle+b_throttle  (linear motor torque eaten by linear viscosity)
    xdot5 = x[3]  # v Simple s_dot approx taken from machining formulation

    # assemble derivatives
    xdot = np.array([xdot1, xdot2, xdot3, xdot4, xdot5])
    return xdot

# not using this from now
def Jetracer_dynamics_machining_MPCC_approx_cheby(z, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)

    # extract x, u and subparameters
    u = z[0:2]
    x = z[2:7]

    x_next = forcespro.nlp.integrate(continuous_dynamics_Jetracer_machining_MPCC_cheby_longitudinal_motor, x, u, p,
                                     integrator=forcespro.nlp.integrators.RK4, stepsize=dt)
    return x_next

def objective_jetracer_machining_MPCC_cheby(z, p):
    u = z[0:2]
    x = z[2:7]
    # extract parameters
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)
    #evaluate spline quantities
    Cx_out, Cy_out, s_dot, x_Cdev, y_Cdev = evaluate_spline_quantities_machining_MPCC_cheby(x, p)
    tangent_vector_norm = np.sqrt(x_Cdev ** 2 + y_Cdev ** 2)

    err_lag_squared = ((x[0] - Cx_out) * x_Cdev / tangent_vector_norm + (x[1] - Cy_out) * y_Cdev / tangent_vector_norm) ** 2
    err_lat_squared = ((x[0] - Cx_out) * -y_Cdev / tangent_vector_norm + (x[1] - Cy_out) * x_Cdev / tangent_vector_norm) ** 2


    j = q1 * (x[3] - V_target) ** 2 + q2 * err_lat_squared + q3 * err_lag_squared + q4 * u[0] ** 2 + q4 * u[1] ** 2
    return j















def FORCES_solver_jetracer_inequality_cheby(z,p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)
    #extract state
    x = z[2:7]
    coll_avoid0 = (x[0] + 0.25 * L * np.cos(x[2]) - xo) ** 2 + (x[1] + 0.25 * L * np.sin(x[2]) - yo) ** 2 - (
                0.4167 * L + ro) ** 2
    coll_avoid10 = (x[0] + 0.75 * L * np.cos(x[2]) - xo) ** 2 + (x[1] + 0.75 * L * np.sin(x[2]) - yo) ** 2 - (
                0.4167 * L + ro) ** 2
    Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby(x, p)

    err_lat = (x[0] - Cx_out) * y_Cdev + (x[1] - Cy_out) * (-x_Cdev) #dot product with right-pointing unit vector
    #err_lat = np.sqrt((x[0] - Cx_out) ** 2 + (x[1] - Cy_out) ** 2)

    lane_boundaries_curvature = 1 / (2 * k) - err_lat   # so to have 0<val<inf also 2 k is just a safety factor (also since err_lat is signed it only enforces inside of curve limitation)
    lane_bounds = (err_lat - l_shift) / (l_width / 2)  # scale down and centre so  -1 < val < +1
    ineq_val = [coll_avoid0, coll_avoid10, lane_boundaries_curvature, lane_bounds]
    return ineq_val

def FORCES_solver_jetracer_inequality_cheby_stage(z, p, stage):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)
    #adding time related part
    #for now hard coding constant speed for other vehicle, later do this parametrically to have flexibility
    xo_now = dyn_ob_traj_x[stage]
    yo_now = dyn_ob_traj_y[stage]
    ro_now = dyn_ob_traj_r[stage]

    #extract state
    x = z[2:7]
    coll_avoid0 = (x[0] + 0.25 * L * np.cos(x[2]) - xo) ** 2 + (x[1] + 0.25 * L * np.sin(x[2]) - yo) ** 2 - (
                0.4167 * L + ro) ** 2
    coll_avoid10 = (x[0] + 0.75 * L * np.cos(x[2]) - xo) ** 2 + (x[1] + 0.75 * L * np.sin(x[2]) - yo) ** 2 - (
                0.4167 * L + ro) ** 2
    dyn_coll_avoid0 = (x[0] + 0.25 * L * np.cos(x[2]) - xo_now) ** 2 + (x[1] + 0.25 * L * np.sin(x[2]) - yo_now) ** 2 - (
            0.4167 * L + ro_now) ** 2
    dyn_coll_avoid10 = (x[0] + 0.75 * L * np.cos(x[2]) - xo_now) ** 2 + (x[1] + 0.75 * L * np.sin(x[2]) - yo_now) ** 2 - (
            0.4167 * L + ro_now) ** 2
    Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby(x, p)

    err_lat = (x[0] - Cx_out) * y_Cdev + (x[1] - Cy_out) * (-x_Cdev) #dot product with right-pointing unit vector
    #err_lat = np.sqrt((x[0] - Cx_out) ** 2 + (x[1] - Cy_out) ** 2)

    lane_boundaries_curvature = 1 / (2 * k) - err_lat   #so to have 0<val<inf also 2 k is just a safety factor (also since err_lat is signed it only enforces inside of curve limitation)
    lane_bounds = (err_lat - l_shift) / (l_width / 2)  # scale down and centre so  -1 < val < +1
    ineq_val = [coll_avoid0, coll_avoid10, dyn_coll_avoid0, dyn_coll_avoid10, lane_boundaries_curvature, lane_bounds]
    return ineq_val






def continuous_dynamics_Jetracer_cheby_simple_model(x, u, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)

    Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby(x, p)

    xdot1 = x[3] * casadi.cos(x[2])  # x_dot = v * cos(eta)
    xdot2 = x[3] * casadi.sin(x[2])  # y_dot = v * sin(eta)
    xdot3 = x[3] * casadi.tan(u[1]) / L  # v * tan(delta) / L
    xdot4 = C * (u[0] - x[3]) # vdot = C * (vref - v)
    xdot5 = s_dot  # v * (cos(eta) * cos(phi) + sin(eta) * sin(phi)) * projection_ratio

    # assemble derivatives
    xdot = [xdot1, xdot2, xdot3, xdot4, xdot5]
    return xdot

def evaluate_local_path(x_y_state, x_of_s, y_of_s, Ds_forecast, s_vals_global_path, x_vals_original_path, y_vals_original_path, loop_path, Cheby_data_points):
    #remove the last value to avoid ambiguity since first and last value may be the same
    distances = np.zeros(s_vals_global_path.size-1)
    for ii in range(0, s_vals_global_path.size-1):
        distances[ii] = math.dist([x_vals_original_path[ii], y_vals_original_path[ii]], x_y_state[0:2])
    index = np.where(distances == np.min(distances))
    s = float(s_vals_global_path[index])

    #a = s  # if some going back is expected then change this
    #b = s + Ds_forecast

    s_subpath_cheby_for_xy_data_generation = np.linspace(s, s + Ds_forecast, Cheby_data_points)

    if loop_path and (s + Ds_forecast) > s_vals_global_path[-1]:

        s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation > s_vals_global_path[-1]] = s_subpath_cheby_for_xy_data_generation[s_subpath_cheby_for_xy_data_generation > s_vals_global_path[-1]] - s_vals_global_path[-1]

    x_data_points_Cheby = x_of_s(s_subpath_cheby_for_xy_data_generation)
    y_data_points_Cheby = y_of_s(s_subpath_cheby_for_xy_data_generation)

    #so since the initial value of s is always 0 the x(s) functions will always take [0 to Ds] as input
    s_subpath_cheby = np.linspace(0, Ds_forecast, Cheby_data_points)

    # evaluate cheby coeffs
    # the number of basis should match the number of parameters allocated for Chebishev coefficients in the solver
    coeffx = np.polynomial.chebyshev.chebfit(s_subpath_cheby, x_data_points_Cheby, 9)
    coeffy = np.polynomial.chebyshev.chebfit(s_subpath_cheby, y_data_points_Cheby, 9)


    return s, coeffx, coeffy, s_subpath_cheby, x_data_points_Cheby, y_data_points_Cheby

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

def evaluate_local_path_Chebyshev_coefficients(s, x_of_s, y_of_s, Ds_forecast, s_vals_global_path, loop_path, Cheby_data_points):
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
    #add this to smooth rcond = 0.000001
    coeffx = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, x_data_points_Cheby, 9)
    coeffy = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, y_data_points_Cheby, 9)

    return coeffx, coeffy

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
    #add this to smooth rcond = 0.000001
    # adding weights to enforce better fitting closer to the current state
    weights = np.linspace(2, 1, s_subpath_for_fitting_operation.size)
    # coeffx = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, x_data_points_Cheby, 19, rcond = 0.00000000001, w = weights)
    # coeffy = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, y_data_points_Cheby, 19, rcond = 0.00000000001, w = weights)
    coeffx = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, x_data_points_Cheby, 19)
    coeffy = np.polynomial.chebyshev.chebfit(s_subpath_for_fitting_operation, y_data_points_Cheby, 19)
    return coeffx, coeffy











def distance_from_curve_given_s(x,y,s,p):
    # s = s.reshape(-1, 0)
    dummy_x = np.array([x, y, 0, 0, s[0]])
    Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby(dummy_x, p)
    squared_norm_val = (x - Cx_out) ** 2 + (y - Cy_out) ** 2
    return squared_norm_val


def s_eval_a_posteriori_numerically(x_vec, y_vec, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(p)

    s_vec = np.zeros(x_vec.size)

    #define boounds
    bounds = Bounds(a, b)
    x0 = np.array([(a+b)/2])
    for tt in range(0, x_vec.size):
        # define anonymous function
        dist_squared = lambda s: distance_from_curve_given_s(x_vec[tt], y_vec[tt], s, p)
        res = minimize(dist_squared, x0, bounds=bounds)
        s_vec[tt] = res.x

    #s_dot_vec = np.diff(s_vec) / dt
    return s_vec


def continuous_dynamics_Jetracer_only_vehicle_model(x, u, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(
        p)

    xdot1 = x[3] * casadi.cos(x[2])  # x_dot = v * cos(eta)
    xdot2 = x[3] * casadi.sin(x[2])  # y_dot = v * sin(eta)
    xdot3 = x[3] * casadi.tan(u[1]) / L  # v * tan(delta) / L
    xdot4 = -C * x[3] + u[
        0] * a_th - b_th  # vdot = -C*v+a_th*throttle+b_throttle  (linear motor torque eaten by linear viscosity)

    # assemble derivatives
    xdot = np.array([xdot1, xdot2, xdot3, xdot4])
    return xdot

# not using this from now
def Jetracer_dynamics_circle_approx_cheby_large_DS(z, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(
        p)

    # extract x, u and subparameters
    u = z[0:2]
    x = z[2:7]

    # x_next_vehicle = forcespro.nlp.integrate(continuous_dynamics_Jetracer_only_vehicle_model, x[1:5], u, p,
    #                                   integrator=forcespro.nlp.integrators.RK4, stepsize=dt)

    x_next = forcespro.nlp.integrate(continuous_dynamics_Jetracer_machining_MPCC_cheby_longitudinal_motor, x, u, p,
                                     integrator=forcespro.nlp.integrators.RK4, stepsize=dt)


    #evaluate the next S state
    Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby(x, p)

    #  unitary inward pointing normal vector to the path
    # n_x = x_Cdev2 / k
    # n_y = y_Cdev2 / k

    # b = (x_next_vehicle[0] - x[0]) * x_Cdev + (x_next_vehicle[1] - x[1]) * y_Cdev
    # l = (1 - ((x_next_vehicle[0] - Cx_out) * x_Cdev2 + (x_next_vehicle[1] - Cy_out) * y_Cdev2)) / k
    # DS = 1 / k * np.arctan2(b, l)

    b = (x_next[0] - x[0]) * x_Cdev + (x_next[1] - x[1]) * y_Cdev
    l = (1 - ((x_next[0] - Cx_out) * x_Cdev2 + (x_next[1] - Cy_out) * y_Cdev2)) / k
    DS = 1 / k * np.arctan2(b, l)

    x_next[-1] = x[-1] + DS

    return x_next



def Jetracer_dynamics_circle_approx_forward_Euler(z, p):
    a, b, coeffx_vec, coeffy_vec, V_target, L, C, a_th, b_th, dt, xo, yo, ro, dyn_ob_traj_x, dyn_ob_traj_y, dyn_ob_traj_r, l_shift, l_width, q1, q2, q3, q4, S1, S2, S3, S4, S5, S6, S7, S8, S9 = unpack_parameters_cheby(
        p)

    # extract x, u and subparameters
    u = z[0:2]
    x = z[2:7]

    # x_next_vehicle = forcespro.nlp.integrate(continuous_dynamics_Jetracer_only_vehicle_model, x[1:5], u, p,
    #                                   integrator=forcespro.nlp.integrators.RK4, stepsize=dt)

    x_next = forcespro.nlp.integrate(continuous_dynamics_Jetracer_machining_MPCC_cheby_longitudinal_motor, x, u, p,
                                     integrator=forcespro.nlp.integrators.RK4, stepsize=dt)

    #evaluate the next S state
    Cx_out, Cy_out, s_dot, k, x_Cdev, y_Cdev, x_Cdev2, y_Cdev2 = evaluate_spline_quantities_cheby(x, p)


    DS = x[3] * dt

    x_next[-1] = x[-1] + DS

    return x_next

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

    else:
        print('Invalid choice of track:')
        print('You selected: ', choice)


    return Checkpoints_x, Checkpoints_y

