#!/usr/bin/env python3
import numpy as np

import sys
import rospy

from std_msgs.msg import Float32, Float32MultiArray

from datetime import datetime
import csv
from matplotlib.patches import Rectangle
# for dynamic paramters reconfigure (setting param values from rqt_reconfigure GUI)
from dynamic_reconfigure.server import Server
from dynamic_reconfigure_pkg.cfg import Platooning_dynamic_reconfigureConfig
import rospkg


from custom_msgs_optitrack.msg import custom_opti_pose_stamped_msg
#import forcespro (add path before you add it)
sys.path.insert(0, '/home/lorenzo/OneDrive/PhD/ubuntu_stuff/Forces_pro_extracted/forces_pro_client')
import forcespro.nlp


#Unfortunatly there seems to be no function "get latest value on a topic" so global
#values are defined and updated by callback functions in the relative subscribers
#for now they will be used as initial values cause they are not updated



class Platooning_controller_class:
    def __init__(self, car_number):
        rospy.init_node('Platooning_control_node_' + str(car_number), anonymous=False)

        #set up variables
        self.car_number = car_number

        self.setup_constant_parameters()

        # initialize state variables
        # [v v_rel x_rel]
        self.state = [0, 0, 0]



        # set up publisher and subscribers
        self.throttle_publisher = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=1)
        self.steering_publisher = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=1)
        self.safety_value_subscriber = rospy.Subscriber('safety_value', Float32, self.safety_value_subscriber_callback)
        #self.occupancy_xyr_4_subscriber = rospy.Subscriber('occupancy_xyr_4', Float32MultiArray, self.occupancy_xyr_4_subscriber_callback)
        self.v_encoder_subscriber = rospy.Publisher('velocity_' + str(car_number), Float32, queue_size=1)
        self.x_rel_subscriber = rospy.Subscriber('x_rel_', Float32, self.x_rel_subscriber_callback) #subscribe to lidar and camera data output


        #fory dynamic parameter change using rqt_reconfigure GUI
        srv = Server(Platooning_dynamic_reconfigureConfig, self.reconfig_callback)


    def start_platooning_control_loop(self):


        self.rate = rospy.Rate(1 / self.dt)
        while not rospy.is_shutdown():
            #compute linear controller contorl action

            u_lin = self.kd * self.state[1] + self.kp*self.state[2] + self.h*(self.state[0] - self.V_target)
            tau = self.acc_2_throttle(u_lin)
            self.publish_throttle(tau)

            self.rate.sleep()




    def setup_constant_parameters(self):
        # setu up N
        self.N = 30  # must match the number of stages in the solver

        # generate parameters
        self.V_target = 1  # in terms of being scaled down the proportion is vreal life[km/h] = v[m/s]*42.0000  (assuming the 30cm jetracer is a 3.5 m long car)
        self.dt = 0.1  # so first number is the prediction horizon in seconds -this is the dt of the solver so it will think that the control inputs are changed every dt seconds

        self.kp = -1
        self.kd = -2
        self.h = -1

    def acc_2_throttle(self, acc):
        # compute inverted dynamics to recover throttle from required acceleration
        C = 1.54 / 1.63  # longitudinal damping coefficient divided by the mass
        a_th = 60 / 1.63  # motor curve coefficient divided by the mass
        b_th = 1.54 / 1.63

        # xdot4 = -C * (x[3] - 1) + (u[0] - 0.129) * a_th
        tau = (acc + C * (self.state[0] - 1))/a_th + 0.129
        return tau
    def publish_throttle(self, tau):
        # saturation limits for tau
        if tau < 0:
            tau = 0
        elif tau > 1:
            tau = 1

        throttle_val = Float32(tau)

        # for data storage purpouses
        self.throttle = throttle_val.data

        #publish inputs
        self.throttle_publisher.publish(throttle_val)


    def setup_data_recording(self):
        date_time = datetime.now()
        date_time_str = date_time.strftime("%m_%d_%Y_%H_%M_%S")
        # date_time = date_time.strftime("%H_%M_%S")
        #subfolder = 'GP_MPCC_data_recordings/'
        #current_dir = os.path.realpath(os.path.dirname(__file__))
        #rospack.get_path('self.data_folder_name')
        rospack = rospkg.RosPack()
        gp_pkg_path = rospack.get_path('platooning_mpcc_pkg')
        #print('---------')
        #print(gp_pkg_path)
        print(gp_pkg_path)
        file_name = gp_pkg_path + '/src/' + self.data_folder_name + '/recording_' + date_time_str + '.csv'
        # start_clock_time = rospy.get_rostime()

        file = open(file_name, 'w')
        writer = csv.writer(file)
        #data_msg = Float32MultiArray()
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





    def sub_vel_callback(self, msg):
        # state = [v v_rel x_rel]
        self.state[0] = msg.data

    def x_rel_subscriber_callback(self, msg):
        # state = [v v_rel x_rel]
        self.state[1] = msg.data
        self.state[2] = 1
        #compute x_rel and v_rel

    def safety_value_subscriber_callback(self, msg):
        print(msg.data)
        self.safety_value = msg.data



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

    def reconfig_callback(self, config, level):
        print('reconfiguring parameters from dynamic_reconfig')

        self.V_target = config['V_target']
        self.kp = config['kp']
        self.kd = config['kd']
        self.h = config['h']
        self.save_data = config['save_data']
        #self.data_folder_name = config['data_folder_name']

        if self.save_data == False:
            try:
                #close file when recording is switched off
                self.file.close()
            except:
                print('no file to close, probably it is first initialization')


        self.rate = rospy.Rate(1 / self.dt)



        return config










if __name__ == '__main__':
    try:

        car_number_3 = 3

        # choose solver to run
        vehicle_3_controller = Platooning_controller_class(car_number_3)
        vehicle_3_controller.start_platooning_control_loop()




    except rospy.ROSInterruptException:
        pass
