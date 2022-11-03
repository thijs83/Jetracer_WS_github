#!/usr/bin/env python3

# this script simulates the output from the optitrack, so you can use it for tests
# the 4 vehicles are all alligne spced 0.5 m apart

import rospy
from std_msgs.msg import String, Float32, Header



class relay_time_stamp:
    def __int__(self):
        print('hello 3')


    def time_stamp_callback(self, incoming_header):
        self.timestamp_publisher.publish(incoming_header)

    def print_hello(self):
        print('hello 4')
        rospy.init_node('timestamp_relay_node', anonymous=False)
        rospy.Subscriber('Vehicle_timestamp_3', Header, self.time_stamp_callback)
        self.timestamp_publisher = rospy.Publisher('republished_vehicle_timestamp_3', Header, queue_size=1)
        rospy.spin()



if __name__ == '__main__':
    print('hello 1')
    try:
        print('hello 2')
        a = relay_time_stamp()
        a.print_hello()
    except rospy.ROSInterruptException:
        print('onf')
        pass
