#!/usr/bin/env python3

# this script simulates the output from the optitrack, so you can use it for tests
# the 4 vehicles are all alligne spced 0.5 m apart

import rospy
from std_msgs.msg import String, Float32
from custom_msgs_optitrack.msg import custom_opti_pose_stamped_msg

def opti_test_publisher():
    #set up publisher handles
    test_publisher_1 = rospy.Publisher('Optitrack_data_topic_1', custom_opti_pose_stamped_msg, queue_size = 10)
    test_publisher_2 = rospy.Publisher('Optitrack_data_topic_2', custom_opti_pose_stamped_msg, queue_size = 10)
    test_publisher_3 = rospy.Publisher('Optitrack_data_topic_3', custom_opti_pose_stamped_msg, queue_size = 10)
    test_publisher_4 = rospy.Publisher('Optitrack_data_topic_4', custom_opti_pose_stamped_msg, queue_size = 10)

    test_velocity_publisher_1 = rospy.Publisher('velocity_1', Float32, queue_size = 10)
    test_velocity_publisher_2 = rospy.Publisher('velocity_2', Float32, queue_size = 10)
    test_velocity_publisher_3 = rospy.Publisher('velocity_3', Float32, queue_size = 10)
    test_velocity_publisher_4 = rospy.Publisher('velocity_4', Float32, queue_size = 10)

    # initialize current node and set publishing rate
    rospy.init_node('Optitrack_tests', anonymous=False)
    rate = rospy.Rate(10)

    # create empty messages
    message_1 = custom_opti_pose_stamped_msg()
    message_2 = custom_opti_pose_stamped_msg()
    message_3 = custom_opti_pose_stamped_msg()
    message_4 = custom_opti_pose_stamped_msg()

    velocity_message_1 = custom_opti_pose_stamped_msg()
    velocity_message_2 = custom_opti_pose_stamped_msg()
    velocity_message_3 = custom_opti_pose_stamped_msg()
    velocity_message_4 = custom_opti_pose_stamped_msg()



    while not rospy.is_shutdown():
        #fill in the messages
        message_1.header.stamp = rospy.Time.now()
        message_1.x=0
        message_1.y=0
        message_1.rotation= 3.14 * 0.5

        message_2.header.stamp = rospy.Time.now()
        message_2.x=0.5
        message_2.y=0.5
        message_2.rotation=0

        message_3.header.stamp = rospy.Time.now()
        message_3.x=1
        message_3.y=1
        message_3.rotation=0

        message_4.header.stamp = rospy.Time.now()
        message_4.x=1.5
        message_4.y=1.5
        message_4.rotation=0

        #velocity
        velocity_message_1 = 1
        velocity_message_2 = 1
        velocity_message_3 = 1
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

        rate.sleep()

if __name__ == '__main__':
    try:
        opti_test_publisher()
    except rospy.ROSInterruptException:
        pass
