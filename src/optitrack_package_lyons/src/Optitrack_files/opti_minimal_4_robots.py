#!/usr/bin/env python3

from NatNetClient import NatNetClient
import rospy
from geometry_msgs.msg import PoseStamped
from support_functions import Y_axis_quaternion_to_euler_angle_home_made
from custom_msgs_optitrack.msg import custom_opti_pose_stamped_msg


def receive_new_frame(*args, **kwargs):
    pass


def receive_rigidbody_frame(id, position, rotation):
    Optitrack_message = custom_opti_pose_stamped_msg()#() THIS EXTRA PARENTESYS CAN BE REMOVED?

    #WATCH OUT FOR OPTITRACK CONVENTION
    #fill in header with time information (dependent on optitrack convention)
    Optitrack_message.header.stamp = rospy.Time.now()
    Optitrack_message.x = -position[0]
    Optitrack_message.y = position[2]
    Optitrack_message.rotation = Y_axis_quaternion_to_euler_angle_home_made(rotation[3], rotation[1])

    # publish on the topic according to the rigid body ID number
    if id == 1:
        optitrack_publisher_1.publish(Optitrack_message)
    elif id == 2:
        optitrack_publisher_2.publish(Optitrack_message)
    elif id == 3:
        optitrack_publisher_3.publish(Optitrack_message)
    elif id == 4:
        optitrack_publisher_4.publish(Optitrack_message)
    elif id == 5:
        optitrack_publisher_5.publish(Optitrack_message)
    elif id == 6:
        optitrack_publisher_6.publish(Optitrack_message)
    elif id == 7:
        optitrack_publisher_7.publish(Optitrack_message)
    else:
        print('rigid body id was larger than 7 so did not know where to publish')




optitrack_publisher_1 = rospy.Publisher('Optitrack_data_topic_1', custom_opti_pose_stamped_msg, queue_size=1)
optitrack_publisher_2 = rospy.Publisher('Optitrack_data_topic_2', custom_opti_pose_stamped_msg, queue_size=1)
optitrack_publisher_3 = rospy.Publisher('Optitrack_data_topic_3', custom_opti_pose_stamped_msg, queue_size=1)
optitrack_publisher_4 = rospy.Publisher('Optitrack_data_topic_4', custom_opti_pose_stamped_msg, queue_size=1)
optitrack_publisher_5 = rospy.Publisher('Optitrack_data_topic_5', custom_opti_pose_stamped_msg, queue_size=1)
optitrack_publisher_6 = rospy.Publisher('Optitrack_data_topic_6', custom_opti_pose_stamped_msg, queue_size=1)
optitrack_publisher_7 = rospy.Publisher('Optitrack_data_topic_7', custom_opti_pose_stamped_msg, queue_size=1)
rospy.init_node('Optitrack_publisher_node', anonymous=False)




if __name__ == '__main__':
    try:
        # Streaming client in separate thread
        streaming_client = NatNetClient()
        streaming_client.newFrameListener = receive_new_frame
        streaming_client.rigidBodyListener = receive_rigidbody_frame
        streaming_client.run()
        print('running the python file opti_minimal 4 cars --> will publish each car on a different topic called: Optitrack_data_topic_NUMBER')
    except rospy.ROSInterruptException:
        pass
