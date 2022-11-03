#!/usr/bin/env python3

from NatNetClient import NatNetClient
import rospy
from geometry_msgs.msg import PoseStamped
from support_functions import Y_axis_quaternion_to_euler_angle_home_made


def receive_new_frame(*args, **kwargs):
    pass


def receive_rigidbody_frame(id, position, rotation):
    print('recieved new rigidbody frame')
    #print(f'id: {id}, position: {position}, rotation: {rotation}')
    Optitrack_message = PoseStamped()

    #fill in header with time information
    Optitrack_message.header.stamp = rospy.Time.now()

    Optitrack_message.pose.position.x = -position[0]
    Optitrack_message.pose.position.y = position[2]
    Optitrack_message.pose.position.z = position[1]

    y_axis_rot_home_made=Y_axis_quaternion_to_euler_angle_home_made(rotation[3], rotation[1])

    Optitrack_message.pose.orientation.x = 0
    Optitrack_message.pose.orientation.y = y_axis_rot_home_made
    Optitrack_message.pose.orientation.z = 0
    Optitrack_message.pose.orientation.w = 0


    #send message to the right topic
    if id == 1:
        optitrack_publisher_1.publish(Optitrack_message)
        print('published on Optitrack_data_topic_1')
    elif id == 2:
        optitrack_publisher_2.publish(Optitrack_message)
        print('published on Optitrack_data_topic_2')
    elif id == 3:
        optitrack_publisher_3.publish(Optitrack_message)
        print('published on Optitrack_data_topic_3')
    elif id == 4:
        optitrack_publisher_4.publish(Optitrack_message)
        print('published on Optitrack_data_topic_4')
    elif id == 5:
        optitrack_publisher_5.publish(Optitrack_message)
        print('published on Optitrack_data_topic_5')
    elif id == 6:
        optitrack_publisher_6.publish(Optitrack_message)
        print('published on Optitrack_data_topic_6')
    elif id == 7:
        optitrack_publisher_7.publish(Optitrack_message)
        print('published on Optitrack_data_topic_7')
    else:
        print('rigid body id was larger than 7 so did not know where to publish')


#set up the publisher nodes
optitrack_publisher_1 = rospy.Publisher('Optitrack_data_topic_1', PoseStamped, queue_size=1)
optitrack_publisher_2 = rospy.Publisher('Optitrack_data_topic_2', PoseStamped, queue_size=1)
optitrack_publisher_3 = rospy.Publisher('Optitrack_data_topic_3', PoseStamped, queue_size=1)
optitrack_publisher_4 = rospy.Publisher('Optitrack_data_topic_4', PoseStamped, queue_size=1)
optitrack_publisher_5 = rospy.Publisher('Optitrack_data_topic_5', PoseStamped, queue_size=1)
optitrack_publisher_6 = rospy.Publisher('Optitrack_data_topic_6', PoseStamped, queue_size=1)
optitrack_publisher_7 = rospy.Publisher('Optitrack_data_topic_7', PoseStamped, queue_size=1)

rospy.init_node('Optitrack_publisher_node', anonymous=False)



if __name__ == '__main__':
    try:
        # Streaming client in separate thread
        streaming_client = NatNetClient()
        streaming_client.newFrameListener = receive_new_frame
        streaming_client.rigidBodyListener = receive_rigidbody_frame
        streaming_client.run()
        print('running the python file opti_minimal 4 cars --> will publish each car on a different topic called: Optitrack_data_topic_1')
    except rospy.ROSInterruptException:
        pass
