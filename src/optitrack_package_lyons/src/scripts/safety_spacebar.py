#!/usr/bin/env python3

import rospy
from getkey import getkey
from std_msgs.msg import Float32
from pynput.keyboard import Key, Listener




#defining a global variable
safety_value=0

def on_press(key):
    global safety_value
    if key == Key.backspace:
        safety_value= 1
        pub_safety.publish(safety_value)
        # print('{0} pressed'.format(
        # key))
        print('safety disingaged')
    else:
        print('press backspace to disinge safety')


def on_release(key):
    global safety_value
    safety_value = 0
    pub_safety.publish(safety_value)
    # print('{0} release'.format(key))
    print('safety engaged')
    if key == Key.esc:
        # Stop listener
        return False



def teleop():
    #Setup topics publishing and nodes

    rospy.init_node('safety', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #Print hints
    print("Running safety.py")
    print("Press any key to set safety_value to 1 and published at 10hz, when released safety_value is set to 0 and stops publishing. To stop the key listeneer press esc")

    while not rospy.is_shutdown():
        #Get key press
        #key = getkey()

        # Collect events until released
        with Listener(
                on_press=on_press,
                on_release=on_release) as listener:
            listener.join()



        #set safety value
        #if key == 'SPACE':
        #if keyboard.is_pressed('q'):
        #    pub_safety.publish(1)
        #else:
        pub_safety.publish(safety_value)

        rate.sleep()

pub_safety = rospy.Publisher('safety_value', Float32, queue_size=1)

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
