# %%
#!/usr/bin/env python3

import rospy
import pygame
import time
from std_mesgs.msg import Float32

# %%
def teleop_gamepad():
    #Initialize pygame and gamepad
    pygame.init()
    j = pygame.joystick.Joystick(0)
    j.init()
    print('Initialized Joystick ; %s' % j.get_name())

    car_number = 3
    ref_tau = 0.10
    incr = 0.02

    # Set up topics publishing and nodes
    pub_throttle = rospy.Publisher('throttle' + str(car_number), Float32, queue_size=8)
    pub_steering = rospy.Publisher('steering' + str(car_number), Float32, queue_size=8)

    pub_safety_value = rospy.Publisher('safety_value', Float32, queue_size=8)

    rospy.init_node('teleop_gamepad' + str(car_number), anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pygame.event.pump()

        steering = 0
        pub_steering.publish(steering)

		#safety value publishing
        if j.get_button(7) == 1:
            print('safety off')
            pub_safety_value.publish(1)
        else:
            pub_safety_value.publish(0)

		
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if j.get_button(4) == 1: # button Y
                    throttle = throttle + incr
                    print('tau_ref = ', throttle)
                    pub_throttle.publish(throttle)
                if j.get_button(0) == 1: # button A
                    throttle = throttle - incr
                    print('tau_ref = ', throttle)
                    pub_throttle.publish(throttle)
                if j.get_button(1) == 1: # button B
                    throttle = ref_tau
                    print('tau_ref = ', throttle)
                    pub_throttle.publish(throttle)
                    
        rate.sleep()

if __name__ == '__main__':
    try:
        teleop_gamepad()
    except rospy.ROSInterruptException:
        pass



