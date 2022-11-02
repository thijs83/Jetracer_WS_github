import numpy as np

def Y_axis_quaternion_to_euler_angle_home_made(w, y):
    # THIS IS ALSO RESPONSIBLE FOR AXIS FLIPPING from optitrack
    #assuming the axis of rotation is the y axis (y up in motive convention)
    #If w comes negative then just flip everything (the sign is inverted with respect to what comes out ot the optitrack)
    #be aware of this because it feels like a hard coded quick fix that might be a problem if optitrack stops flipping the signs
    if w < 0:
        w = -w
        y = -y

    a = + 2 * np.arctan2(y, w)
    return a