#!/usr/bin/env python3

"""
Publisher node to setup driving speed
and steering angles to the 4 wheel 
independent steering robot.

node: teleop_keyboard
Publish to: /drivingmotors/commands
            /steeringmotors/commands

Moving Commands:
---------------------------

        w
    a       d
        s

w/s: increase/decrease linear velocity 
a/d: increase/decrease steering angle
space key: force stop

CTRL-C to quit

by Pablo
Last review: 2022/07/26
"""

import rospy

import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

from scripts.robotDic import robot

from std_msgs.msg import Float64MultiArray


LIN_VEL_STEP_SIZE = 0.1
STEERING_ANGLE_STEP_SIZE = 1


msg = """
Moving Commands:
---------------------------

        w
    a       d
        s

w/s: increase/decrease linear velocity 
a/d: increase/decrease steering angle
space key: force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel):
    return "currently:\tlinear vel %s " % (target_linear_vel)

def angles(target_angle):
    return "currently:\tangle %s " % (target_angle)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -robot["MAX_LINEAR_SPEED"], robot["MAX_LINEAR_SPEED"])
    return vel

def checkLimitAngle(angle):
    angle = constrain(angle, robot["MIN_STEERING_ANGLE"], robot["MAX_STEERING_ANGLE"])
    return angle

def angleScale(angle):
    """
    Function to scale the angles values between 0 to 180
    """
    if angle >= 180:
        angle-=270
    else:
        angle+=90

def drivingMessage(mode, vel):
    # Create message as Float array
    dmke_msg = Float64MultiArray()

    if mode == 1: # Parallel
        dmke_msg.data = [vel, vel, vel, vel]
        return dmke_msg

    elif mode == 2: # Double ackerman
        dmke_msg.data = [vel, vel, vel, vel]
        return dmke_msg

    elif mode == 3: # Spin
        dmke_msg.data = [vel, -vel, -vel, vel]
        return dmke_msg

    elif mode == 4: # Crab
        dmke_msg.data = [vel, -vel, -vel, vel]
        return dmke_msg

    else: # Just
        dmke_msg.data = [vel, vel, vel, vel]
        return dmke_msg

def steeringMessage(mode, angle):
    # Create message as Float array
    ggm_msg = Float64MultiArray()   
    #print(mode) 

    if mode == 1: # Parallel
        ggm_msg.data = [angle, angle, angle, angle]
        return ggm_msg

    elif mode == 2: # Double ackerman
        backAngle = 180 - angle
        ggm_msg.data = [angle, angle, backAngle, backAngle]
        return ggm_msg

    elif mode == 3: # Spin
        ggm_msg.data = [135, 45, 135, 45]
        return ggm_msg

    elif mode == 4: # Crab
        ggm_msg.data = [180, 0, 0, 180]
        return ggm_msg

    else: # Just
        ggm_msg.data = [angle, angle, angle, angle]
        return ggm_msg

    


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # Create node
    rospy.init_node('teleop_keyboard')
    pub_dmke = rospy.Publisher('/drivingmotors/commands', Float64MultiArray)
    pub_ggm = rospy.Publisher('/steeringmotors/commands', Float64MultiArray)
    rate = rospy.Rate(10) # 10hz

    # Parameters inicialization
    target_linear_vel   = 0.0
    control_linear_vel  = 0.0

    target_angle   = 90.0
    control_angle  = 90.0

    mode = 1

    try:
        print(msg)
        while not rospy.is_shutdown():
            # Get Key
            key = getKey()
            if key == 'w':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                print(vels(target_linear_vel))
            elif key == 's':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                print(vels(target_linear_vel))
            elif key == 'a' and (mode == 1 or mode == 2):
                target_angle = checkLimitAngle(target_angle - STEERING_ANGLE_STEP_SIZE)
                print(angles(target_angle))
            elif key == 'd' and (mode == 1 or mode == 2):
                target_angle = checkLimitAngle(target_angle + STEERING_ANGLE_STEP_SIZE)
                print(angles(target_angle))
            elif key == ' ':
                # Driving motor speed 0
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                print(vels(target_linear_vel))
                # Steering motor angles = current angle
                target_angle = control_angle
                print(angles(target_angle))
            elif key == '1':
                mode = 1
            elif key == '2':
                mode = 2
            elif key == '3':
                mode = 3
            elif key == '4':
                mode = 4
            else:
                if (key == '\x03'):
                    break

            # Calculate linear speed profile 
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_angle = makeSimpleProfile(control_angle, target_angle, (STEERING_ANGLE_STEP_SIZE/1.3))

            # Fill message depending on the mode and publish
            pub_dmke.publish(drivingMessage(mode, control_linear_vel))
            pub_ggm.publish(steeringMessage(mode, control_angle))
            rate.sleep()

    except:
        print(e)

    

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)