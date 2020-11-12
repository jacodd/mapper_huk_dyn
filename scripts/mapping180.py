#!/usr/bin/env python

import rospy
import sys
import time
import rospkg
import os
import datetime

from sensor_msgs.msg import JointState
import mapper_dyn_huk.srv
import std_srvs.srv
from dynamixel_workbench_msgs.srv import DynamixelCommand

reached = False
joint_msg = None

def set_position(setpoint, speed=1):
    set_speed(speed)
    try:
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command', timeout=2)
        dyn_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        resp1 = dyn_command("", 1, "Goal_Position", setpoint)
        #print("Set position: %s"% resp1)
        return resp1.comm_result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def set_speed(speed):
    try:
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command', timeout=2)
        dyn_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        resp1 = dyn_command("", 1, "Moving_Speed", speed)
        #print("Set speed: %s"% resp1)
        return resp1.comm_result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def joint_states_callback(msg):
    global joint_msg
    joint_msg = msg
    #print 'Received msg:', msg.header.seq

def set_motor_position(setpoint=0.0, tolerance=0.01, speed=5):
    a = -3.14
    b = 3.14
    minx = 0
    maxx = 4096

    angle_setpoint = ((b - a) * (float(setpoint - minx) / float(maxx - minx)) + a)

    r = rospy.Rate(2)
    while not rospy.is_shutdown():

        if not joint_msg:
            r.sleep()

        setpoint_error = (abs(joint_msg.position[0] - angle_setpoint))
        rospy.loginfo('[mapping180] setpoint_error:%s', setpoint_error)
        if joint_msg and setpoint_error < tolerance:    
            rospy.loginfo('[mapping180] pantilt 0.0 position reached')
            break
        
        set_position(
            setpoint,
            speed=speed)

        #rospy.loginfo('[mapping180] sent control: setpoint:%s', setpoint)
        r.sleep()
    pass

def reset_octomap():
    try:
        rospy.wait_for_service('/octomap_server/reset', timeout=3.0)
        reset_octomap_srv = rospy.ServiceProxy('/octomap_server/reset', std_srvs.srv.Empty)
        resp1 = reset_octomap_srv()
        rospy.loginfo("[mapping180] response from octomap reset: %s", resp1)
    except rospy.ServiceException as e:
        rospy.loginfo("[mapping180] Exception in service call octomap failed: %s", e)
    except Exception as e:
        rospy.loginfo("[mapping180] Service call octomap reset: %s", e)

def save_octomap():
    try:
        rospy.wait_for_service('/espeleo_octomap_saver', timeout=3.0)
        reset_octomap_srv = rospy.ServiceProxy('/espeleo_octomap_saver', mapper_dyn_huk.srv.espeleoSaveOctomap)

        filename = 'octomap_dump-{date:%Y-%m-%d_%H:%M:%S}.bt'.format(date=datetime.datetime.now())
        rospack = rospkg.RosPack()
        output_path = os.path.join(rospack.get_path('mapper_dyn_huk'), 'bags', filename)
        is_full = True

        rospy.loginfo("[mapping180] saving octomap dump to: %s", output_path)

        resp1 = reset_octomap_srv(output_path, is_full)
        rospy.loginfo("[mapping180] response from octomap save: %s", resp1)
    except rospy.ServiceException as e:
        rospy.loginfo("[mapping180] Service call octomap save failed: %s", e)
    except Exception as e:
        rospy.loginfo("[mapping180] Exception in service call octomap save: %s", e)

if __name__ == '__main__':
    rospy.init_node('dyn_cmd_positions')
    rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, joint_states_callback)

    tolerance = 0.01

    pan_min = 0.0
    pan_max = 4096
    pan = pan_max
    tilt = 0.0
    map_speed = rospy.get_param('~map_speed', 0.1)

    rospy.loginfo('[mapping180] pan_min:%s pan_max:%s', pan_min, pan_max)
    rospy.loginfo('[mapping180] pan map_speed:%s', map_speed)

    rospy.loginfo('[mapping180] go to initial position (fast!)')
    set_motor_position(pan_min, speed=60)

    rospy.loginfo('[mapping180] resetting Octomap server')
    reset_octomap()

    rospy.loginfo('[mapping180] start mapping...')
    set_motor_position(2048, speed=map_speed)
    
    #time.sleep(2)

    # while not rospy.is_shutdown():
    #     if joint_msg:
    #         rospy.loginfo('[mapping180] pan pos:%s', joint_msg.position[0])

    #     if joint_msg and abs(joint_msg.position[0] - pan) < tolerance:
    #         rospy.set_param('capture_point_cloud_360', 0)
    #         rospy.loginfo('[mapping180] pantilt 360 pan movement completed err %s = (%s - %s)', abs(joint_msg.position[0] - pan), joint_msg.position[0], pan)
    #         break

    #     set_position(
    #         float(pan),
    #         tilt,
    #         speed=map_speed)
    #     rospy.loginfo('[mapping180] sent control: pan:%s tilt:%s', pan, tilt)
    rospy.loginfo('[mapping180] ending mapping...')
    rospy.set_param("capture_point_cloud_360", False)
    save_octomap()
