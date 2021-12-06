#!/usr/bin/env python

import rospy
from scipy import interpolate
import json
import numpy as np
from msgs.msg import MotorCmd # In cloned directory

def trajectory(data):
    robot_name = "a1"

    pubs = []
    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FR_hip_controller/command', MotorCmd, queue_size=1))
    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FR_thigh_controller/command', MotorCmd, queue_size=1))
    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FR_calf_controller/command', MotorCmd, queue_size=1))

    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FL_hip_controller/command', MotorCmd, queue_size=1))
    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FL_thigh_controller/command', MotorCmd, queue_size=1))
    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FL_calf_controller/command', MotorCmd, queue_size=1))

    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RR_hip_controller/command', MotorCmd, queue_size=1))
    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RR_thigh_controller/command', MotorCmd, queue_size=1))
    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RR_calf_controller/command', MotorCmd, queue_size=1))

    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RL_hip_controller/command', MotorCmd, queue_size=1))
    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RL_thigh_controller/command', MotorCmd, queue_size=1))
    pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RL_calf_controller/command', MotorCmd, queue_size=1))

    if not started:
        rospy.init_node('cmd_publisher')

    self.mpos_interp_func =  interpolate.interp1d(np.linspace(0, data['time_span'], data['total_grid']).flatten(), self.ref_motion['Motor_Pos'].T, kind='linear', axis=-1)

    index = 0

    motor_cmds = [MotorCmd() for _ in range(12)]

    for i in range(4):
        motor_cmds[i*3+0].mode = 10
        motor_cmds[i*3+0].Kp = 5 # 70 in gazebo
        motor_cmds[i*3+0].dq = 0
        motor_cmds[i*3+0].Kd = 1 # 3 in gazebo
        motor_cmds[i*3+0].tau = 0

        motor_cmds[i*3+1].mode = 10
        motor_cmds[i*3+1].Kp = 5 # 180
        motor_cmds[i*3+1].dq = 0
        motor_cmds[i*3+1].Kd = 1 # 8
        motor_cmds[i*3+1].tau = 0

        motor_cmds[i*3+2].mode = 10
        motor_cmds[i*3+2].Kp = 5 # 300
        motor_cmds[i*3+2].dq = 0
        motor_cmds[i*3+2].Kd = 1 # 15
        motor_cmds[i*3+2].tau = 0

    while not rospy.is_shutdown():
        target = []

        for i in range(12):
            motor_cmds[i].q = states[i][index]

        for i in range(12):
            pubs[i].publish(motor_cmds[i])

        rate.sleep()

        if index < node_num - 1:
            index += 1
        else:
            break

if __name__ == '__main__':
    f = open('results.json', 'r')
    data = json.load(f)
    trajectory(data)
