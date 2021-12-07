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

        rate = rospy.Rate(desired_freq)

    while not rospy.is_shutdown():
        target = []

        for i in range(12):
            motor_cmds[i].q = joint_state_commands[i][index]

        for i in range(12):
            pubs[i].publish(motor_cmds[i])

        rate.sleep()

        if index < node_num - 1:
            index += 1
        else:
            break


def interpolate_traj(data):


if __name__ == '__main__':
    f = open('results.json', 'r')
    data = json.load(f)

    base_poses = data['base_poses']
    FR_states = data['FR_states']
    FL_states = data['FL_states']
    RR_states = data['RR_states']
    RL_states = data['RL_states']
    time_span = data['time_span']
    total_grid = data['total_grid']

    desired_freq = 50 # 50Hz
    node_num = round(time_span*desired_freq)
    time_data = np.linspace(0, time_span, total_grid)
    time_new = np.linspace(0, time_span, node_num)

    # Define interpolation functions
    fr = interpolate.interp1d(time_data, FR_states.T, kind='linear', axis=-1) #time_data = 1xtime, FR states joint num x time
    fr_1 = interpolate.interp1d(time_data, FR_states[0, :])
    fr_2 = interpolate.interp1d(time_data, FR_states[1, :])
    fr_3 = interpolate.interp1d(time_data, FR_states[2, :])

    fl_1 = interpolate.interp1d(time_data, FL_states[0, :])
    fl_2 = interpolate.interp1d(time_data, FL_states[1, :])
    fl_3 = interpolate.interp1d(time_data, FL_states[2, :])

    rr_1 = interpolate.interp1d(time_data, RR_states[0, :])
    rr_2 = interpolate.interp1d(time_data, RR_states[1, :])
    rr_3 = interpolate.interp1d(time_data, RR_states[2, :])

    rl_1 = interpolate.interp1d(time_data, RL_states[0, :])
    rl_2 = interpolate.interp1d(time_data, RL_states[1, :])
    rl_3 = interpolate.interp1d(time_data, RL_states[2, :])

    # Interpolate desired states to signal frequency
    fr_cmd = fr(time_new)
    fr_1_cmd = fr_1(time_new)
    fr_2_cmd = fr_2(time_new)
    fr_3_cmd = fr_3(time_new)

    fl_1_cmd = fl_1(time_new)
    fl_2_cmd = fl_2(time_new)
    fl_3_cmd = fl_3(time_new)

    rr_1_cmd = rr_1(time_new)
    rr_2_cmd = rr_2(time_new)
    rr_3_cmd = rr_3(time_new)

    rl_1_cmd = rl_1(time_new)
    rl_2_cmd = rl_2(time_new)
    rl_3_cmd = rl_3(time_new)

    joint_state_commands = np.vstack((fr_1_cmd, fr_2_cmd, fr_3_cmd, fl_1_cmd, fl_2_cmd, fl_3_cmd, rr_1_cmd, rr_2_cmd, rr_3_cmd, rl_1_cmd, rl_2_cmd, rl_3_cmd))

    try:
        trajectory(data)
    except rospy.ROSInterruptionException:
        pass
