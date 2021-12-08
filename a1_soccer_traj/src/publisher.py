#!/usr/bin/env python

import rospy
from scipy import interpolate
import json
import numpy as np
from msgs.msg import MotorCmd # In cloned directory
from std_msgs.msg import Bool


class CmdPublisher:
    def __init__(self, file_name):
        self.__load_data(file_name)
        self.desired_freq = 200
        self.__init_motor_cmd()
        self.__init_joint_cmd_pubs()
        self.start_interp = False
        self.reset()
        self.start_flag_sub = rospy.Subscriber('/traj_opt/start_traj', Bool, self.__start_flag_sub)
        self.cmd_pub_timer = rospy.Timer(1.0/self.desired_freq, self.__interp_motion)

    def reset(self):
        self.curr_time = 0.0

    def __load_data(self, file_name):
        f = open(file_name, 'r')
        data = json.load(f)

        base_poses = data['base_poses']
        FR_states = data['FR_states']
        FL_states = data['FL_states']
        RR_states = data['RR_states']
        RL_states = data['RL_states']
        self.time_span = data['time_span']
        total_grid = data['total_grid']

        time_data = np.linspace(0, time_span, total_grid)

        self.fr = interpolate.interp1d(time_data, FR_states.T, kind='linear', axis=-1) #time_data = 1xtime, FR states joint num x time
        self.fl = interpolate.interp1d(time_data, FL_states.T, kind='linear', axis=-1)
        self.rr = interpolate.interp1d(time_data, RR_states.T, kind='linear', axis=-1)
        self.rl = interpolate.interp1d(time_data, RL_states.T, kind='linear', axis=-1)

    def __init_motor_cmd(self):
        self.motor_cmds = [MotorCmd() for _ in range(12)]

        for i in range(4):
            self.motor_cmds[i*3+0].mode = 10
            self.motor_cmds[i*3+0].Kp = 5 # 70 in gazebo
            self.motor_cmds[i*3+0].dq = 0
            self.motor_cmds[i*3+0].Kd = 1 # 3 in gazebo
            self.motor_cmds[i*3+0].tau = 0

            self.motor_cmds[i*3+1].mode = 10
            self.motor_cmds[i*3+1].Kp = 5 # 180
            self.motor_cmds[i*3+1].dq = 0
            self.motor_cmds[i*3+1].Kd = 1 # 8
            self.motor_cmds[i*3+1].tau = 0

            self.motor_cmds[i*3+2].mode = 10
            self.motor_cmds[i*3+2].Kp = 5 # 300
            self.motor_cmds[i*3+2].dq = 0
            self.motor_cmds[i*3+2].Kd = 1 # 15
            self.motor_cmds[i*3+2].tau = 0


    def __init_joint_cmd_pubs(self):
        robot_name = "a1"

        self.pubs = []
        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FR_hip_controller/command', MotorCmd, queue_size=1))
        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FR_thigh_controller/command', MotorCmd, queue_size=1))
        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FR_calf_controller/command', MotorCmd, queue_size=1))

        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FL_hip_controller/command', MotorCmd, queue_size=1))
        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FL_thigh_controller/command', MotorCmd, queue_size=1))
        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/FL_calf_controller/command', MotorCmd, queue_size=1))

        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RR_hip_controller/command', MotorCmd, queue_size=1))
        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RR_thigh_controller/command', MotorCmd, queue_size=1))
        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RR_calf_controller/command', MotorCmd, queue_size=1))

        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RL_hip_controller/command', MotorCmd, queue_size=1))
        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RL_thigh_controller/command', MotorCmd, queue_size=1))
        self.pubs.append(rospy.Publisher('/' + robot_name + '_gazebo/RL_calf_controller/command', MotorCmd, queue_size=1))

    def __start_flag_sub(self, data):
        self.start_interp = data.data
        if self.start_interp: self.reset()

    def __interp_motion(self, event):
        if self.start_interp:
            self.curr_time = np.clip(self.curr_time， 0, self.time_span)
            fr_cmd = self.fr(self.curr_time)
            fl_cmd = self.fl(self.curr_time)
            rr_cmd = self.rr(self.curr_time)
            rl_cmd = self.rl(self.curr_time)
            self.curr_time += 1/self.desired_freq

            joint_state_commands = np.vstack((fr_cmd, fl_cmd, rr_cmd, rl_cmd))
            self.__pub_joint_state_cmd(joint_state_commands)

    def __pub_joint_state_cmd(self, joint_state_commands):
        for i in range(12):
            motor_cmds[i].q = joint_state_commands[i]
            pubs[i].publish(motor_cmds[i])


if __name__ == '__main__':
    rospy.init_node('cmd_publisher')
    cp = CmdPublisher(file_name='results.json')
    rospy.spin()
