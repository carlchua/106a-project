import casadi as ca
from a1_kinematics import A1Kinematics
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pickle
import json

def cs2np(asd):
	return ca.Function("temp",[],[asd])()["o0"].toarray()

class A1KinematicsOpti:
	def __init__(self):
		self.opti = ca.Opti()
		self.anim_fps = 24
		self.a1_kin = A1Kinematics()
		self.ef = 'FR'
		# state
		self.input_num = 6 # velocity on base xdot ydot ... for the base
		self.base_pose_num = 6 # x y z r p y
		self.FR_state_num = 3 # joint pos 3DoF
		self.FR_vel_num = 3
		self.FL_state_num = 3
		self.FL_vel_num = 3
		self.RR_state_num = 3
		self.RR_vel_num = 3
		self.RL_state_num = 3
		self.RL_vel_num = 3
		self.lambda_num = 3
		self.total_grid = 50 # knot num of traj opti
		self.min_time = 4
		self.vel_bool = False

		# weights of the cost
		# dx dy dz droll dpitch dyaw
		self.w_vel = np.array([5,5,5,5,5,5])
		# diff_x ,diff_y, diff_z, cos_diff_roll, cos_diff_pitch, cos_diff_yaw
		self.w_final = np.array([500,500,500,100,100,100])
		self.w_ef = np.array([50,50,250,10,10,10])
		self.w_ef_final = np.array([500,500,2000,100,100,100])
		self.w_time = 10
		self.w_rpy = np.array([5, 5, 5])

		#added weights for midpoint
		self.w_mid_vel = -1

		# self.ipopt_option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
		#self.ipopt_option = {"max_iter": 100}
		self.ipopt_option = None

	def reset(self):
		self.opti = ca.Opti()
		if self.ipopt_option:
			self.opti.solver("ipopt", self.ipopt_option)
		else:
			self.opti.solver("ipopt")
		self.total_cost = 0
		#self.time_span = 15
		self.time_span = self.opti.variable(1)
		self.base_poses = self.opti.variable(self.base_pose_num, self.total_grid)
		self.FR_states = self.opti.variable(self.FR_state_num, self.total_grid)
		self.FR_vels = self.opti.variable(self.FR_vel_num, self.total_grid)
		self.FL_states = self.opti.variable(self.FL_state_num, self.total_grid)
		self.FL_vels = self.opti.variable(self.FL_vel_num, self.total_grid)
		self.RR_states = self.opti.variable(self.RR_state_num, self.total_grid)
		self.RR_vels = self.opti.variable(self.RR_vel_num, self.total_grid)
		self.RL_states = self.opti.variable(self.RL_state_num, self.total_grid)
		self.RL_vels = self.opti.variable(self.RL_vel_num, self.total_grid)
		self.lambdas = self.opti.variable(self.lambda_num, self.total_grid)
		self.inputs = self.opti.variable(self.input_num, self.total_grid-1)
		self.opti.subject_to(self.time_span >= self.min_time)
		#self.opti.subject_to(self.time_span <= 12)

		self.base_vels = self.opti.variable(self.base_pose_num, self.total_grid)

	def __addNodeVelCost(self, dx):
		self.node_vel_cost = ca.mtimes(self.w_vel.reshape((1, 6)), dx ** 2)
		self.total_cost += self.node_vel_cost

	def __addNodeRPYCost(self, foot_pose, target):
		rot_diff = ca.vec(1 - ca.cos(foot_pose[3:] - target[3:]))
		self.total_cost += ca.mtimes(self.w_rpy.reshape((1, 3)), rot_diff)

	def __addBaseNodeCost(self, final_sate, t):
		# TODO: add orientation cost to
		#ef_pose = self.get_end_effector_pos(-1)
		#pos_diff = ca.vec(ef_pose[[0,1,2]] - final_sate[[0,1,2]])**2
		#rot_diff = ca.vec(1 - ca.cos(ef_pose[[3,4,5], -1] - final_sate[[3,4,5]]))
		#self.final_cost = ca.mtimes(self.w_final.reshape((1,6)), pos_diff)
		#self.total_cost += self.final_cost
		pos_diff = ca.vec(self.base_poses[[0,1,2], t] - final_sate[[0,1,2]])**2
		rot_diff = ca.vec(1 - ca.cos(self.base_poses[[3,4,5], t] - final_sate[[3,4,5]]))
		self.final_cost = ca.mtimes(self.w_final.reshape((1,6)), ca.vertcat(pos_diff, rot_diff))
		self.total_cost += self.final_cost

	def __addEFNodeCost(self, target_state, t, w):
		pos_diff = ca.vec(self.get_end_effector_pos(t)[:3] - target_state[[0,1,2]])**2
		#rot_diff = ca.vec(1 - ca.cos(self.get_end_effector_pos(t)[3:] - target_state[[3,4,5]]))
		rot_diff = ca.vec([0, 0, 0])
		self.final_ef_cost = ca.mtimes(w.reshape((1,6)), ca.vertcat(pos_diff, rot_diff))
		self.total_cost += self.final_ef_cost

	###############
	def __addEFVelocityCost(self, base_pose, FR_motors, target_velocity, t, w):
		fk_position_jacobian, fk_rotation_jacobian, fk_dual_quaternion_jacobian = self.a1_kin.get_FR_jacobian(self.base_poses[:, t], self.FR_states[:, t])

		FR_vels = self.FR_vels
		FR_states = self.FR_states
		q = ca.vertcat(base_pose, FR_motors)

		# Find velocity using Jacobian
		jacobian_pos = fk_position_jacobian(q)
		current_velocity = ca.mtimes(jacobian_pos, ca.vertcat(self.base_vels[:, t], FR_vels[:, t]))

		rot_diff = ca.vec([0, 0, 0])
		vel_diff = ca.vec(current_velocity - target_velocity)**2

		self.final_ef_vel_cost = ca.mtimes(w.reshape((1,6)), ca.vertcat(vel_diff, rot_diff))
		self.total_cost += self.final_ef_vel_cost
		# else:
		# 	curr_vel_mag = np.sqrt(current_velocity[0]**2 + current_velocity[1]**2 + current_velocity[2]**2)
		# 	self.total_cost += w*curr_vel_mag
	###############

	def __addTotalTimeCost(self):
		self.total_cost += self.w_time * self.time_span

	def visualize_robot_single_frame(self, base_rot, motor_pos):
		self.a1_kin.visualize_robot(base_rot, motor_pos)

	def set_init_base_pose(self,init_base_pose):
		self.init_base_pose = init_base_pose

	def set_init_ef_pos(self,init_ef_pos):
		self.init_ef_pos = init_ef_pos

	def set_vel_bool(self, bool_val):
		self.vel_bool = bool_val

	def set_init_ef_vel(self,init_ef_vel):
		self.init_ef_vel = init_ef_vel

	def set_final_ef_pos(self,final_ef_pos):
		self.final_ef_pos = final_ef_pos

	def set_final_ef_vel(self,final_ef_vel):
		self.final_ef_vel = final_ef_vel

	def set_FR_pos(self,FR_pos):
		self.des_FR_pos = FR_pos

	def set_FL_pos(self,FL_pos):
		self.des_FL_pos = FL_pos

	def set_RR_pos(self,RR_pos):
		self.des_RR_pos = RR_pos

	def set_RL_pos(self,RL_pos):
		self.des_RL_pos = RL_pos

	def set_min_time(self, min_t):
		self.min_time = min_t

	# ef could be set to either 'FR' or 'FL'
	def set_end_effector(self, ef):
		if ef not in ['FR', 'FL']:
			print("illegal end effector config")
			return
		self.ef = ef

	def get_foot_pos(self, foot, t):
		if foot == 'FR':
			pos, R = self.a1_kin.get_FR_pose(self.base_poses[:, t], self.FR_states[:, t])
		elif foot == 'FL':
			pos, R = self.a1_kin.get_FL_pose(self.base_poses[:, t], self.FL_states[:, t])
		elif foot == 'RR':
			pos, R = self.a1_kin.get_RR_pose(self.base_poses[:, t], self.RR_states[:, t])
		elif foot == 'RL':
			pos, R = self.a1_kin.get_RL_pose(self.base_poses[:, t], self.RL_states[:, t])
		else:
			print("Illegal foot, defaulting to RR")
			pos, R = self.a1_kin.get_RR_pose(self.base_poses[:, t], self.RR_states[:, t])
		eft = ca.vertcat(pos, ca.atan2(R[1,0], R[0,0]), ca.atan2(-R[2,0], ca.sqrt(R[2,1]**2 + R[2,2]**2)), ca.atan2(R[2,1], R[2,2]))
		return eft

	def get_end_effector_pos(self, t):
		if self.ef == 'FR':
			pos, R = self.a1_kin.get_FR_pose(self.base_poses[:, t], self.FR_states[:, t])
		else:
			pos, R = self.a1_kin.get_FL_pose(self.base_poses[:, t], self.FL_states[:, t])
		#rpy = ca.MX.sym('rpy', 3, 1)
		#rpy[0] = ca.atan2(R[1,0], R[0,0])
		#rpy[1] = ca.atan2(-R[2,0], ca.sqrt(R[2,1]**2 + R[2,2]**2))
		#rpy[2] = ca.atan2(R[2,1], R[2,2])
		eft = ca.vertcat(pos, ca.atan2(R[1,0], R[0,0]), \
			ca.atan2(-R[2,0], ca.sqrt(R[2,1]**2 + R[2,2]**2)), \
				ca.atan2(R[2,1], R[2,2]))
		return eft

	def trans_vel2ef(self, t):
		def skewsym(vec):
			r0 = ca.horzcat(0, -vec[2], vec[1])
			r1 = ca.horzcat(vec[2], 0, -vec[0])
			r2 = ca.horzcat(-vec[1], vec[0], 0)
			return ca.vertcat(r0, r1, r2)

		if self.ef == 'FR':
			p, R = self.a1_kin.get_FR_pose(self.base_poses[:, t], self.FR_states[:, t])
		else:
			p, R = self.a1_kin.get_FL_pose(self.base_poses[:, t], self.FL_states[:, t])
		p_hat_R = ca.mtimes(skewsym(p), R)
		top = ca.horzcat(R, p_hat_R)
		vec0 = ca.horzcat(0, 0, 0)
		bottom = ca.horzcat(ca.vertcat(vec0, vec0, vec0), R)
		T = ca.vertcat(top, bottom)
		return ca.mtimes(T, self.inputs[:, t])

	def solve(self):
		self.reset()
		#self.__addBaseNodeCost(self.init_base_pose, 0)
		#self.__addEFNodeCost(self.init_ef_pos, 0)
		self.slack_final = self.opti.variable(6)
		self.total_cost += ca.mtimes(np.array([10000, 10000, 10000]).reshape((1, 3)), ca.vec(self.slack_final[:3])**2)
		self.total_cost += ca.mtimes(np.array([10000, 10000, 10000]).reshape((1, 3)), ca.vec(1 - ca.cos(self.slack_final[3:])))
		self.opti.subject_to(self.base_poses[:, 0] == self.init_base_pose) # initial condition
		self.opti.subject_to(self.get_end_effector_pos(0)[:3] == self.init_ef_pos[:3]) # initial condition for end effecotor of the swing leg
		#print("init_ef_pos", self.init_ef_pos)
		#self.opti.subject_to(self.get_end_effector_pos(-1) + self.slack_final == self.final_ef_pos)

		deltaT = self.time_span / (self.total_grid - 1)
		last_base_vel = [0,0,0,0,0,0] # start from zero velocity
		last_FR_vel = [0,0,0]
		last_FL_vel = [0,0,0]
		last_RR_vel = [0,0,0]
		last_RL_vel = [0,0,0]
		for i in range(self.total_grid):
			base_k = self.base_vels[:,i]
			self.opti.subject_to(base_k < np.array([10, 10, 10, 10, 10, 10]))
			base_k = self.base_poses[:,i]
			ef_k = self.get_end_effector_pos(i)
			FR_k = self.FR_states[:,i]
			FL_k = self.FL_states[:,i]
			RR_k = self.RR_states[:,i]
			RL_k = self.RL_states[:,i]
			# basic joint limits
			self.opti.subject_to(FR_k <= self.a1_kin.joint_uppers)
			self.opti.subject_to(FL_k <= self.a1_kin.joint_uppers)
			self.opti.subject_to(RR_k <= self.a1_kin.joint_uppers)
			self.opti.subject_to(RL_k <= self.a1_kin.joint_uppers)
			self.opti.subject_to(FR_k >= self.a1_kin.joint_lowers)
			self.opti.subject_to(FL_k >= self.a1_kin.joint_lowers)
			self.opti.subject_to(RR_k >= self.a1_kin.joint_lowers)
			self.opti.subject_to(RL_k >= self.a1_kin.joint_lowers)
			# foot pos constrain
			FR_pos_k = self.get_foot_pos('FR', i)
			FL_pos_k = self.get_foot_pos('FL', i)
			RR_pos_k = self.get_foot_pos('RR', i)
			RL_pos_k = self.get_foot_pos('RL', i)
			# fix 3 feet position
			self.opti.subject_to(RR_pos_k[:3] == self.des_RR_pos[:3])
			#self.__addNodeRPYCost(RR_pos_k, self.des_RR_pos)
			self.opti.subject_to(RL_pos_k[:3] == self.des_RL_pos[:3])
			#self.__addNodeRPYCost(RL_pos_k, self.des_RL_pos)
			if self.ef == 'FR':
				self.opti.subject_to(FL_pos_k[:3] == self.des_FL_pos[:3])
				#self.__addNodeRPYCost(FL_pos_k, self.des_FL_pos)
			else:
				self.opti.subject_to(FR_pos_k[:3] == self.des_FR_pos[:3])
				#self.__addNodeRPYCost(FR_pos_k, self.des_FR_pos)
			# end effector cannot be underground
			# if self.ef == 'FR':
			# 	self.opti.subject_to(FR_pos_k[2] >= self.init_ef_pos[2])
			# else:
			# 	self.opti.subject_to(FL_pos_k[2] >= self.init_ef_pos[2])
			#non-falling constraint
			x_RR_k = RR_pos_k[:2]
			x_RL_k = RL_pos_k[:2]
			if self.ef == 'FR':
				x_fixed_k = FL_pos_k[:2]
			else:
				x_fixed_k = FR_pos_k[:2]
			triangle_mat = ca.hcat([x_RR_k,  x_RL_k, x_fixed_k])
			self.opti.subject_to(
				ca.vertcat(base_k[0], base_k[1]) == ca.mtimes(triangle_mat, self.lambdas[:, i])) # not falling constraints
			self.opti.subject_to(self.lambdas[:, i] >= 0)
			self.opti.subject_to(ca.sum1(self.lambdas[:, i]) == 1)

			#self.__addEFNodeCost(self.final_ef_pos, i, self.w_ef)
			# add input & smoothin the traj
			if i is not self.total_grid-1:
				base_vel_k = self.inputs[:,i]
				base_knext = self.base_poses[:, i+1]
				base_p = base_k + (base_vel_k + last_base_vel) * deltaT / 2.0 # trapezodial method to do collocation on base dynamics
				self.opti.subject_to(base_knext == base_p)
				self.__addNodeVelCost(base_vel_k)
				last_base_vel = base_vel_k

				# collocation on the swing leg
				if self.ef == 'FR':
					FR_vel_k = self.FR_vels[:,i]
					self.opti.subject_to(FR_vel_k <= self.a1_kin.joint_vel_uppers)
					self.opti.subject_to(FR_vel_k >= self.a1_kin.joint_vel_lowers)
					FR_knext = self.FR_states[:, i+1]
					FR_p = FR_k + (FR_vel_k + last_FR_vel) * deltaT / 2.0
					self.opti.subject_to(FR_knext == FR_p)
					last_FR_vel = FR_vel_k
				elif self.ef == 'FL':
					FL_vel_k = self.FL_vels[:,i]
					self.opti.subject_to(FL_vel_k <= self.a1_kin.joint_vel_uppers)
					self.opti.subject_to(FL_vel_k >= self.a1_kin.joint_vel_lowers)
					FL_knext = self.FL_states[:, i+1]
					FL_p = FL_k + (FL_vel_k + last_FL_vel) * deltaT / 2.0
					self.opti.subject_to(FL_knext == FL_p)
					last_FL_vel = FL_vel_k
				else:
					print('illegal end effector')

		self.__addTotalTimeCost()

		#Final time
		self.__addBaseNodeCost(self.init_base_pose, -1) # force the base to move as little as possible
		self.__addEFNodeCost(self.final_ef_pos, -1, self.w_ef_final) # NOTE: change the target state constraint to be soccer one
		# NOTE: add constraint to force the end effector velocity at the end node to be a desired one
		# NOTE: try traget state as constraint

		# self.opti.subject_to(self.final_ef_pos == self.desired_ef_pos + final_node_slacking)

		# Set init velocity if second loop
		if not self.vel_bool:
			self.__addEFVelocityCost(self.base_poses[:, 0], self.FR_states[:, 0], self.init_ef_vel, 0, self.w_vel)
			self.__addEFVelocityCost(self.base_poses[:, -1], self.FR_states[:, -1], self.final_ef_vel, -1, self.w_vel)

		# self.total_cost += large_weight*final_node_slacking**2
		# self.opti.subject_to(self.final_ef_vel == self.desired_ef_vel + final_vel_slacking)
		# self.total_cost += large_weight*final_vel_slacking**2
		# self.opti.subject_to(self.pelvis_states[:,-1]==self.final_pelvis_pose)
		self.opti.minimize(self.total_cost)

		sol = self.opti.solve()
		base_sol = sol.value(self.base_poses)
		RR_joint_sol = sol.value(self.RR_states)
		RL_joint_sol = sol.value(self.RL_states)
		FL_joint_sol = sol.value(self.FL_states)
		FR_joint_sol = sol.value(self.FR_states)
		input_sol = sol.value(self.inputs)
		#time_sol = sol.value(self.time_span)

		return base_sol, FR_joint_sol, FL_joint_sol, RR_joint_sol, RL_joint_sol, sol.value(self.FR_vels), sol.value(self.base_vels)

def compute_actual_t(a1_kin_opti, foot, t):
	basepose = a1_kin_opti.opti.debug.value(a1_kin_opti.base_poses)[:, t]
	if foot == 'FR':
		motors = a1_kin_opti.opti.debug.value(a1_kin_opti.FR_states)[:, t]
		pos, R = a1_kin_opti.a1_kin.get_FR_pose(basepose, motors)
	elif foot == 'FL':
		motors = a1_kin_opti.opti.debug.value(a1_kin_opti.FL_states)[:, t]
		pos, R = a1_kin_opti.a1_kin.get_FL_pose(basepose, motors)
	elif foot == 'RR':
		motors = a1_kin_opti.opti.debug.value(a1_kin_opti.RR_states)[:, t]
		pos, R = a1_kin_opti.a1_kin.get_RR_pose(basepose, motors)
	else:
		motors = a1_kin_opti.opti.debug.value(a1_kin_opti.RL_states)[:, t]
		pos, R = a1_kin_opti.a1_kin.get_RL_pose(basepose, motors)
	eft = ca.vertcat(pos, ca.atan2(R[1,0], R[0,0]), ca.atan2(-R[2,0], ca.sqrt(R[2,1]**2 + R[2,2]**2)), ca.atan2(R[2,1], R[2,2]))
	return eft

def plot_traj_post_solve(a1_kin_opti):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	FR_xs = np.zeros(a1_kin_opti.total_grid)
	FL_xs = np.zeros(a1_kin_opti.total_grid)
	RR_xs = np.zeros(a1_kin_opti.total_grid)
	RL_xs = np.zeros(a1_kin_opti.total_grid)
	FR_ys = np.zeros(a1_kin_opti.total_grid)
	FL_ys = np.zeros(a1_kin_opti.total_grid)
	RR_ys = np.zeros(a1_kin_opti.total_grid)
	RL_ys = np.zeros(a1_kin_opti.total_grid)
	FR_zs = np.zeros(a1_kin_opti.total_grid)
	FL_zs = np.zeros(a1_kin_opti.total_grid)
	RR_zs = np.zeros(a1_kin_opti.total_grid)
	RL_zs = np.zeros(a1_kin_opti.total_grid)
	base_xs = np.zeros(a1_kin_opti.total_grid)
	base_ys = np.zeros(a1_kin_opti.total_grid)
	base_zs = np.zeros(a1_kin_opti.total_grid)
	for i in range(a1_kin_opti.total_grid):
		FR_t = compute_actual_t(a1_kin_opti, 'FR', i).full()
		FL_t = compute_actual_t(a1_kin_opti, 'FL', i).full()
		RR_t = compute_actual_t(a1_kin_opti, 'RR', i).full()
		RL_t = compute_actual_t(a1_kin_opti, 'RL', i).full()
		FR_xs[i], FR_ys[i], FR_zs[i] = FR_t[0, 0], FR_t[1, 0], FR_t[2, 0]
		FL_xs[i], FL_ys[i], FL_zs[i] = FL_t[0, 0], FL_t[1, 0], FL_t[2, 0]
		RR_xs[i], RR_ys[i], RR_zs[i] = RR_t[0, 0], RR_t[1, 0], RR_t[2, 0]
		RL_xs[i], RL_ys[i], RL_zs[i] = RL_t[0, 0], RL_t[1, 0], RL_t[2, 0]
		base_xs[i] = a1_kin_opti.opti.debug.value(a1_kin_opti.base_poses)[0, i]
		base_ys[i] = a1_kin_opti.opti.debug.value(a1_kin_opti.base_poses)[1, i]
		base_zs[i] = a1_kin_opti.opti.debug.value(a1_kin_opti.base_poses)[2, i]
	ax.scatter(FR_xs, FR_ys, FR_zs, marker='^')
	ax.scatter(FL_xs, FL_ys, FL_zs, marker='o')
	ax.scatter(RR_xs, RR_ys, RR_zs, marker='o')
	ax.scatter(RL_xs, RL_ys, RL_zs, marker='o')
	ax.scatter(base_xs, base_ys, base_zs, marker='d')
	plt.show()

def get_motor_i(results, i):
	print(results['FR_states'][0][i], results['FR_states'][1][i], results['FR_states'][2][i])
	print(results['FL_states'][0][i], results['FL_states'][1][i], results['FL_states'][2][i])
	print(results['RR_states'][0][i], results['RR_states'][1][i], results['RR_states'][2][i])
	print(results['RL_states'][0][i], results['RL_states'][1][i], results['RL_states'][2][i])

if __name__ == "__main__":
	a1_kin_opti = A1KinematicsOpti()
	init_base_pose = np.array([0,0,0.3,0,0,0])
	a1_kin_opti.set_init_base_pose(init_base_pose)
	a1_kin_opti.set_end_effector('FR')

	# Initial position for first loop
	init_ef_pos = np.array([0.183, -0.13205, 0, 0, 0, 0])
	a1_kin_opti.set_init_ef_pos(init_ef_pos)

	# Final location for first loop
	final_ef_pos = np.array([0.1, -0.1, 0.2, 0, -0.5*np.pi, 0]) #small lift
	a1_kin_opti.set_final_ef_pos(final_ef_pos)
	final_ef_vel = np.array([0, 0, 0])
	a1_kin_opti.set_final_ef_vel(final_ef_vel)
	a1_kin_opti.set_vel_bool(True)

	# Set other 3 legs init pos
	a1_kin_opti.set_FL_pos(np.array([0.183, 0.13205, 0, 0, 0, 0]))
	a1_kin_opti.set_RR_pos(np.array([-0.183, -0.13205, 0, 0, 0, 0]))
	a1_kin_opti.set_RL_pos(np.array([-0.183, 0.13205, 0, 0, 0, 0]))

	# try:
	base_sol, FR_joint_sol, FL_joint_sol, RR_joint_sol, RL_joint_sol, base_vels, fr_vels = a1_kin_opti.solve()
	motor_last = np.concatenate([FR_joint_sol[:,-1], FL_joint_sol[:,-1], RR_joint_sol[:,-1], RL_joint_sol[:,-1]])
	base_last = base_sol[:,-1]

############################ SECOND LOOP #######################################
	a1_kin_opti_next = A1KinematicsOpti()
	init_base_pose2 = base_sol[:, -1]

	# Jacobian
	fr_joints = FR_joint_sol[:, -1]
	fk_position_jacobian, fk_rotation_jacobian, fk_dual_quaternion_jacobian = a1_kin_opti.a1_kin.get_FR_jacobian(init_base_pose2, fr_joints)
	q = ca.vertcat(base_sol[:, -1], fr_joints)
	jacobian_pos = fk_position_jacobian(q)
	prev_vel = np.matmul(np.array(jacobian_pos), np.hstack((base_vels[:, -1], fr_vels[:, -1])))
	#prev_vel = np.array(prev_vel).reshape(3,)
	#prev_vel = cs2np(prev_vel)
	a1_kin_opti_next.set_init_ef_vel(prev_vel)

	a1_kin_opti_next.set_init_base_pose(init_base_pose2)
	a1_kin_opti_next.set_end_effector('FR')

	# Set FR init pos
	pos, rot = a1_kin_opti_next.a1_kin.get_FR_pose(base_last, FR_joint_sol[:, -1])
	pos = np.array(pos)
	R = np.array(rot)
	eft = ca.vertcat(pos, ca.atan2(R[1,0], R[0,0]), ca.atan2(-R[2,0], ca.sqrt(R[2,1]**2 + R[2,2]**2)), ca.atan2(R[2,1], R[2,2]))
	eft = np.array(eft).reshape(6,)
	a1_kin_opti_next.set_init_ef_pos(eft)

	# Final location for second loop
	final_ef_pos2 = np.array([0.2, -0.2, 0.3, 0, -0.75*np.pi, 0]) #small lift
	a1_kin_opti_next.set_final_ef_pos(final_ef_pos2)
	final_ef_vel2 = np.array([0, 0, 0])
	a1_kin_opti_next.set_final_ef_vel(final_ef_vel2)
	a1_kin_opti_next.set_vel_bool(False)

	# Set other 3 legs init pos
	pos, rot = a1_kin_opti_next.a1_kin.get_FL_pose(base_last, FL_joint_sol[:, -1])
	pos = np.array(pos)
	R = np.array(rot)
	eft = ca.vertcat(pos, ca.atan2(R[1,0], R[0,0]), ca.atan2(-R[2,0], ca.sqrt(R[2,1]**2 + R[2,2]**2)), ca.atan2(R[2,1], R[2,2]))
	eft = np.array(eft).reshape(6,)
	a1_kin_opti_next.set_FL_pos(eft)

	pos, rot = a1_kin_opti_next.a1_kin.get_RR_pose(base_last, RR_joint_sol[:, -1])
	pos = np.array(pos)
	R = np.array(rot)
	eft = ca.vertcat(pos, ca.atan2(R[1,0], R[0,0]), ca.atan2(-R[2,0], ca.sqrt(R[2,1]**2 + R[2,2]**2)), ca.atan2(R[2,1], R[2,2]))
	eft = np.array(eft).reshape(6,)
	a1_kin_opti_next.set_RR_pos(eft)

	pos, rot = a1_kin_opti_next.a1_kin.get_RL_pose(base_last, RL_joint_sol[:, -1])
	pos = np.array(pos)
	R = np.array(rot)
	eft = ca.vertcat(pos, ca.atan2(R[1,0], R[0,0]), ca.atan2(-R[2,0], ca.sqrt(R[2,1]**2 + R[2,2]**2)), ca.atan2(R[2,1], R[2,2]))
	eft = np.array(eft).reshape(6,)
	a1_kin_opti_next.set_RL_pos(eft)

	# try:
	base_sol2, FR_joint_sol2, FL_joint_sol2, RR_joint_sol2, RL_joint_sol2, _, _ = a1_kin_opti_next.solve()
	motor_last2 = np.concatenate([FR_joint_sol2[:,-1], FL_joint_sol2[:,-1], RR_joint_sol2[:,-1], RL_joint_sol2[:,-1]])
	base_last2 = base_sol2[:,-1]


	print('FR0:', compute_actual_t(a1_kin_opti, 'FR', 0))
	print('FR:', compute_actual_t(a1_kin_opti, 'FR', -1))
	print('FL:', compute_actual_t(a1_kin_opti, 'FL', -1))
	print('RR:', compute_actual_t(a1_kin_opti, 'RR', -1))
	print('RL:', compute_actual_t(a1_kin_opti, 'RL', -1))
	print(a1_kin_opti.opti.debug.value(a1_kin_opti.time_span))
	plot_traj_post_solve(a1_kin_opti)
	plot_traj_post_solve(a1_kin_opti_next)
	results = {}
	resultsjson = {}
	results['base_poses'] = a1_kin_opti.opti.debug.value(a1_kin_opti.base_poses)
	results['FR_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.FR_states)
	results['FL_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.FL_states)
	results['RR_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.RR_states)
	results['RL_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.RL_states)

	resultsjson['base_poses'] = a1_kin_opti.opti.debug.value(a1_kin_opti.base_poses).tolist()
	resultsjson['FR_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.FR_states).tolist()
	resultsjson['FL_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.FL_states).tolist()
	resultsjson['RR_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.RR_states).tolist()
	resultsjson['RL_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.RL_states).tolist()

	results['time_span'] = a1_kin_opti.opti.debug.value(a1_kin_opti.time_span)
	results['total_grid'] = a1_kin_opti.total_grid
	f = open('test_interp.p', 'w')
	pickle.dump(results, f)
	f.close()
	with open('results.json', 'w') as fp:
		json.dump(resultsjson, fp)
	get_motor_i(results, 0)
	# except RuntimeError:
	# 	print('FR0:', compute_actual_t(a1_kin_opti, 'FR', 0))
	# 	print('FR:', compute_actual_t(a1_kin_opti, 'FR', -1))
	# 	print('FL:', compute_actual_t(a1_kin_opti, 'FL', -1))
	# 	print('RR:', compute_actual_t(a1_kin_opti, 'RR', -1))
	# 	print('RL:', compute_actual_t(a1_kin_opti, 'RL', -1))
	# 	print(a1_kin_opti.opti.debug.value(a1_kin_opti.time_span))
	# 	results = {}
	# 	results['base_poses'] = a1_kin_opti.opti.debug.value(a1_kin_opti.base_poses)
	# 	results['FR_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.FR_states)
	# 	results['FL_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.FL_states)
	# 	results['RR_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.RR_states)
	# 	results['RL_states'] = a1_kin_opti.opti.debug.value(a1_kin_opti.RL_states)

	# 	results['time_span'] = a1_kin_opti.opti.debug.value(a1_kin_opti.time_span)
	# 	results['total_grid'] = a1_kin_opti.total_grid
	# 	plot_traj_post_solve(a1_kin_opti)
