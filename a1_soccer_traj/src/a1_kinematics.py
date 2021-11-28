import casadi as ca
from urdf2casadi import urdfparser as u2c
#import kinpy as kp
import numpy as np

class A1Kinematics():
    def __init__(self):
        self.urdf = "../urdf/a1.urdf"
	#self.default_chain = kp.build_chain_from_urdf(open("/home/daly/guidedog_ws/src/unitree_ros/robots/a1_description/urdf/a1_stl.urdf").read())
        self.robot_parser = u2c.URDFparser()
        self.robot_parser.from_file(self.urdf)
        self.FR_fk_dict = self.robot_parser.get_forward_kinematics("world", "FR_foot")
        self.FL_fk_dict = self.robot_parser.get_forward_kinematics("world", "FL_foot")
        self.RR_fk_dict = self.robot_parser.get_forward_kinematics("world", "RR_foot")
        self.RL_fk_dict = self.robot_parser.get_forward_kinematics("world", "RL_foot")
        self.joint_uppers = np.array(self.FR_fk_dict['upper'])[6:]
        self.joint_lowers = np.array(self.FR_fk_dict['lower'])[6:]
        self.joint_uppers[0] = 0.4
        self.joint_uppers[1] = 2
        self.joint_lowers[0] = -0.4
        self.FR_T = self.FR_fk_dict['T_fk']
        self.FL_T = self.FL_fk_dict['T_fk']
        self.RR_T = self.RR_fk_dict['T_fk']
        self.RL_T = self.RL_fk_dict['T_fk'] # transformation matrix from world to EF
        #self.joint_vel_uppers = np.array([52.4, 28.6, 28.6])
        #self.joint_vel_lowers = np.array([-52.4, -28.6, -28.6])
        #self.joint_vel_uppers = np.array([np.pi, np.pi, np.pi])
        #self.joint_vel_lowers = np.array([-np.pi, -np.pi, -np.pi])
        self.joint_vel_uppers = np.array([np.pi/8, np.pi/8, np.pi/8])
        self.joint_vel_lowers = np.array([-np.pi/8, -np.pi/8, -np.pi/8])

    def cs2np(asd):
        return ca.Function("temp",[],[asd])()["o0"].toarray()

    def get_FR_pose(self, base_pose, FR_motors):
        q = ca.vertcat(base_pose, FR_motors)
        T0 = self.FR_T(q)
        pos = T0[:3, 3]
        rot = T0[:3, :3]
        return pos, rot
    
    def get_FL_pose(self, base_pose, FL_motors):
        q = ca.vertcat(base_pose, FL_motors)
        T0 = self.FL_T(q)
        pos = T0[:3, 3]
        rot = T0[:3, :3]
        return pos, rot

    def get_RR_pose(self, base_pose, RR_motors):
        q = ca.vertcat(base_pose, RR_motors)
        T0 = self.RR_T(q)
        pos = T0[:3, 3]
        rot = T0[:3, :3]
        return pos, rot
    
    def get_RL_pose(self, base_pose, RL_motors):
        q = ca.vertcat(base_pose, RL_motors)
        T0 = self.RL_T(q)
        pos = T0[:3, 3]
        rot = T0[:3, :3]
        return pos, rot

    # TODO: get jacobain of the end effector
    # see template from urdf2casadi
    # https://github.com/mahaarbo/urdf2casadi/blob/master/examples/kinematics/UR5_urdf_example.ipynb

    def get_FR_jacobian(self, base_pose, FR_motors):
        q = self.FR_fk_dict['q']
        # q = ca.vertcat(base_pose, FR_motors)
        T_fk = self.FR_fk_dict['T_fk']

        # Create symbols
        fk_position_jacobian_sym = ca.jacobian(T_fk(q)[:3,3], q)
        # fk_rotation_jacobian_sym = ca.jacobian(self.get_FR_pose(q)[:3,:3], q)
        # fk_dual_quaternion_jacobian_sym = ca.jacobian(self.get_FR_pose(q), q)
        fk_rotation_jacobian_sym = 0
        fk_dual_quaternion_jacobian_sym = 0

        # Create functions
        fk_position_jacobian = ca.Function("jac_fk_pos", [q], [fk_position_jacobian_sym], ["q"], ["jac_fk_pos"])
        # fk_rotation_jacobian = ca.Function("jac_fk_rot", [q], [fk_rotation_jacobian_sym], ["q"], ["jac_fk_rot"])
        # fk_dual_quaternion_jacobian = ca.Function("jac_fk_Q", [q], [fk_dual_quaternion_jacobian_sym], ["q"], ["jac_fk_Q"])
        fk_rotation_jacobian = 0
        fk_dual_quaternion_jacobian = 0

        return fk_position_jacobian, fk_rotation_jacobian, fk_dual_quaternion_jacobian
    
    # def visualize_robot(self, motor_pos=None):
    #     if motor_pos is not None:
    #         th = {'FR_hip_joint':motor_pos[0], 'FR_thigh_joint':motor_pos[1], 'FR_calf_joint':motor_pos[2], 
    #             'FL_hip_joint':motor_pos[3], 'FL_thigh_joint':motor_pos[4], 'FL_calf_joint':motor_pos[5], 
    #             'RR_hip_joint':motor_pos[6], 'RR_thigh_joint':motor_pos[7], 'RR_calf_joint':motor_pos[8], 
    #             'RL_hip_joint':motor_pos[9], 'RL_thigh_joint':motor_pos[10], 'RL_calf_joint':motor_pos[11]}
    #     else:
    #         th = {}
    #     ret = self.default_chain.forward_kinematics(th)
    #     viz = kp.Visualizer()
    #     viz.add_robot(ret, self.default_chain.visuals_map(),axes=True)
    #     viz.spin()

if __name__ == "__main__":
    ak = A1Kinematics()
    #ak.visualize_robot([0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3])
    print("FR:", ak.FR_T([.0, .0, .3, .0, .0, .0, -0.037248236447189775, 0.7159849301283057, -1.4606433108501058])[[0, 1, 2], 3].full().flatten())
    print("FL:", ak.FL_T([.0, .0, .3, .0, .0, .0, -0.07316931885328248, 0.7270735689669223, -1.4562430870218819])[[0, 1, 2], 3])
    print("RR:", ak.RR_T([.0, .0, .3, .0, .0, .0, 0.07599912707136625, 0.7298351970704555, -1.4556752845279597])[[0, 1, 2], 3])
    print("RL:", ak.RL_T([.0, .0, .3, .0, .0, .0, -0.0879768014279314, 0.7140789543799659, -1.462480147990135])[[0, 1, 2], 3])
    print(ak.joint_uppers)
    print(ak.joint_lowers)

    print("FR:", ak.FR_T([5.33124525e-03, 1.40476550e-04, 2.96701968e-01, 2.21779981e-02, -2.95332678e-03, -4.35811163e-01, 0.34513559, 0.71611916, -1.5943171])[[0, 1, 2], 3].full().flatten())
    print("FL:", ak.FL_T([5.33124525e-03, 1.40476550e-04, 2.96701968e-01, 2.21779981e-02, -2.95332678e-03, -4.35811163e-01, 0.10806939, 0.87501563, -1.2839156])[[0, 1, 2], 3])
    print("RR:", ak.RR_T([5.33124525e-03, 1.40476550e-04, 2.96701968e-01, 2.21779981e-02, -2.95332678e-03, -4.35811163e-01, -0.16803905, 0.48412645, -1.32322079])[[0, 1, 2], 3])
    print("RL:", ak.RL_T([5.33124525e-03, 1.40476550e-04, 2.96701968e-01, 2.21779981e-02, -2.95332678e-03, -4.35811163e-01, -0.39999989, 0.85956598, -1.46872909])[[0, 1, 2], 3])


