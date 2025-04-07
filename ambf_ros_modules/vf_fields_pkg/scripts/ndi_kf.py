# maintains RCM pose with a Kalman filter using NDI messages
# need to install crtk in catkin_ws folder and build to use crtk library

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import WrenchStamped, Wrench, Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
import rospy
import crtk, dvrk

class PSM_KF:
    def __init__(self, psmnum, rcm_pub_flag = False):
        self.rcm_pub_flag = rcm_pub_flag
        self.psmnum = psmnum
        self.toolname = "PSM" + str(self.psmnum) + '_kf'
        self.ral = crtk.ral(self.toolname)
        self.arm = dvrk.arm(self.ral)

        # note change of topic name to /NDI/PSM#_kf to support new calibration
        self.topic_dict = {'/NDI/' + self.toolname + '/measured_cp': {'data': None, 'type': PoseStamped, 'arm':'PSM'}}

        self.RCM_pose_posterior = None
        self.RCM_pose_state = Pose()

        
        self.cleft_p = np.array([-0.93998,0.003,1.05936]).reshape(3,1)
        cleft_R = R.from_euler('xyz', [1.85791,-0.2802,2.27472])
        self.cleft_R = cleft_R.as_matrix()

        pad = np.array([0,0,0,1])
        self.cleft_pose = np.hstack((self.cleft_R,self.cleft_p))
        self.cleft_pose = np.vstack((self.cleft_pose, pad))


        # measurement covariance should generally be "bigger" than state covariance, more confident in dvrk measurements
        state_pos_var = 0.01
        state_ang_var = 0.01

        # diagonal Q matrix
        self.Q = np.array([
            [state_pos_var, 0, 0, 0, 0, 0],
            [0, state_pos_var, 0, 0, 0, 0],
            [0, 0, state_pos_var, 0, 0, 0],
            [0, 0, 0, state_ang_var, 0, 0],
            [0, 0, 0, 0, state_ang_var, 0],
            [0, 0, 0, 0, 0, state_ang_var]
            ])
        
        meas_pos_var = 0.03
        meas_ang_var = 0.02
        self.R = np.array([
            [meas_pos_var, 0, 0, 0, 0, 0],
            [0, meas_pos_var, 0, 0, 0, 0],
            [0, 0, meas_pos_var, 0, 0, 0],
            [0, 0, 0, meas_ang_var, 0, 0],
            [0, 0, 0, 0, meas_ang_var, 0],
            [0, 0, 0, 0, 0, meas_ang_var]
            ])
        
        
        self.RCM_pub = rospy.Publisher(self.toolname+'_RCM_pose', Pose, queue_size=10)

    def DH_to_transform(self, DH_params):
        # program assumes DH format in this order
        # angle values should be in radians (alpha and theta)
        # distance values should be in decimetres (a and d)
        a, alpha, d, theta = DH_params
        
        H = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
            ])
        return H


    def calculate_RCM(self, joints):
        # multiply q3 by 10 because ambf in decimetres, dvrk in metres
        q1 = joints[0]
        q2 = joints[1]
        q3 = joints[2]*10

        H_yaw = self.DH_to_transform([0, np.pi/2, 0, q1+np.pi/2])
        H_pitch = self.DH_to_transform([0, -np.pi/2, 0, q2-np.pi/2])
        H_d = self.DH_to_transform([0, np.pi/2, q3, 0])
        
        # intialize RCM state by computing kinematics backwards
        H_PSM_RCM = np.matmul(H_yaw, np.matmul(H_pitch, H_d))
        H_RCM_PSM = np.linalg.inv(H_PSM_RCM)
        H_RCM = np.matmul(self.PSM_pose_homogeneous_posterior, H_RCM_PSM)

        # RCM pose is based off of POSTERIOR state estimate, incorporates info after measurement
        return H_RCM


    def kf_init(self, joints, time, data, joint_vel_now):
        # initialize posterior states, necessary for next iteration of algorithm
        self.PSM_pose_homogeneous_posterior, self.PSM_pose_state_posterior = self.calculate_measurement(data)

        # record start time for future interval calcs
        self.prev_time = time

        # intialize covariance matrix P
        pos_var = 0.1
        ang_var = 0.05
        self.P_posterior = np.array([
            [pos_var, 0, 0, 0, 0, 0],
            [0, pos_var, 0, 0, 0, 0],
            [0, 0, pos_var, 0, 0, 0],
            [0, 0, 0, ang_var, 0, 0],
            [0, 0, 0, 0, ang_var, 0],
            [0, 0, 0, 0, 0, ang_var]
            ])

        # record joint velocities
        self.joint_vel_prev = joint_vel_now

        self.RCM_pose_posterior = self.calculate_RCM(joints)

        self.joints_prev = joints

        return
    

    def calculate_measurement(self, data):
        # grab NDI pose and PSM joint positions and write to class variables

        # note that the kalman state is a 6x1 pose vector 
        # the last 3 numbers are zyz euler angles in radians
        PSM_p = np.array([data.pose.position.x*10,data.pose.position.y*10,data.pose.position.z*10]).reshape(3,1)

        PSM_orientation = R.from_quat([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        
        PSM_pose_homogeneous = np.hstack((PSM_orientation.as_matrix(),PSM_p))
        PSM_pose_homogeneous = np.vstack((PSM_pose_homogeneous, np.array([0,0,0,1])))
        
        # NDI PSM pose is reported relative to cleft, multiply to account for additional transform
        # this is the pose as a homogeneous matrix, necessary for RCM and Jacobian calculations
        PSM_pose_homogeneous = np.matmul(self.cleft_pose,PSM_pose_homogeneous)

        PSM_orientation = R.from_matrix(PSM_pose_homogeneous[0:3, 0:3])

        # this is the pose in a 6x1 vector, this form is the state of the kalman filter
        PSM_pose_state = np.vstack((PSM_pose_homogeneous[0:3, 3], PSM_orientation.as_euler('zyz').reshape(3,1)))

        # return pose in matrix and state form
        return PSM_pose_homogeneous, PSM_pose_state


    def get_spatial_angular_jacobian(self):
        # note that jacobian matrices are derived from posterior RCM frame estimate
        # not backwards from new PSM measurement
        
        # multiply q3 by 10 because ambf in decimetres, dvrk in metres
        q1 = self.joints_prev[0]
        q2 = self.joints_prev[1]
        q3 = self.joints_prev[2]*10

        # homogeneous transforms, same as in RCM estimation function
        H_yaw = self.DH_to_transform([0, np.pi/2, 0, q1+np.pi/2])
        H_pitch = self.DH_to_transform([0, -np.pi/2, 0, q2-np.pi/2])
        H_d = self.DH_to_transform([0, np.pi/2, q3, 0])

        H_yaw_RCM = np.matmul(self.RCM_pose_posterior, H_yaw)
        H_pitch_RCM = np.matmul(H_yaw_RCM, H_pitch)
        H_d_RCM = np.matmul(H_pitch_RCM, H_d)

        # PSM origin
        o_n = H_d_RCM[0:2,3]
        o_n = o_n.ravel()

        # RCM origin
        o_RCM = self.RCM_pose_posterior[0:2,3]
        o_RCM = o_RCM.ravel()

        # due to kinematic setup, cross product always with o_diff in subsequent calcs (yaw and pitch origins at RCM )
        o_diff = o_n-o_RCM

        # yaw jacobian vectors
        z_RCM = self.RCM_pose_posterior[0:2,2]
        J_w_yaw = z_RCM
        z_RCM = z_RCM.ravel()
        J_v_yaw = np.cross(z_RCM,o_diff).reshape(3,1)

        # pitch jacobian vectors
        z_yaw = H_yaw_RCM[0:2,2]
        J_w_pitch = z_yaw
        z_yaw = z_yaw.ravel()
        J_v_pitch = np.cross(z_RCM,o_diff).reshape(3,1)

        z_pitch = H_pitch_RCM[0:2,2]
        J_v_d = z_pitch
        J_w_d = np.array([0, 0, 0]).reshape(3,1)

        J_v = np.hstack(J_v_yaw,np.hstack(J_v_pitch, J_v_d))
        J_w = np.hstack(J_w_yaw,np.hstack(J_w_pitch, J_w_d))
        return J_v, J_w


    def get_b_matrix(self):
        theta = self.PSM_pose_state_posterior[4]
        si = self.PSM_pose_state_posterior[5]
        B = np.array([[np.cos(si)*np.sin(theta), -np.sin(si), 0],
                      [np.sin(si)*np.sin(theta), np.cos(si), 0],
                      [np.cos(theta), 0, 1]])
        return B


    def kf_update(self, data, args):

        # get position, velocity, and time
        [joints_now, joint_vel_now, _, time] = self.arm.measured_js()

        # this might fail depending on the time_delta object format
        # hopefully just a float
        time_delta = time - self.prev_time

        # NDI pose should be calibrated in no extension, eliminates differences in roll joint length between tools

        # intialize kalman filter if first iteration
        if self.RCM_pose_posterior == None:
            self.kf_init(joints_now, time, data, joint_vel_now)

            # set up joints for next iteration
            self.joints_prev = joints_now
            return

        # get pose state measurement
        _, PSM_pose_state_measurement = self.calculate_measurement(data)

        # get Jacobians and B matrix
        J_v_prev, J_w_prev = self.get_spatial_angular_jacobian()
        B = self.get_b_matrix()
        B_inv = np.linalg.inv(B)
        
        # estimate spatial and angular state changes
        d_delta = np.matmul(J_v_prev, self.joint_vel_prev[0:2].reshape(3,1))
        a_delta = np.matmul(B_inv, np.matmul(J_w_prev, self.joint_vel_prev[0:2].reshape(3,1)))
        
        # add state delta to posterior to get new prior
        state_delta = time_delta * np.vstack(d_delta, a_delta)
        PSM_pose_prior = self.PSM_pose_state_posterior + state_delta

        # calculate prior covariance P
        P_prior = self.P_posterior + self.Q

        # calculate kalman gain
        K = np.matmul(P_prior, np.linalg.inv(self.R + P_prior))
        
        # calculate posterior state update and update homogenous posterior as well
        self.PSM_pose_state_posterior = PSM_pose_prior + np.matmul(K, (PSM_pose_state_measurement - PSM_pose_prior))
        PSM_posterior_orientation = R.from_euler('zyz', self.PSM_pose_state_posterior[3:].reshape(1,3))
        PSM_posterior_orientation = PSM_posterior_orientation.as_matrix()
        self.PSM_pose_homogeneous_posterior = np.vstack((np.hstack((PSM_posterior_orientation, self.PSM_pose_state_posterior[0:3])), np.array([0,0,0,1])))

        # calculate posterior covariance P
        self.P_posterior = np.matmul((np.identity(3) - K), P_prior)

        # calculate new RCM pose and update posterior by computing kinematics backwards
        self.RCM_pose_posterior = self.calculate_RCM(joints_now)

        if self.rcm_pub_flag is True:
            # publish RCM pose
            RCM_r = R.from_matrix(self.RCM_pose_posterior[0:3,0:3])
            RCM_r = RCM_r.as_quat()
            self.RCM_pose_state.position.x = self.RCM_pose_posterior[0,3]
            self.RCM_pose_state.position.y = self.RCM_pose_posterior[1,3]
            self.RCM_pose_state.position.z = self.RCM_pose_posterior[2,3]
            self.RCM_pose_state.orientation.x = RCM_r[0]
            self.RCM_pose_state.orientation.y = RCM_r[1]
            self.RCM_pose_state.orientation.z = RCM_r[2]
            self.RCM_pose_state.orientation.w = RCM_r[3]
            self.RCM_pub.publish(self.RCM_pose_state)


        # update timestamp, velocity, and joints for next iteration
        self.prev_time = time
        self.joint_vel_prev = joint_vel_now

        return


    def init_subs(self):
        rospy.init_node(self.toolname + '_KF', anonymous = True)

        rospy.on_shutdown(self.cleanup)

        for key in self.topic_dict:
            if self.topic_dict[key]["arm"] == "PSM":
                rospy.Subscriber(name = key, data_class=self.topic_dict[key]["type"], callback=self.kf_update, callback_args=key)
        
        # VF logic, only do this at 5 Hz to not slow down sim
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()