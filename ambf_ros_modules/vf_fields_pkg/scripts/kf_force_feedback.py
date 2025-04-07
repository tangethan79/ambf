# this is the version of the robot control script that interfaces with the dvrk and ndi
# to replace the ambf ros topics, we calibrate using the dvrk and ndi ROS topics
# to simplify interaction, dvrk will be interfaced with through crtk with commands found here: https://crtk-robotics.readthedocs.io/en/latest/pages/clients.html#client-libraries-python

# the NDI ROS bridge program supports providing reference frames with respect to other frames
# with this tool, we can get the PSM frame wrt the cleft and leave the cleft completely stationary in the scene at an origin determined in blender
# This also preserves the relationship if the NDI camera gets moved, both origins are stationary if they don't move relative to each other
import dvrk
# these two packages control reading and writing dvrk joint vals

from trimesh import proximity

# ros and ambf imports
import rosbag
import rospy
import rospkg
#import time
from ambf_msgs.msg import RigidBodyState, RigidBodyCmd
#from ambf_client import Client
from geometry_msgs.msg import WrenchStamped, Wrench, Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header


#mesh and model libraries
import numpy as np
from scipy import spatial
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R
from collections import deque


#plotting and data libraries
from matplotlib import pyplot as plt
import json
import os
import datetime

rospack = rospkg.RosPack()
VF_path = rospack.get_path('vf_fields_pkg')
logs_path = os.path.join(VF_path, 'logs')


def butter_lowpass(cutoff, fs, order=2):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


# Apply the filter to 3D force vectors
def apply_filter(b, a, force_history):
    return filtfilt(b, a, force_history, axis=0, method="gust")


def convert_np_to_list(data):
    if isinstance(data, dict):
        return {k: convert_np_to_list(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [convert_np_to_list(i) for i in data]
    elif isinstance(data, np.ndarray):
        return data.tolist()
    else:
        return data


class rob_state_kf:
    def __init__(self, tree, filter, psmnum = 1, cylinder_update = True, surface_sphere = True, force_pub = True, sdf = True, record_result = False):
        self.psmnum = psmnum
        
        self.kf_estimator = filter

        ndi_ecm_str = '/NDI/ECM/measured_cp'
        dvrk_ecm_str = '/ECM/measured_cp'

        self.record_result = record_result
        if self.record_result:
            date_str = datetime.date.today().strftime('%y-%m-%d')
            self.today_dir_str = os.path.join(logs_path, date_str)

            if not os.path.exists(self.today_dir_str):
                os.mkdir(self.today_dir_str)

            time_str = datetime.datetime.now()
            self.time_str = time_str.strftime("%H:%M:%S")

            file_path = os.path.join(self.today_dir_str, self.time_str) + 'PSM' + str(self.psmnum)

            self.json_file_path = file_path + '.json' # create new json file in today's log directory with timestamp as filename

            self.bag = rosbag.Bag(file_path + '.bag', 'w')
            self.roll_frame_bag = PoseStamped()


        self.cylinder_update = cylinder_update

        self.last_valid_norm = None
        self.cutoff_freq = 0.05
        self.sampling_rate = 50 # appx. publishing rate of force
        self.b, self.a = butter_lowpass(self.cutoff_freq, self.sampling_rate)

        self.force_history = deque(maxlen=20)

        self.cleft_p = np.array([-0.93998,0.003,1.05936]).reshape(3,1)
        cleft_R = R.from_euler('xyz', [1.85791,-0.2802,2.27472])
        self.cleft_R = cleft_R.as_matrix()

        pad = np.array([0,0,0,1])
        self.cleft_pose = np.hstack((self.cleft_R,self.cleft_p))
        self.cleft_pose = np.vstack((self.cleft_pose, pad))

        #initialize ecm pose
        self.ECM_pose = np.array([[1,0,0,0],
                                 [0,1,0,0],
                                 [0,0,1,0],
                                 [0,0,0,1]])

        self.psmnum = psmnum

        # self.toolname should be the string used in the NDI ROS topic, update this later to correct value

        self.toolname = "PSM" + str(self.psmnum)+'_kf'
        self.nodename = 'psm_ndi_listener'

        # this has been updated to take the pose of the NDI tracker which we assume is at the tip of the robot with at least one axis lining up with the roll axis
        # we also assume that this pose is wrt the cleft pose
        # in callback, we need to deal with scaling and add static pose offset imposed by blender cleft model (x = -9.3998 cm, y = 0.0301 cm, z = 10.594 cm)
        
        if psmnum == 1:
            mtm_hand = "R"
        else:
            mtm_hand = "L"
        self.topic_dict = {'/NDI/' + self.toolname + '/measured_cp': {'data': None, 'type': PoseStamped, 'arm':'PSM'},
                           ndi_ecm_str: {'data': None, 'type': PoseStamped, 'arm':'ECM'},
                           '/MTM'+ mtm_hand+'/measured_cp': {'data': None, 'type': PoseStamped, 'arm':'MTM_P'},
                           dvrk_ecm_str: {'data': None, 'type': PoseStamped, 'arm':'ECM_P_DVRK'}}

        # need y axis to align with roll joint

        self.bimanual_topic = None

        self.roll_frame = Pose()
        self.sdf_flag = sdf

        if sdf is False:
            # stl tree passed onto psm at runtime
            self.tree_obj = tree
        else:
            self.sdf = tree

        self.roll_end_dist = 3.2 # shortened to have haptics apply further back

        # running variables keeping track of position and distances of various bodies
        # note that each dist variable keeps track of distance and timestamp
        self.roll_position = None
        self.roll_z_axis = None

        self.fmag_list = np.empty([0,3])

        # maximum distance until manipulator experiences force
        self.dmax = 0.2
        self.wall_thresh = 0.02
        # saturation cutoff for force generation
        self.force_sat = 2.5

        # mesh indicator tracking closest point
        self.surface_sphere = surface_sphere
        if self.surface_sphere == True:
            self.sphere_pub = RigidBodyCmd()
            self.sphere_pub.cartesian_cmd_type = 1
            self.sphere_pub.pose.orientation.w = 1
        else:
            self.sphere_pub = None
        
        self.cylinder_pub = RigidBodyCmd()
        self.cylinder_pub.cartesian_cmd_type = 1

        # flag for force publishing
        self.force_pub = force_pub


    def calc_force_sdf(self, grad, dist):
        # this initializes at 0 for both force and torque
        wrench_vec = Wrench()

        f = self.vec_to_force(grad, dist)

        # add effects of points together in wrench
        wrench_vec.force.x += f[0]
        wrench_vec.force.y += f[1]
        wrench_vec.force.z += f[2]

        lin_norm = np.linalg.norm([wrench_vec.force.x, wrench_vec.force.y, wrench_vec.force.z])

        return wrench_vec, lin_norm


    def sqrt_force(self, dist):
        f_scale = np.sqrt(abs(self.dmax - dist)/self.dmax)* self.force_sat
        return f_scale
    

    def exp_force(self, dist):
        if dist < 0:
            dist = 0
        steepness = 50
        dist_inv = self.dmax-dist

        f_scale = (self.force_sat/(np.exp(steepness*self.dmax)-1)) * (np.exp(steepness*dist_inv)-1)
        #f_scale = np.exp(-20*dist)*self.force_sat
        return f_scale


    def vec_to_force(self, v_p, dist):
        # parallel component of distance vector
        v_par = (np.dot(v_p, self.ECM_z)/np.dot(self.ECM_z,self.ECM_z)) * self.ECM_z

        # perpendicular component of force then normalized
        v_perp = v_p-v_par

        # this is a direction not a force scale!
        f = v_perp/np.linalg.norm(v_perp)


        # scale force according to inverse square law, reverse how gravity works
        if dist < self.dmax:
            if dist < self.wall_thresh:

                # if we are in a wall, use the last valid norm
                f_scale = self.force_sat
                f = self.last_valid_norm
            else:
                f_scale = self.exp_force(dist)
                self.last_valid_norm = f

        # do nothing if above dist threshold
        else:
            f_scale = 0

        f = np.matmul(np.linalg.inv(self.ECM_pose[:3,:3]),f.reshape(3,1))
        f = f*f_scale

        # force direction was wrong, this is 180 degree rotation about x axis
        flip_mat = np.array([[-1,0,0],
                            [0,-1,0],
                            [0,0,1]])

        f = np.matmul(flip_mat,f)

        self.force_history.append(f)

        # Only apply the filter when we have enough data points
        if len(self.force_history) >= 2:
            # Apply the Butterworth filter to smooth the forces
            # try out some new filtering techniques, this doesn't work too good
            filtered_force = apply_filter(self.b, self.a, list(self.force_history))
            f = filtered_force[-1]
            #print(f)

    
        #f = np.array([0,2,0]).reshape(3,1)

        # replace below check with psm force threshold
        f = np.matmul(np.linalg.inv(self.MTM_R),f)
        #print(dist, np.linalg.norm(f))
        return f

    def calc_force(self, roll_point, mouth_point):
        # this initializes at 0 for both force and torque
        wrench_vec = Wrench()


        # add force components for closest point
        v_p = np.transpose(roll_point - mouth_point)
        #print(v_p)

        f = self.vec_to_force(v_p, np.linalg.norm(v_p))

        # add effects of point in wrench


        wrench_vec.force.x += f[0]
        wrench_vec.force.y += f[1]
        wrench_vec.force.z += f[2]

        lin_norm = np.linalg.norm([wrench_vec.force.x, wrench_vec.force.y, wrench_vec.force.z])


        return wrench_vec, lin_norm
    

    def psm_update(self):

        h = Header()
        h.stamp = rospy.Time.now()

        # position info from ros, assume default is in metres
        # ambf units are in decimetres so we multiply the metre value by 10

        # the extra fixed pose is the pose of the reference geometry in ambf, i.e. the cleft model

        # get RCM and joint information from state estimator
        RCM_H = self.kf_estimator.RCM_pose_posterior
        joints = self.kf_estimator.arm.measured_jp()

        q1 = joints[0]
        q2 = joints[1]
        # distance subtraction is difference between RCM and jaw at 0 position, roughly 4 cm
        q3 = joints[2]*10 - 0.4

        H_yaw = self.kf_estimator.DH_to_transform([0, np.pi/2, 0, q1+np.pi/2])
        H_pitch = self.kf_estimator.DH_to_transform([0, -np.pi/2, 0, q2-np.pi/2])
        H_d = self.kf_estimator.DH_to_transform([0, np.pi/2, q3, 0])

        H_yaw_RCM = np.matmul(RCM_H, H_yaw)
        H_pitch_RCM = np.matmul(H_yaw_RCM, H_pitch)
        PSM_pose = np.matmul(H_pitch_RCM, H_d)

        Rotation = PSM_pose[0:3,0:3]
        r = R.from_matrix(Rotation)
        r = r.as_quat()
        # quaternion info from ros
        xw = r[0]
        yw = r[1]
        zw = r[2]
        ww = r[3]

        self.roll_frame.position.x = PSM_pose[0,3]
        self.roll_frame.position.y = PSM_pose[1,3]
        self.roll_frame.position.z = PSM_pose[2,3]

        self.roll_frame.orientation.x = xw
        self.roll_frame.orientation.y = yw
        self.roll_frame.orientation.z = zw
        self.roll_frame.orientation.w = ww

        self.roll_position = PSM_pose[0:3,3].reshape(1,3)
        self.roll_z_axis = PSM_pose[0:3,2].reshape(1,3)

        # find start and end point based on known length of roll body and orientation of y axis
        start_point = self.roll_position
        end_point = self.roll_position - self.roll_z_axis* self.roll_end_dist

        # generate list of query points and transpose to fit query requirements
        num_points = 20
        q_points = np.linspace(start_point, end_point, num=num_points)


        if self.sdf_flag is False:
            q_distances = self.tree_obj.query(q_points)
            # print(q_distances)

            # store closest points in appropriate array
            query_closest = np.argmin(q_distances[0])
            dist = q_distances[0][query_closest]
        else:
            grad_vec, dist, closest = self.sdf.query_SDF_grad(q_points)


        if self.cylinder_update:
            self.cylinder_pub.pose = self.roll_frame
            self.cylinder_cmd.publish(self.cylinder_pub)

        if self.sdf_flag is False:
            wrench, mag = self.calc_force(q_points[query_closest], self.tree_obj.data[q_distances[1][query_closest]]) # remember to update MTM publisher with wrench info!
        else:
            wrench, mag = self.calc_force_sdf(grad_vec, dist)

        if self.force_pub == True:
            # add header to wrench for publishing protocol
            w_stamped = WrenchStamped()
            w_stamped.header = h
            w_stamped.wrench = wrench
            self.force_cmd.publish(w_stamped)

        if self.record_result == True:
            mag_stamp = np.array([mag, dist, h.stamp.to_sec()])
            self.fmag_list = np.vstack((self.fmag_list, mag_stamp))

            self.roll_frame_bag.pose = self.roll_frame
            self.roll_frame_bag.header = h
            self.bag.write('/ambf/env/Cylinder' + str(self.psmnum) + '/Command', self.roll_frame_bag)

        # update surface sphere pos based on KD_tree query
        if self.surface_sphere == True:
            if self.sdf_flag is False:
                mesh_coord = self.tree_obj.data[q_distances[1][query_closest]]
            else:
                meshes, __ , __ = proximity.closest_point(self.sdf.mesh, closest.reshape(1,-1))
                mesh_coord = meshes[0]
            
            self.sphere_pub.pose.position.x = mesh_coord[0]
            self.sphere_pub.pose.position.y = mesh_coord[1]
            self.sphere_pub.pose.position.z = mesh_coord[2]

            self.sphere_cmd.publish(self.sphere_pub)
                

    def cleanup(self):
        # execute on finish
        self.bag.close()
        if self.record_result:

            # shift distance values to zero seconds starting at program start
            oldest_time = self.fmag_list[0][-1]
            self.fmag_list[:, -1] -= oldest_time


            print('program finished')
            print(self.fmag_list)

            f_timestamp = self.fmag_list[:, -1]
            mag_list = self.fmag_list[:, 0]

            results_dict = {'timestamp':self.fmag_list[:,2],'forces':self.fmag_list[:,0],'distance':self.fmag_list[:,1]}
            results_dict = convert_np_to_list(results_dict) # convert results dictionary into list format for json writing

            with open(self.json_file_path, "w") as json_file:
                json.dump(results_dict, json_file, indent=4)

            fig, ax = plt.subplots()
            plt.plot(f_timestamp, mag_list, 'r')
            plt.show()


    def ECM_callback(self, data, args):
        ECM_p = np.array([data.pose.position.x*10,data.pose.position.y*10,data.pose.position.z*10]).reshape(3,1)
        ECM_R = R.from_quat([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        ECM_pose = np.hstack((ECM_R.as_matrix(),ECM_p))
        ECM_pose = np.vstack((ECM_pose, np.array([0,0,0,1])))
        self.ECM_pose = np.matmul(self.cleft_pose,ECM_pose)
        self.ECM_z = self.ECM_pose[:3, 2]
        
        # consider adding a a transform here which rotates ECM pose by 45 deg about y axis for tilted ECM config


    def MTM_P_callback(self, data, args):
        MTM_R = R.from_quat([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        self.MTM_R = MTM_R.as_matrix()

    def ECM_P_DVRK_callback(self, data, args):
        # 45 degree rotation only needed if endoscope not aligned properly
        # typically we use HD_DOWN for this experiment
        r = R.from_quat([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        """tilt_rotation = np.array([[1, 0, 0],
                                  [0, 0.70710678, -0.70710678],
                                  [0, 0.7071068, 0.7071068]])
        self.ECM_R_dVRK = np.matmul(r.as_matrix(),tilt_rotation)"""
        self.ECM_R_dVRK = r.as_matrix()


    def listener(self):
        rospy.init_node(self.nodename, anonymous = True)

        # generate subscriber topic for controlled arm
        for key in self.topic_dict:
            if self.topic_dict[key]["arm"] == "PSM":
                rospy.Subscriber(name = key, data_class=self.topic_dict[key]["type"], callback=self.kf_estimator.kf_update, callback_args=key)
            elif self.topic_dict[key]["arm"] == "ECM":
                rospy.Subscriber(name = key, data_class=self.topic_dict[key]["type"], callback=self.ECM_callback, callback_args=key)
            elif self.topic_dict[key]["arm"] == "MTM_P":
                rospy.Subscriber(name = key, data_class=self.topic_dict[key]["type"], callback=self.MTM_P_callback, callback_args=key)
            elif self.topic_dict[key]["arm"] == "ECM_P_DVRK":
                rospy.Subscriber(name = key, data_class=self.topic_dict[key]["type"], callback=self.ECM_P_DVRK_callback, callback_args=key)
        
        rospy.on_shutdown(self.cleanup)

        if self.surface_sphere == True:
            self.sphere_cmd = rospy.Publisher(name='/ambf/env/Icosphere' + str(self.psmnum) +'/Command', data_class=RigidBodyCmd, tcp_nodelay=True, queue_size=10)

        if self.force_pub == True:
            if self.psmnum == 2:
                mtm_label = '/MTML_PSM2/following/mtm/'
            else:
                mtm_label = '/MTMR_PSM1/following/mtm/'
            self.force_cmd = rospy.Publisher(name=mtm_label + 'body/servo_cf', data_class=WrenchStamped, tcp_nodelay=True, queue_size=10)


        self.cylinder_cmd = rospy.Publisher(name='/ambf/env/Cylinder' + str(self.psmnum) +'/Command', data_class=RigidBodyCmd, tcp_nodelay=True, queue_size=10)
        print(self.cylinder_cmd.name)

        # VF logic, only do this at 5 Hz to not slow down sim
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.PSM_update()
            rate.sleep()