# ros and ambf imports
import rospy
import rospkg
#import time
from ambf_msgs.msg import RigidBodyState, RigidBodyCmd
#from ambf_client import Client
from geometry_msgs.msg import WrenchStamped, Wrench
from std_msgs.msg import Header


#mesh and model libraries
import numpy as np
from scipy import spatial
from scipy.spatial.transform import Rotation as R


#plotting libraries
from matplotlib import pyplot as plt


rospack = rospkg.RosPack()
VF_path = rospack.get_path('vf_fields_pkg')

class rob_state:
    def __init__(self, tree, psmnum = 1, surface_sphere = True, force_vis = True, force_pub = False, bimanual = 0):
        self.psmnum = psmnum
        self.nodename = 'psm_listener'
        self.topic_dict = {'ambf/env/psm'+ str(psmnum) + '/toolrolllink/State': {'data': None, 'type': RigidBodyState}}

        self.bimanual_topic = None
        if bimanual != 0:
            self.bimanual_topic = {'ambf/env/psm'+ str(bimanual) + '/toolrolllink/State': {'data': None, 'type': RigidBodyState}}

        # stl tree passed onto psm at runtime
        self.tree_obj = tree

        self.roll_start_dist = 1.476
        # self.roll_end_dist = 2.168
        self.roll_end_dist = 1.8 # shortened to have haptics apply further back

        # running variables keeping track of position and distances of various bodies
        # note that each dist variable keeps track of distance and timestamp
        self.roll_position = None
        self.roll_y_axis = None

        # variable to keep track of current second arm position
        # initialized to 0 to prevent problems with accessing before assigned
        self.bim_q_points = np.array([0,0,0], ndmin=2)


        # dist is a 3xn list of the distance, the tree index, and the timestep
        self.roll_dist = np.empty([0,3])
        self.roll_points = None

        self.grip1_position = None
        self.grip1_dist = np.empty([0,3])

        self.grip2_position = None
        self.grip2_dist = np.empty([0,3])
        # all three dist lists will be "zippered" together at the end with interpolation 
        # so each distance can be properly compared and the minimum selected

        self.fmag_list = np.empty([0,2])

        # maximum distance until manipulator experiences force
        self.dmax = 0.25
        # saturation cutoff for force generation
        self.force_sat = 5

        # mesh indicator tracking closest point
        self.surface_sphere = surface_sphere
        if self.surface_sphere == True:
            self.sphere_pub = RigidBodyCmd()
            self.sphere_pub.cartesian_cmd_type = 1
            self.sphere_pub.pose.orientation.w = 1
        else:
            self.sphere_pub = None
        
        # flag for force visualization arrow
        self.force_vis = force_vis

        # flag for force publishing
        self.force_pub = force_pub

        self.plot_roll = False
        self.plot_force_mag = True

    def vec_to_force(self, v_p, dist):
        # parallel component of distance vector
        v_par = (np.dot(v_p, self.roll_y_axis)/np.dot(self.roll_y_axis,self.roll_y_axis)) * self.roll_y_axis

        # perpendicular component of force then normalized
        v_perp = v_p-v_par
        v_perp = v_perp/np.linalg.norm(v_perp)

        # scale force according to inverse square law, reverse how gravity works
        if dist < self.dmax:
            f_scale = np.sqrt((self.dmax - dist)/self.dmax)

            # scale force according to velocity vector alignment with offset
            v_scale = (np.dot(self.roll_vel, v_perp)*0.5)+1
        else:
            f_scale = 0
            v_scale = 0


        f = v_perp*v_scale*f_scale
        return f

    def calc_force(self, q_distances, q_points, bim_vec = None):
        # this initializes at 0 for both force and torque
        wrench_vec = Wrench()

        # add force components for each close point
        for i in range(len(q_distances[0])):
            # note both roll points and tree points are row vectors not column
            mesh_coord = self.tree_obj.data[q_distances[1][i]]

            v_p = np.transpose(q_points[i]-mesh_coord)

            f = self.vec_to_force(v_p, q_distances[0][i])

            # add effects of points together in wrench


            wrench_vec.force.x += f[0]
            wrench_vec.force.y += f[1]
            wrench_vec.force.z += f[2]

        # if bimanual forces are enabled, we know the last entry corresponds to the bimanual effects
        # we multiply this by 3 to scale it up to a similar degree as the cup interactions
        if self.bimanual_topic is not None:
            f = self.vec_to_force(bim_vec, np.linalg.norm(bim_vec))*3
            wrench_vec.force.x += f[0]
            wrench_vec.force.y += f[1]
            wrench_vec.force.z += f[2]

        linear_force = [wrench_vec.force.x, wrench_vec.force.y, wrench_vec.force.z]
        lin_norm = np.linalg.norm([wrench_vec.force.x, wrench_vec.force.y, wrench_vec.force.z])

        # check if force has reached saturation cutoff
        if lin_norm > self.force_sat:
            linear_force = self.force_sat*(linear_force/lin_norm)
            wrench_vec.force.x = linear_force[0]
            wrench_vec.force.y = linear_force[1]
            wrench_vec.force.z = linear_force[2]

            mag = self.force_sat
        else:
            mag = lin_norm

        return wrench_vec, mag
    
    def callback_bim(self, data, args):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        bim_roll_position = np.array([x,y,z])
        rollframe = R.from_quat([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        rollframe = rollframe.as_matrix()
        bim_roll_y_axis = rollframe[:,1]

        # find start and end point based on known length of roll body and orientation of y axis
        start_point = bim_roll_position + bim_roll_y_axis* self.roll_start_dist
        end_point = bim_roll_position - bim_roll_y_axis* self.roll_end_dist
        # print(start_point,end_point,self.roll_position, self.roll_y_axis)

        # generate list of query points and transpose to fit query requirements
        self.bim_q_points = np.linspace(start_point, end_point, num=10)


    def callback(self, data, args):

        if data.name.data == 'toolrolllink':
            x = data.pose.position.x
            y = data.pose.position.y
            z = data.pose.position.z

            # store linear velocity of roll joint
            self.roll_vel = np.array([data.twist.linear.x, data.twist.linear.y, data.twist.linear.z])
            self.roll_vel.reshape(3,1)

            self.roll_position = np.array([x,y,z])
            rollframe = R.from_quat([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
            rollframe = rollframe.as_matrix()
            self.roll_y_axis = rollframe[:,1]

            # find start and end point based on known length of roll body and orientation of y axis
            start_point = self.roll_position + self.roll_y_axis* self.roll_start_dist
            end_point = self.roll_position - self.roll_y_axis* self.roll_end_dist
            # print(start_point,end_point,self.roll_position, self.roll_y_axis)

            # generate list of query points and transpose to fit query requirements
            q_points = np.linspace(start_point, end_point, num=20)
            q_distances = self.tree_obj.query(q_points)
            # print(q_distances)

            # store closest points in appropriate array
            query_closest = np.argmin(q_distances[0])
            closest = np.array([q_distances[0][query_closest], q_distances[1][query_closest], data.sim_time])
            self.roll_dist = np.vstack((self.roll_dist,closest))
            # print(closest[0])

            if self.bimanual_topic is not None:
                # find the distances between each arm point and select the shortest distance
                bim_dist_mat = spatial.distance.cdist(q_points, self.bim_q_points, metric = 'euclidean')
                small_index = np.unravel_index(np.argmin(bim_dist_mat, axis = None), bim_dist_mat.shape)

                # find the closest pair of points in the two arrays
                bim_closest = self.bim_q_points[small_index[1], :]
                teleop_closest = q_points[small_index[0], :]

                # add this vector to force calculations
                bim_vec = teleop_closest-bim_closest
            else:
                bim_vec = None


            wrench, mag = self.calc_force(q_distances, q_points, bim_vec = bim_vec) # remember to update MTM publisher with wrench info!

            if self.force_pub == True:
                # add header to wrench for publishing protocol
                h = Header()
                h.stamp = rospy.Time.now()
                w_stamped = WrenchStamped()
                w_stamped.header = w_stamped.wrench = wrench
                self.force_cmd.publish(w_stamped)

            if self.force_vis == True:
                mag_stamp = np.array([mag, data.sim_time])
                self.fmag_list = np.vstack((self.fmag_list, mag_stamp))

            # update surface sphere pos based on KD_tree query
            if self.surface_sphere == True:
                mesh_coord = self.tree_obj.data[q_distances[1][query_closest]]
                self.sphere_pub.pose.position.x = mesh_coord[0]
                self.sphere_pub.pose.position.y = mesh_coord[1]
                self.sphere_pub.pose.position.z = mesh_coord[2]

                self.sphere_cmd.publish(self.sphere_pub)
        #elif data.name.data == 'toolpitchlink':
                


    def cleanup(self):
        # execute on finish

        if self.plot_roll:
            # shift distance values to zero seconds starting at program start
            oldest_time = self.roll_dist[0][2]
            self.roll_dist[:, -1] -= oldest_time


            print('program finished')
            print(self.roll_dist)

            roll_timestamp = self.roll_dist[:, -1]
            roll_d_list = self.roll_dist[:, 0]

            # print(roll_timestamp)

            fig, ax = plt.subplots()
            plt.plot(roll_timestamp, roll_d_list, 'r')
            plt.show()

        elif self.plot_force_mag:
            # shift distance values to zero seconds starting at program start
            oldest_time = self.fmag_list[0][1]
            self.fmag_list[:, -1] -= oldest_time


            print('program finished')
            print(self.fmag_list)

            f_timestamp = self.fmag_list[:, -1]
            mag_list = self.fmag_list[:, 0]

            fig, ax = plt.subplots()
            plt.plot(f_timestamp, mag_list, 'r')
            plt.show()


    def listener(self):
        rospy.init_node(self.nodename, anonymous = True)

        # generate subscriber topic for controlled arm
        for key in self.topic_dict:
            rospy.Subscriber(name = key, data_class=self.topic_dict[key]["type"], callback=self.callback, callback_args=key)
        rospy.on_shutdown(self.cleanup)

        # update this to work with multiple spheres
        if self.surface_sphere == True:
            self.sphere_cmd = rospy.Publisher(name='/ambf/env/Icosphere' + str(self.psmnum) +'/Command', data_class=RigidBodyCmd, tcp_nodelay=True, queue_size=10)

        if self.force_pub == True:
            if self.psmnum == 1:
                mtm_label = '/MTML/'
            else:
                mtm_label = '/MTMR/'
            self.force_cmd = rospy.Publisher(name=mtm_label + 'body/servo_cf', data_class=WrenchStamped, tcp_nodelay=True, queue_size=10)


        # generate subscriber for possible other arms
        if self.bimanual_topic is not None:
            for key in self.bimanual_topic:
                rospy.Subscriber(name = key, data_class=self.bimanual_topic[key]["type"], callback=self.callback_bim, callback_args=key)

        #if self.force_vis = True:

        # VF logic, only do this at 5 Hz to not slow down sim
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()