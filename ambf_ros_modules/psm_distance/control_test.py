# I am going to attempt parsing the information I need directly from ros with a subscriber
# the ambf client only supports one connection at a time so it does not work if you open a second 
# client for the same object

# ros and ambf imports
import rospy
import time
from ambf_msgs.msg import RigidBodyState, RigidBodyCmd, ObjectCmd
from ambf_client import Client
from argparse import ArgumentParser
from geometry_msgs.msg import Pose, Wrench

#mesh and model libraries
import numpy as np
from stl import mesh
from scipy import spatial
from scipy.spatial.transform import Rotation as R
import yaml

#plotting libraries
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt


class MeshObj:
    def __init__(self, adf_num = None, body_index = None, stl_num = None):
        yaml_list = ["mouth_cup.yaml",'scan_aperture.yaml','open_oral_cavity.yaml']
        stl_list = ['mouth cup.STL','cleft_retracted_june_7.STL','Complete_remeshed.STL']
        if adf_num:
            self.adf_str = yaml_list[adf_num]
        else:
            # default stl string, change as needed
            self.adf_str = yaml_list[0]

        if body_index:
            self.body_ind = body_index
        else:
            self.body_ind = 0

        if stl_num:
            self.stl_str = stl_list[stl_num]
        else:
            self.stl_str = stl_list[adf_num]

        self.get_stl()
        self.load_tree()


    def load_tree(self):
        self.tree = spatial.KDTree(self.points)
        return self.tree


    def plot_mesh(self):
        # plotting test
        figure = plt.figure()
        axes = figure.add_subplot(projection='3d')
        axes.add_collection3d(mplot3d.art3d.Poly3DCollection(self.mesh.vectors))
        # automatically scale to the mesh size
        scale = self.mesh.points.flatten()
        axes.auto_scale_xyz(scale, scale, scale)
        plt.show()
        return


    def get_stl(self):
        # load stl from file and get transfrom from adf
        mouth = mesh.Mesh.from_file(self.stl_str)
        self.get_pose_homog()
        mouth.transform(self.homog)
        self.mesh = mouth
        self.points = np.around(np.unique(mouth.vectors.reshape([int(mouth.vectors.size/3), 3]), axis=0),2)
        return self.points


    def get_pose_homog(self):

        # extract the correct transform information for the desired body
        with open(self.adf_str, 'r') as stream:
            mouth_attrib = yaml.safe_load(stream)
        bodies = mouth_attrib['bodies']

        body_string = bodies[self.body_ind]
        transform = mouth_attrib[body_string]['location']

        # take rpy values from yaml file and put them in a list for scipy spatial method
        # assume that this is in the xyz Euler angle form
        xyz_eul = [transform['orientation']['r'],transform['orientation']['p'],transform['orientation']['y']]
        r_mat = R.from_euler('xyz', xyz_eul)
        r_mat = r_mat.as_matrix()

        # extract position information and concatenate to get homogenous transform
        pos = np.array( [[transform['position']['x']],
                         [transform['position']['y']],
                         [transform['position']['z']]] )
        h_mat = np.hstack((r_mat, pos))
        buffer = np.array([0,0,0,1])
        h_mat = np.vstack((h_mat, buffer))

        self.homog = h_mat
        # print(h_mat)
        return self.homog
    

    def query_tree(self, query_points):
        return self.tree.query(query_points)



class rob_state:
    def __init__(self, tree, psmnum = 1, surface_sphere = True, force_vis = True, force_pub = False):
        self.psmnum = psmnum
        self.nodename = 'psm'+str(self.psmnum)+'_listener'
        self.topic_dict = {'ambf/env/psm'+ str(self.psmnum) + '/toolrolllink/State': {'data': None, 'type': RigidBodyState}}
                           #'ambf/env/psm'+ str(self.psmnum) + '/toolpitchlink/State': {'data': None, 'type': RigidBodyState}}

        # stl tree passed onto psm at runtime
        self.tree_obj = tree

        self.roll_start_dist = 1.476
        self.roll_end_dist = 2.168

        # running variables keeping track of position and distances of various bodies
        # note that each dist variable keeps track of distance and timestamp
        self.roll_position = None
        self.roll_y_axis = None

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

    def calc_force(self, q_distances, q_points):
        # this initializes at 0 for both force and torque
        wrench_vec = Wrench()

        # add force components for each close point
        for i in range(len(q_distances[0])):
            # note both roll points and tree points are row vectors not column
            mesh_coord = self.tree_obj.data[q_distances[1][i]]

            v_p = np.transpose(q_points[i]-mesh_coord)

            # parallel component of distance vector
            v_par = (np.dot(v_p, self.roll_y_axis)/np.dot(self.roll_y_axis,self.roll_y_axis)) * self.roll_y_axis

            # perpendicular component of force then normalized
            v_perp = v_p-v_par
            v_perp = v_perp/np.linalg.norm(v_perp)

            # scale force according to inverse square law, reverse how gravity works
            if q_distances[0][i] < self.dmax:
                f_scale = np.sqrt((self.dmax- q_distances[0][i])/self.dmax)*0.5

                # scale force according to velocity vector alignment with offset
                v_scale = (np.dot(self.roll_vel, v_perp)*0.5)+1
            else:
                f_scale = 0
                v_scale = 0


            f = v_perp*v_scale*f_scale

            # add effects of points together in wrench
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
            q_points = np.linspace(start_point, end_point, num=5)
            q_distances = self.tree_obj.query(q_points)
            # print(q_distances)

            # store closest points in appropriate array
            query_closest = np.argmin(q_distances[0])
            closest = np.array([q_distances[0][query_closest], q_distances[1][query_closest], data.sim_time])
            self.roll_dist = np.vstack((self.roll_dist,closest))
            # print(closest[0])

            if self.force_vis == True:
                wrench, mag = self.calc_force(q_distances, q_points)

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
        node_name = 'psm' + str(self.psmnum) + 'listener'
        rospy.init_node(node_name, anonymous = True)
        for key in self.topic_dict:
            rospy.Subscriber(name = key, data_class=self.topic_dict[key]["type"], callback=self.callback, callback_args=key)
        rospy.on_shutdown(self.cleanup)

        if self.surface_sphere == True:
            self.sphere_cmd = rospy.Publisher(name='/ambf/env/Icosphere/Command', data_class=RigidBodyCmd, tcp_nodelay=True, queue_size=10)

        #if self.force_vis = True:

        # VF logic, only do this at 5 Hz to not slow down sim
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    tree = MeshObj(adf_num = 0)
    print(tree.tree.data[0])

    # initialize the listener subscriber with the known tree mesh info
    listen = rob_state(tree.tree)

    # start the main subscriber loop
    listen.listener()