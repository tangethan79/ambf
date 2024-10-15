
import rospkg

#mesh and model libraries
import numpy as np
from stl import mesh
from scipy import spatial
from scipy.spatial.transform import Rotation as R
import yaml

#plotting libraries
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt


rospack = rospkg.RosPack()
VF_path = rospack.get_path('vf_fields_pkg')

class MeshObj:
    def __init__(self, adf_num = None, body_index = None, stl_num = None):
        yaml_list = ['mouth_cup.yaml','scan_aperture.yaml','open_oral_cavity.yaml','mouth_cup.yaml', 'mouth_cup_v2.yaml', 'suture_box.yaml']
        stl_list = ['mouth cup.STL','cleft_retracted_june_7.STL','Complete_remeshed.STL','mouth cup smooth.STL', 'mouth cup v2 no holes.STL']
        if adf_num:
            self.adf_str = yaml_list[adf_num]
        else:
            # default stl string, change as needed
            self.adf_str = yaml_list[0]
        self.adf_str = VF_path + '/config/' + self.adf_str

        if body_index:
            self.body_ind = body_index
        else:
            self.body_ind = 0

        if stl_num:
            self.stl_str = stl_list[stl_num]
        else:
            # extract the correct STL information for the desired body
            with open(self.adf_str, 'r') as stream:
                mouth_attrib = yaml.safe_load(stream)
            bodies = mouth_attrib['bodies']

            body_string = bodies[self.body_ind]
            self.stl_str = mouth_attrib[body_string]['mesh']
        self.stl_str = VF_path + '/meshes/' + self.stl_str

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