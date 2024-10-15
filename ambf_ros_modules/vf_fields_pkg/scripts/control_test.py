# I am going to attempt parsing the information I need directly from ros with a subscriber
# the ambf client only supports one connection at a time so it does not work if you open a second 
# client for the same object

# ros and ambf imports
import rospy
import rospkg
#import time
from ambf_msgs.msg import RigidBodyState, RigidBodyCmd
#from ambf_client import Client
from argparse import ArgumentParser
from geometry_msgs.msg import WrenchStamped, Wrench
from std_msgs.msg import Header


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

# constructed classes
from mesh import MeshObj
from robot import rob_state


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-arm', type=int)
    parser.add_argument('-bimanual', type=int)
    args, _ = parser.parse_known_args()

    if args.arm is None:
        args.arm = 1

    if args.bimanual is None or args.bimanual == args.arm: # prevent from querying itself for force feedback
        args.bimanual = 0

    tree = MeshObj(adf_num = 5)
    print(tree.tree.data[0])

    # initialize the listener subscriber with the known tree mesh info
    psm_listener = rob_state(tree.tree, psmnum = args.arm, bimanual = args.bimanual)

    # start the main subscriber loop for each arm
    psm_listener.listener()