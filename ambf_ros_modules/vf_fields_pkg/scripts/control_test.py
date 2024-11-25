
# I am going to attempt parsing the information I need directly from ros with a subscriber
# the ambf client only supports one connection at a time so it does not work if you open a second 
# client for the same object

# ros and ambf imports
import rospy
import rospkg
#import time
#from ambf_client import Client
from argparse import ArgumentParser

rospack = rospkg.RosPack()
VF_path = rospack.get_path('vf_fields_pkg')

# constructed classes
from mesh import MeshObj
from ambf_ros_modules.vf_fields_pkg.scripts.robot_ambf import rob_state
from ambf_ros_modules.vf_fields_pkg.scripts.robot_ndi import rob_state_ndi


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--arm', type=int)
    parser.add_argument('--bimanual', type=int)
    parser.add_argument('--ndi', action='store_true')
    parser.set_defaults(ndi = False)
    args, _ = parser.parse_known_args()

    if args.arm is None:
        args.arm = 1

    if args.bimanual is None or args.bimanual == args.arm: # prevent from querying itself for force feedback
        args.bimanual = 0

    tree = MeshObj(adf_num = 5)
    print(tree.tree.data[0])

    if args.ndi is True:
        psm_listener = rob_state_ndi(tree.tree, psmnum = args.arm, bimanual = args.bimanual)
    else:
        # initialize the listener subscriber with the known tree mesh info
        psm_listener = rob_state(tree.tree, psmnum = args.arm, bimanual = args.bimanual)

    # start the main subscriber loop for each arm
    psm_listener.listener()