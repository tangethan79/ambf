
# I am going to attempt parsing the information I need directly from ros with a subscriber
# the ambf client only supports one connection at a time so it does not work if you open a second 
# client for the same object

# ros and ambf imports
import rospy
import os
import sys
import rospkg
#import time
#from ambf_client import Client
from argparse import ArgumentParser

rospack = rospkg.RosPack()
VF_path = rospack.get_path('vf_fields_pkg')
scripts_path = os.path.join(VF_path, 'scripts')
sys.path.append(scripts_path)

# constructed classes
from mesh import MeshObj
from robot_ambf import rob_state
from robot_ndi import rob_state_ndi


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--arm', type=int)
    parser.add_argument('--adf', type=int)
    parser.add_argument('--bimanual', type=int)
    parser.add_argument('--ndi', action='store_true')
    parser.set_defaults(ndi = False)
    parser.add_argument('--sdf', action='store_true')
    parser.set_defaults(sdf = False)
    parser.add_argument('--haptic', action='store_true')
    parser.set_defaults(haptic = False)
    parser.add_argument('--launch', action='store_true') # nodes started from launch file, use throttled topics for ecm
    parser.set_defaults(haptic = False)
    args, _ = parser.parse_known_args()

    if args.arm is None:
        args.arm = 1

    if args.bimanual is None or args.bimanual == args.arm: # prevent from querying itself for force feedback
        args.bimanual = 0

    if args.adf is None:
        args.adf = 5

    # Jacky: modified this if statement
    # if args.sdf is False or args.ndi is False:
    if args.sdf is False:
        tree = MeshObj(adf_num = args.adf)
        print(tree.tree.data[0])
        
        if args.ndi is True:
            psm_listener = rob_state_ndi(tree.tree, psmnum = args.arm, bimanual = args.bimanual, force_pub=args.haptic, launch = args.launch)
        else:
            # initialize the listener subscriber with the known tree mesh info
            psm_listener = rob_state(tree.tree, psmnum = args.arm, bimanual = args.bimanual, force_pub=args.haptic, sdf=False)
    else:
        sdf = MeshObj(adf_num = args.adf, sdf = True)
        if args.ndi is True:
            psm_listener = rob_state_ndi(sdf, psmnum = args.arm, bimanual = args.bimanual, sdf = True, force_pub=args.haptic, launch = args.launch)
        else:
            psm_listener = rob_state(tree=sdf, psmnum = args.arm, bimanual = args.bimanual, sdf = True, force_pub=args.haptic)
            


    # start the main subscriber loop for each arm
    psm_listener.listener()