#mesh and model libraries
import copy
import os
import yaml
yaml.Dumper.ignore_aliases = lambda *args : True


# the purpose of this script is to add all the .obj files to the yaml description file as colliders
# it takes the first object (i.e. the unsplit object) and copies its transform to the remaining .obj file components

def yaml_edit(config_string = "test_cube.yaml", body_index = 0):

    cwd = os.getcwd()
    obj_str_add = []
    for file in os.listdir(cwd):
        if file.endswith(".obj"):
            try:
                obj_str_add.append(file[0:-4])
            except:
                print(file)
                pass

    print(obj_str_add)

    # extract the correct transform information for the desired body
    if type(config_string) == None:
        config_string = input('stl file string (must be in same directory): ')
    with open(config_string, 'r') as stream:
        mouth_attrib = yaml.safe_load(stream)
        print(mouth_attrib)

    """
    bodies = mouth_attrib['bodies']
    if type(body_index) == None:
        print(bodies)
        body_index = input('type index of desired body: ')
    body_string = bodies[body_index]
    transform = mouth_attrib[body_string]['location']

    print(transform)

    print(mouth_attrib[body_string]['visible'])

    first_obj = obj_str_add[0] + '.obj'
    mouth_attrib[body_string]['collision mesh'] = first_obj

    """

    bodies = mouth_attrib['bodies']
    body_string = bodies[body_index]
    mouth_attrib['high resolution path'] = "../../../meshes/blender_afmb/cleft_models/convex_cup_v2/"
    mouth_attrib['low resolution path'] = "../../../meshes/blender_afmb/cleft_models/convex_cup_v2/"
    for body_name in obj_str_add:

        print(body_name)
        # add body name to list of bodies
        mouth_attrib['bodies'].append('BODY ' + body_name)

        # make a copy of the first body dictionary
        new_body_dict = copy.deepcopy(mouth_attrib[body_string])

        # set all transforms to zero due to weird origin in stuff in blender
        new_body_dict['location']['orientation']['r'] = 1.5708
        new_body_dict['location']['orientation']['p'] = 0
        new_body_dict['location']['orientation']['y'] = 0

        new_body_dict['location']['position']['x'] = 0
        new_body_dict['location']['position']['y'] = 0
        new_body_dict['location']['position']['z'] = 0

        # change visibility and mesh parameters
        new_body_dict['name'] = body_name
        new_body_dict['mesh'] = body_name + '.obj'
        new_body_dict['collision mesh'] = body_name + '.obj'
        new_body_dict['visible'] = False

        # add new body to the original dictionary
        mouth_attrib['BODY ' + body_name] = new_body_dict

    print(mouth_attrib)
    out_str = config_string[0:-5]+'_split'+'.yaml'

    # remove original cup
    mouth_attrib.pop(body_string)
    mouth_attrib['bodies'].pop(0)

    with open(out_str, 'w') as yaml_file:
        yaml.dump(mouth_attrib, yaml_file, default_flow_style=False)

if __name__ == "__main__":
    yaml_edit("mouth_cup_v2.yaml")