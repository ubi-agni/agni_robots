#!/usr/bin/env python

import argparse
import yaml
import rospkg

import os.path
import os


def create_symlink(path, source , target, suffix):
    if os.path.isfile(path + source + suffix):
        if os.path.isfile(path + target + suffix):
            print "file ", target+suffix , " already exists"
        else:
            cmd = "cd " + path + "; "
            cmd += "ln -s " + source + suffix + " " + target + suffix
            #print cmd
            os.system(cmd)
    else:
        print "source file ", source + suffix , " does not exist" 


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser(usage='Generate symlinks for given profile based on given existing profile')
    PARSER.add_argument('sourceprofile', type=str, nargs='?',
                        default=None,
                        help='source profile to load.')
    PARSER.add_argument('targetprofile', type=str, nargs='?',
                        default=None,
                        help='target profile to load.')                     
    ARGS = PARSER.parse_args()

    if ARGS.sourceprofile is not None and ARGS.targetprofile is not None:

        source_profile = ARGS.sourceprofile
        target_profile = ARGS.targetprofile
        agni_deploy = rospkg.RosPack().get_path('agni_deploy') 
        agni_description = rospkg.RosPack().get_path('agni_description') 
        robots_yaml_file = agni_description + "/robots/robots.yaml"
        agni_description_config_path = agni_description + "/config/"
        agni_deploy_launch_path = agni_deploy + "/launch/"
        controller_yaml_suffix = "_position_controller_gazebo.yaml"
        controller_launch_suffix = "_controllers_prefix.launch"
        traj_suffix = "_trajectory_controller.yaml"
        bringup_suffix = "_bringup.launch"

        stream = open(robots_yaml_file, 'r')
        yamldoc = yaml.load(stream)

        if source_profile in yamldoc['tools']: 
            if target_profile in yamldoc['tools']:  
                if source_profile != target_profile:
                    
                    # print cmd
                    
                    create_symlink(agni_description_config_path, source_profile, target_profile, controller_yaml_suffix)
                    create_symlink(agni_description_config_path, source_profile, target_profile, traj_suffix)
 
                    # print cmd
                    create_symlink(agni_deploy_launch_path, source_profile, target_profile, controller_launch_suffix)
                    create_symlink(agni_deploy_launch_path, source_profile, target_profile, bringup_suffix)
            else:
                print target_profile , "does not exist"
        else:
            print source_profile , "does not exist"
    else:
        print ("source or target profile not provided")
