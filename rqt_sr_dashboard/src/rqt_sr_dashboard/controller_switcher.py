#!/usr/bin/env python

# Author Guillaume Walck 2016
#

import rospy
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

TRAJ_CTRL_NAME = "position_controllers/JointTrajectoryController"


class ControllerSwitcher(object):

    managed_controller_types = ["sr_mechanism_controllers/SrhMixedPositionVelocityJointController",
                                "sr_mechanism_controllers/SrhJointPositionController",
                                "position_controllers/JointTrajectoryController"]

    def __init__(self, namespace, only_low_level_ctrl=False):

        self.namespace = namespace+"/"
        rospy.loginfo("controller switcher using ns:" + self.namespace)
        self.traj_controller_name = None
        self.only_low_level_ctrl = only_low_level_ctrl
        self.managed_controllers = []
        self.list_controllers = rospy.ServiceProxy(self.namespace + 'controller_manager/list_controllers',
                                                   ListControllers)
        self.switch_controllers = rospy.ServiceProxy(self.namespace + 'controller_manager/switch_controller',
                                                     SwitchController)
        self.init_controller_list()

    def init_controller_list(self):
        """
        store controllers for managed types
        """
        success = True
        try:
            resp1 = self.list_controllers()
        except rospy.ServiceException:
            success = False

        if success:
            for c in resp1.controller:

                # store the name of the trajectory controller separately
                if c.type == TRAJ_CTRL_NAME:
                    if self.traj_controller_name is None:
                        self.traj_controller_name = c.name
                    else:
                        rospy.logwarn("Two trajectory controllers found,\
                                      this is not valid,\
                                      keeping the first one found")

                if c.type in self.managed_controller_types:
                    if self.only_low_level_ctrl and c.type == TRAJ_CTRL_NAME:
                        continue
                    self.managed_controllers.append(c.name)
            rospy.logdebug("managed_controllers: ")
            rospy.logdebug(self.managed_controllers)
        else:
            rospy.logwarn("did not find the controller manager service: controller_manager/list_controllers")

    def stop_controllers(self):
        """
        stop managed and running controllers
        """
        success = True
        try:
            resp1 = self.list_controllers()
        except rospy.ServiceException:
            success = False

        if success:
            controllers_to_stop = [
                c.name for c in resp1.controller
                if c.state == "running" and c.name in self.managed_controllers]

            controllers_to_start = [""]
            req = SwitchControllerRequest()
            req.start_controllers = controllers_to_start
            req.stop_controllers = controllers_to_stop
            req.strictness = SwitchControllerRequest.BEST_EFFORT
            # only for newest control messages, using defaults for now
            # req.start_asap = False
            # req.timeout =  0.0

            try:
                resp1 = self.switch_controllers.call(req)
            except rospy.ServiceException:
                success = False

            if not resp1.ok:
                success = False

        if not success:
            rospy.logwarn(
                "Failed to change some of the controllers. ")

    def start_controllers(self):
        """
        start managed and stopped controllers
        """
        success = True
        try:
            resp1 = self.list_controllers()
        except rospy.ServiceException:
            success = False

        if success:
            controllers_to_start = [
                c.name for c in resp1.controller
                if (c.state == "stopped" or c.state == "initialized") and c.name in self.managed_controllers]

            if len(controllers_to_start)==0:
                rospy.logwarn("no controllers to start, check their state")
            rospy.logdebug(controllers_to_start)
            controllers_to_stop = [""]

            req = SwitchControllerRequest()
            req.start_controllers = controllers_to_start
            req.stop_controllers = controllers_to_stop
            req.strictness = SwitchControllerRequest.BEST_EFFORT
            # only for newest control messages, using defaults for now
            # req.start_asap = False
            # req.timeout =  0.0

            try:
                resp1 = resp1 = self.switch_controllers.call(req)
            except rospy.ServiceException:
                success = False

            if not resp1.ok:
                success = False

        if not success:
            rospy.logwarn(
                "Failed to change some of the controllers. ")

    def get_trajectory_controller_name(self):
        """
        Returns trajectory controller name
        """
        if self.traj_controller_name is not None:
            return self.traj_controller_name
        else:
            return ""

    def get_running_controllers(self, ctrl_type=None):
        """
        Returns a list of running controllers of given type
        """
        ctrl_list = []

        try:
            resp1 = self.list_controllers()

            for c in resp1.controller:
                if c.state == "running":
                    if ctrl_type is not None:
                        if ctrl_type not in c.type.lower():
                            continue
                    # don't count trajectory controller if only low level
                    if self.only_low_level_ctrl and c.type == TRAJ_CTRL_NAME:
                        continue
                    ctrl_list.append(c.name)

        except rospy.ServiceException:
            pass
        return ctrl_list
