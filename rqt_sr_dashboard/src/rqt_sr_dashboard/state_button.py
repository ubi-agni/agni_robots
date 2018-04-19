#!/usr/bin/env python
#
# author Guillaume Walck (2015)
#
# derived from pr2_breaker.py
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import rospy
import functools
from python_qt_binding.QtCore import QSize, pyqtSignal

from rqt_robot_dashboard.widgets import MenuDashWidget

from python_qt_binding.QtWidgets import QMessageBox


class ControlStateButton(MenuDashWidget):
    """
    Dashboard widget to display and interact with the shadow hand state.
    """

    def __init__(self, hand_name, parent):
        """
        :param hand_name: Name of the hand
        :type hand_name: str
        """

        self._serial = 0
        self._parent = parent
        self._name = hand_name

        if hand_name == 'lh':
            state_icon = 'ic-lhand.svg'
        elif hand_name == 'rh':
            state_icon = 'ic-rhand.svg'
        else:
            state_icon = 'ic-breaker.svg'

        run_icon = ['bg-green.svg', state_icon]
        partial_run_icon = ['bg-yellow.svg', state_icon]
        standby_icon = ['bg-grey.svg', state_icon]
        error_icon = ['bg-red.svg', state_icon, 'ol-err-badge.svg']
        disabled_icon = ['bg-light_grey.svg', state_icon]

        icons = [disabled_icon, error_icon, standby_icon, partial_run_icon, run_icon]
        self._state_dict = {"disabled": 0, "error": 1, "standby": 2, "partial_run": 3, "run": 4}

        super(ControlStateButton, self).__init__('State:' + hand_name, icons=icons, icon_paths=[['rqt_sr_dashboard', 'images']])

        # init the button in disabled
        self.update_state(0)

        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))

        self.add_action('Control', functools.partial(self._parent.on_btn_control_clicked, hand_name=self._name))
        self.add_action('Standby', functools.partial(self._parent.on_btn_standby_clicked, hand_name=self._name))

        self._pending_msg = None
        self._state = None
        self._last_status_msg = None
        self.setToolTip(hand_name)

    def set_state(self, state):
        """
        Sets state of button based on msg

        :param msg: message containing the M3 control state
        :type msg: m3meka_msgs.msg.M3ControlStates
        """

        status_msg = "Running"
        # if first message received, enable the group
        if self._state is None:
            self.set_group_enabled(True)
        if self._state != state:
            self._state = state
            if state in self._state_dict:
                self.update_state(self._state_dict[state])

            if status_msg != self._last_status_msg:
                self.setToolTip("Group: %s \nState: %s" % (self._name, status_msg))
                self._last_status_msg = status_msg

    def set_group_enabled(self, val):
        if not val:
            self.update_state(0)

