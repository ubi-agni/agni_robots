#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# derived from emergency_buttons_dashboard.py  rqt_lwr_dashboard
#
# derived from rqt_emergency_buttons: emergency_buttons_dashboard.py
#   original Authors Sammy Pfeiffer
#
# Copyright (c) 2015 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#

# author Guillaume WALCK (2016)
import rospy
import functools
import time
from rqt_robot_dashboard.dashboard import Dashboard


from PyQt5.QtGui import QFont, QColor
from PyQt5.QtCore import QSize, QRect, Qt, QTimer, \
    QObject, QMetaObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QPushButton, QWidget, QLabel, QSlider, QTreeWidgetItem, \
    QCheckBox, QMessageBox, QLayout, QVBoxLayout, QHBoxLayout, QTreeWidget


from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import JointState
from sr_utilities.hand_finder import HandFinder
from .controller_switcher import ControllerSwitcher

from .state_button import ControlStateButton

CHECK_STATE_TIMEOUT = 2.0  # seconds of no diagnostic before considering the driver is dead
NOT_RUNNING = 0
PARTIALLY_RUNNING = 1
ALL_RUNNING = 2

HIGH_FORCE = 220
LOW_FORCE = 120

green = QColor(153, 231, 96)
orange = QColor(247, 206, 134)
red = QColor(236, 178, 178)

class RqtSrDashboard(Dashboard):
    """
    Dashboard for Shadow Hands
    """

    _datachanged = pyqtSignal()

    def setup(self, context):
        """

        :type self: object
        """
        self.name = 'Shadow Robot Buttons Dashboard'
        self.max_icon_size = QSize(50, 30)

        self._state_buttons = {}
        self._cs = {}
        self._widget_initialized = False

        self._hand_names = []
        self._driver_started = {}
        self._driver_running = {}
        self._ctrl_state = {}
        self._last_seen = {}
        self._force_display= {}
        self._last_force_val = {}
        # setting the prefixes
        self._hand_finder = HandFinder()
        hand_parameters = self._hand_finder.get_hand_parameters()

        # create a joint list for force display
        joints = self._hand_finder.get_hand_joints()
        self._joint_list = QTreeWidget()
        self._joint_list.setAnimated(True)
        self._joint_list.setColumnCount(len(joints))
        self._prefix_to_col = {}
        header_labels = ["Joint"]
        line_labels = [""]*(1+len(joints))
        self._force_display = {}
        for i, prefix in enumerate(joints):
            self._prefix_to_col[prefix] = 1 + i
            header_labels.append(prefix)
            for joint in joints[prefix]:
                [joint_prefix, base_jointname] = joint.split("_")
                if base_jointname in self._force_display:
                    self._force_display[base_jointname].setBackground(1 + i, QColor(green))
                else:  # create a new line
                    line_labels[0] = base_jointname
                    self._force_display[base_jointname] = QTreeWidgetItem(self._joint_list, line_labels)
                    self._force_display[base_jointname].setBackground(1 + i, QColor(green))
                    self._joint_list.addTopLevelItem(self._force_display[base_jointname])
        self._joint_list.setHeaderLabels(header_labels)

        # create as many buttons as hands found
        for hand in hand_parameters.mapping:
            self._state_buttons[hand_parameters.mapping[hand]] = ControlStateButton(hand_parameters.mapping[hand], self)

            # we do not consider the trajectory controllers
            self._cs[hand_parameters.mapping[hand]] = ControllerSwitcher(namespace=hand_parameters.mapping[hand],
                                                                         only_low_level_ctrl=True)
            self._hand_names.append(hand_parameters.mapping[hand])
            self._driver_started[hand_parameters.mapping[hand]] = False
            self._driver_running[hand_parameters.mapping[hand]] = False
            self._ctrl_state[hand_parameters.mapping[hand]] = NOT_RUNNING
            self._last_seen[hand_parameters.mapping[hand]] = rospy.Time(0)

        self._main_widget = QWidget()
        vlayout = QVBoxLayout()

        # create buttons
        self.btn_control = QPushButton("control on")
        self.btn_standby = QPushButton("standby")

        # disable buttons by default
        self.btn_control.setEnabled(False)
        self.btn_standby.setEnabled(False)

        # place buttons
        vlayout.addWidget(self.btn_control)
        vlayout.addWidget(self.btn_standby)
        vlayout.addWidget(self._joint_list)

        # signals for buttons
        self.btn_control.clicked.connect(functools.partial(self.on_btn_control_clicked, hand_name=None))
        self.btn_standby.clicked.connect(functools.partial(self.on_btn_standby_clicked, hand_name=None))

        self._datachanged.connect(self.updaterSlot)

        self._main_widget.setLayout(vlayout)
        self.context.add_widget(self._main_widget)
        # self._main_widget.addLayout(hlayout)
        self._widget_initialized = True

        # create diagnostic subscribers and js subscribers
        self._diag_sub = {}
        self._js_sub = {}
        for hand in hand_parameters.mapping:
            self._diag_sub[hand_parameters.mapping[hand]] = rospy.Subscriber(
                "/" + hand_parameters.mapping[hand] + "/diagnostics/", DiagnosticArray,
                functools.partial(self.diag_callback, hand_name=hand_parameters.mapping[hand]))
            self._js_sub[hand_parameters.mapping[hand]] = rospy.Subscriber(
                "/" + hand_parameters.mapping[hand] + "/joint_states/", JointState,
                functools.partial(self.js_callback, hand_name=hand_parameters.mapping[hand]))

        self._timer = QTimer()
        self._timer.timeout.connect(self.check_state_callback)
        self._timer.start(CHECK_STATE_TIMEOUT * 1000.0)
        
        self._monitor_timer = QTimer()
        self._monitor_timer.timeout.connect(self.updaterSlot)
        self._monitor_timer.start(500.0)

    def on_btn_control_clicked(self, hand_name=None):
        """
        switch to command mode
        :param hand_name: group concerned, default is None meaning act on all enabled groups

        """
        if hand_name is not None:
            if self._driver_started[hand_name]:
                self._cs[hand_name].start_controllers()
        else:
            for name in self._hand_names:
                if self._driver_started[name]:
                    self._cs[name].start_controllers()

        self.check_state_callback()

    def on_btn_standby_clicked(self, hand_name=None):
        """
        switch to monitor mode
        :param hand_name: group concerned, default is None meaning act on all enabled groups
        """
        if hand_name is not None:
            if self._driver_started[hand_name]:
                self._cs[hand_name].stop_controllers()
        else:
            for name in self._hand_names:
                if self._driver_started[name]:
                    self._cs[name].stop_controllers()

        self.check_state_callback()

    def get_widgets(self):
        widgets_list = []
        for hand_name in self._state_buttons:
            widgets_list.append([self._state_buttons[hand_name]])

        return widgets_list

    def check_state_callback(self):

        state_changed = False
        for hand_name in self._hand_names:
            sec_since_last_seen = (rospy.Time.now() - self._last_seen[hand_name]).to_sec()
            if sec_since_last_seen > CHECK_STATE_TIMEOUT:
                rospy.logdebug("hand not seen for " + str(sec_since_last_seen) + " seconds")
                if self._driver_started[hand_name]:
                    self._driver_started[hand_name] = False
                    self._driver_running[hand_name] = False
                    state_changed = True
            nb_ctrl = len(self._cs[hand_name].get_running_controllers("position"))
            if nb_ctrl == 0:
                if self._ctrl_state[hand_name] != NOT_RUNNING:
                    self._ctrl_state[hand_name] = NOT_RUNNING
                    state_changed = True
            else:
                if nb_ctrl < 20:
                    if self._ctrl_state[hand_name] != PARTIALLY_RUNNING:
                        self._ctrl_state[hand_name] = PARTIALLY_RUNNING
                        state_changed = True
                else:
                    if self._ctrl_state[hand_name] != ALL_RUNNING:
                        self._ctrl_state[hand_name] = ALL_RUNNING
                        state_changed = True

        if state_changed:
            self.update_state()

    def diag_callback(self, msg, hand_name):
        """
        callback to process sr diagnostic messages
        :param msg:
        :type msg: Diagnostic
        """

        state_changed = False
        if len(msg.status) > 1:
            for state in msg.status:
                if state.name == hand_name + " EtherCAT Dual CAN Palm":
                    self._last_seen[hand_name] = rospy.Time.now()
                    self._driver_started[hand_name] = True
                    if state.level == 0:
                        if not self._driver_running[hand_name]:
                            self._driver_running[hand_name] = True
                            state_changed = True
                    else:
                        if self._driver_running[hand_name]:
                            self._driver_running[hand_name] = False
                            state_changed = True
        if state_changed:
            self.update_state()

    def js_callback(self, msg, hand_name):
        """
        callback to process joint_states messages
        :param msg:
        :type msg: JointState
        """
        for i, joint_name in enumerate(msg.name):
            [prefix, base_jointname] = joint_name.split("_")
            if prefix not in self._last_force_val:
                self._last_force_val[prefix] = {}
            self._last_force_val[prefix][base_jointname] =  msg.effort[i]
           
        # was updating too often self._datachanged.emit()

    @pyqtSlot()
    def updaterSlot(self):

        for prefix in self._last_force_val:
            if prefix in self._prefix_to_col:
                col = self._prefix_to_col[prefix]
            else:
                continue
            for base_jointname in self._last_force_val[prefix]:
                if base_jointname in self._force_display:
                    self._force_display[base_jointname].setText(col, str('%.0f' % self._last_force_val[prefix][base_jointname]))

                    if abs(self._last_force_val[prefix][base_jointname]) > HIGH_FORCE:
                        self._force_display[base_jointname].setBackground(col, QColor(red))
                    else:
                        if abs(self._last_force_val[prefix][base_jointname]) > LOW_FORCE:
                            self._force_display[base_jointname].setBackground(col, QColor(orange))
                        else:
                            self._force_display[base_jointname].setBackground(col, QColor(green))

        self._joint_list.update()
        

    def update_state(self):

        enable_buttons = 0
        for hand_name in self._hand_names:
            # update buttons on driver state change
            enable_buttons += 1
            if self._driver_started[hand_name]:
                if not self._driver_running[hand_name]:
                    self._state_buttons[hand_name].set_state("error")
                    enable_buttons -= 1
                    rospy.logdebug("error " + hand_name)
                else:
                    if self._ctrl_state[hand_name] == NOT_RUNNING:
                        self._state_buttons[hand_name].set_state("standby")
                        rospy.logdebug("standby " + hand_name)
                    if self._ctrl_state[hand_name] == PARTIALLY_RUNNING:
                        self._state_buttons[hand_name].set_state("partial_run")
                        rospy.logdebug("partial run " + hand_name)
                    if self._ctrl_state[hand_name] == ALL_RUNNING:
                        self._state_buttons[hand_name].set_state("run")
                        rospy.logdebug("run " + hand_name)
            else:
                self._state_buttons[hand_name].set_state("disabled")
                rospy.logdebug("disable " + hand_name)
                enable_buttons -= 1

        if enable_buttons > 0:
            self.btn_control.setEnabled(True)
            self.btn_standby.setEnabled(True)
        else:
            self.btn_control.setEnabled(False)
            self.btn_standby.setEnabled(False)

    def shutdown_dashboard(self):
        self._timer.stop()
        self._monitor_timer.stop()
        for hand in self._diag_sub:
            self._diag_sub[hand].unregister()
            self._js_sub[hand].unregister()

