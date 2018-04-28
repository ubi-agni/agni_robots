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

from qt_gui.plugin import Plugin

from PyQt5.QtGui import QFont, QColor
from PyQt5.QtCore import QSize, QRect, Qt, QTimer, \
    QObject, QMetaObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QPushButton, QWidget, QLabel, QTreeWidgetItem, \
    QCheckBox, QMessageBox, QLayout, QVBoxLayout, QHBoxLayout, QTreeWidget


from sensor_msgs.msg import JointState
from sr_utilities.hand_finder import HandFinder

HIGH_FORCE = 220
LOW_FORCE = 120

green = QColor(153, 231, 96)
orange = QColor(247, 206, 134)
red = QColor(236, 178, 178)



class SrForceDisplay(Plugin):

    """
    A rosgui plugin for displaying forces on the Shadow EtherCAT Hand
    """
    _datachanged = pyqtSignal()

    def __init__(self, context):
        super(SrForceDisplay, self).__init__(context)
        self.setObjectName('SrForceDisplay')

        self._widget = QWidget()

        context.add_widget(self._widget)

        self.name = 'Shadow Robot Force Display'
        self.max_icon_size = QSize(50, 30)

        self._widget_initialized = False

        self._hand_names = []
        self._force_val= {}
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
        self._force_val = {}
        for i, prefix in enumerate(joints):
            self._prefix_to_col[prefix] = 1 + i
            header_labels.append(prefix)
            for joint in joints[prefix]:
                [joint_prefix, base_jointname] = joint.split("_")
                if base_jointname in self._force_val:
                    self._force_val[base_jointname].setBackground(1 + i, QColor(green))
                else:  # create a new line
                    line_labels[0] = base_jointname
                    self._force_val[base_jointname] = QTreeWidgetItem(self._joint_list, line_labels)
                    self._force_val[base_jointname].setBackground(1 + i, QColor(green))
                    self._joint_list.addTopLevelItem(self._force_val[base_jointname])
        self._joint_list.setHeaderLabels(header_labels)


        vlayout = QVBoxLayout()
        vlayout.addWidget(self._joint_list)

        self._datachanged.connect(self.updaterSlot)

        self._widget.setLayout(vlayout)
        # self._main_widget.addLayout(hlayout)
        self._widget_initialized = True

        # create js subscribers
        self._js_sub = {}
        for hand in hand_parameters.mapping:
            self._js_sub[hand_parameters.mapping[hand]] = rospy.Subscriber(
                "/" + hand_parameters.mapping[hand] + "/joint_states/", JointState,
                functools.partial(self.js_callback, hand_name=hand_parameters.mapping[hand]))


    def js_callback(self, msg, hand_name):
        """
        callback to process joint_states messages
        :param msg:
        :type msg: JointState
        """
        for i, joint_name in enumerate(msg.name):
            [prefix, base_jointname] = joint_name.split("_")
            if prefix in self._prefix_to_col:
                col = self._prefix_to_col[prefix]
            else:
                continue
            if base_jointname in self._force_val:
                self._force_val[base_jointname].setText(col, str('%.2f' % msg.effort[i]))

                if msg.effort[i] > HIGH_FORCE:
                    self._force_val[base_jointname].setBackground(col, QColor(red))
                else:
                    if msg.effort[i] > LOW_FORCE:
                        self._force_val[base_jointname].setBackground(col, QColor(orange))
                    else:
                        self._force_val[base_jointname].setBackground(col, QColor(green))

        self._joint_list.update()
        self._datachanged.emit()

    @pyqtSlot()
    def updaterSlot(self):

        self._joint_list.update()
        #self._joint_list.setCurrentItem(self._force_val["FFJ1"])

    def _unregisterPublisher(self):
        for hand in self._js_sub:
            self._js_sub[hand].unregister()

    def shutdown_plugin(self):
        self._unregisterPublisher()

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass
