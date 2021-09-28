#!/usr/bin/env python
#-*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
#  * Neither the name of SRI International nor the names of its
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
#
# Author: Seong-Woo Kim

## `rospy` 와 메시지들을 import 합니다:

import sys, csv
import rospkg
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import Constraints, JointConstraint
from ar1440_apps.msg import WeldingInfo
from ar1440_apps.srv import WeldingCommand

rospy.init_node('weldingbot_publisher')

pub = rospy.Publisher('weldinginfo', WeldingInfo, queue_size=10)

rospy.wait_for_service('weldingcommand')
# weldingcommand service - 0: reset, 1: start, 2: stop, response - 0: OK
weldingcommander = rospy.ServiceProxy('weldingcommand', WeldingCommand)

rospack = rospkg.RosPack()
root_path = rospack.get_path('ar1440_apps')

from PyQt5 import QtCore, QtWidgets
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5.QtCore import pyqtSignal,pyqtSlot

form_class = uic.loadUiType(root_path + "/scripts/weldingbot.ui")[0]

class MyWindow(QMainWindow,form_class):
    def __init__(self):
        super(QMainWindow, self).__init__()
        self.setupUi(self)

        # DHBeam Type Widget
        self.typeComboBox.addItem("DHBeam Type 1")
        self.typeComboBox.addItem("DHBeam Type 2")
        self.typeComboBox.activated[str].connect(self.typeActivated)

        # Offset LineEdit Widget
        self.offset = [ 0.0, 0.0, 0.0]
        self.beamXLineEdit.textChanged.connect(self.beamXTextSet)
        self.beamYLineEdit.textChanged.connect(self.beamYTextSet)
        self.beamZLineEdit.textChanged.connect(self.beamZTextSet)

        # Plate Table Widget
        p_column_headers = ['위치', 'X','Y','Z','R','P','Y','DX','DY','DZ']
        self.p_data = {}
        #self.p_data = { 0: [0, 0,0,0,0,0,0,0.3,1.0,0.02],
        #           1: [1, -0.25,0,0,0,0,0,0.02,1.0,0.6],
        #           2: [2,0.25,0,0,0,0,0,0.02,1.0,0.6] }
        self.plateTableWidget.setColumnCount(10)
        #self.plateTableWidget.setRowCount(3)
        self.plateTableWidget.setHorizontalHeaderLabels(p_column_headers)

        # Welding Point Table Widget
        wp_column_headers = ['위치', 'X', 'Y', 'Z']
        self.wp_data = {}
        #self.wp_data = { 0: ['아래', 1.0, 1.0, 1.0],
        #            1: ['아래', 1.0, -1.0, 1.0],
        #            2: ['아래', 2.0, 1.0, 1.0],
        #            3: ['아래', 2.0, -1.0, 1.0] }
        self.wpTableWidget.setColumnCount(4)
        #self.wpTableWidget.setRowCount(4)
        self.wpTableWidget.setHorizontalHeaderLabels(wp_column_headers)

        self.resetButton.clicked.connect(self.resetButtonClicked)
        self.sendButton.clicked.connect(self.sendButtonClicked)
        self.startButton.clicked.connect(self.startButtonClicked)

    def updatePData(self, p_data):
        self.plateTableWidget.setRowCount(len(p_data))
        for row, v in p_data.items():
            for col, val in enumerate(v):
                item = QTableWidgetItem(str(val))
                self.plateTableWidget.setItem(row, col, item)
        self.plateTableWidget.resizeColumnsToContents()
        self.plateTableWidget.resizeRowsToContents()

    def updateWPData(self, wp_data):
        self.wpTableWidget.setRowCount(len(wp_data))
        for row, v in wp_data.items():
            for col, val in enumerate(v):
                item = QTableWidgetItem(str(val))
                self.wpTableWidget.setItem(row, col, item)
        self.wpTableWidget.resizeColumnsToContents()
        self.wpTableWidget.resizeRowsToContents()

    def typeActivated(self, text):
        print("DHBeam Type:", text)
        self.readcsv(self.typeComboBox.currentIndex())
        self.updatePData(self.p_data)
        self.updateWPData(self.wp_data)

    def beamXTextSet(self, text):
        self.offset[0] = float(text)

    def beamYTextSet(self, text):
        self.offset[1] = float(text)

    def beamZTextSet(self, text):
        self.offset[2] = float(text)

    def resetButtonClicked(self):
        #self.resetData.setPlainText("resetButtonClicked")
        self.p_data = {}
        self.wp_data = {}
        self.updatePData(self.p_data)
        self.updateWPData(self.wp_data)
        weldingcommander(0)
        pass

    def sendButtonClicked(self):
        #self.sendBtnData.setPlainText("sendButtonClicked")
        if len(self.p_data) == 0 and len(self.wp_data) == 0:
            print('Empty plates and welding points')
            return

        msg = WeldingInfo()
        msg.offset = geometry_msgs.msg.Point32()
        msg.offset.x = self.offset[0]
        msg.offset.y = self.offset[1]
        msg.offset.z = self.offset[2]
        msg.plate = []
        for row, v in self.p_data.items():
            plate = moveit_msgs.msg.OrientedBoundingBox()
            plate.pose.position.x = float(v[1])
            plate.pose.position.y = float(v[2])
            plate.pose.position.z = float(v[3])
            q = quaternion_from_euler(float(v[4]), float(v[5]), float(v[6]))
            plate.pose.orientation.x = q[0]
            plate.pose.orientation.y = q[1]
            plate.pose.orientation.z = q[2]
            plate.pose.orientation.w = q[3]
            plate.extents.x = float(v[7])
            plate.extents.y = float(v[8])
            plate.extents.z = float(v[9])
            msg.plate.append(plate)
        msg.weldingtype = []
        for row, v in self.wp_data.items():
            msg.weldingtype.append(int(v[0]))
        msg.weldingpoint = []
        for row, v in self.wp_data.items():
            point = geometry_msgs.msg.Point32()
            point.x = float(v[1])
            point.y = float(v[2])
            point.z = float(v[3])
            msg.weldingpoint.append(point)
        pub.publish(msg)

    def startButtonClicked(self):
        #self.startBtnData.setPlainText("startButtonClicked")
        weldingcommander(1)
        pass

    def readcsv(self, id):
        fname = root_path + '/worlds/' + 'dhbeam' + str(id+1) + '.csv'
        with open(fname) as file:
            mode = 0
            data = csv.reader(file)
            self.p_data = {}
            self.wp_data = {}
            for item in data:
                if '#plate' in item[0]:
                    mode = 1
                    idx = 0
                    #print("plate mode")
                    continue
                elif '#welding' in item[0]:
                    mode = 2
                    idx = 0
                    #print("wp mode")
                    continue
                if mode == 1:
                    self.p_data[idx] = item
                    idx = idx + 1
                elif mode == 2:
                    self.wp_data[idx] = item
                    idx = idx + 1
                #print(item)

    @pyqtSlot(str)
    def printRcv(self, rcvData):
        self.rcvData.setPlainText(rcvData)

class RobotServer(QtCore.QObject):
    rcvdMsg = pyqtSignal(str)

    def __init__(self, parent):
        super(QtCore.QObject, self).__init__(parent)

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    ex = MyWindow()
    ex.show()
    app.exec_()
