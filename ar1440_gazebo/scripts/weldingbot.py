#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys, csv
import rospkg

rospack = rospkg.RosPack()
root_path = rospack.get_path('ar1440_gazebo')

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

        # Plate Table Widget
        p_column_headers = ['위치', 'X','Y','Z','R','P','Y','DX','DY','DZ']
        self.p_data = { 0: [0, 0,0,0,0,0,0,0.3,1.0,0.02],
                   1: [1, -0.25,0,0,0,0,0,0.02,1.0,0.6],
                   2: [2,0.25,0,0,0,0,0,0.02,1.0,0.6] }
        self.plateTableWidget.setColumnCount(10)
        #self.plateTableWidget.setRowCount(3)
        self.plateTableWidget.setHorizontalHeaderLabels(p_column_headers)

        # Welding Point Table Widget
        wp_column_headers = ['위치', 'X', 'Y', 'Z']
        self.wp_data = { 0: ['아래', 1.0, 1.0, 1.0],
                    1: ['아래', 1.0, -1.0, 1.0],
                    2: ['아래', 2.0, 1.0, 1.0],
                    3: ['아래', 2.0, -1.0, 1.0] }
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

    def resetButtonClicked(self):
        #self.resetData.setPlainText("resetButtonClicked")
        pass

    def sendButtonClicked(self):
        #self.sendBtnData.setPlainText("sendButtonClicked")
        pass

    def startButtonClicked(self):
        #self.startBtnData.setPlainText("startButtonClicked")
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
