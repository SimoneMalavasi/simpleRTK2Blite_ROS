#!/usr/bin/env python
# coding: utf8

import rospy
import rostopic
import subprocess, shlex, psutil
from PySide2.QtCore import QSize, Qt,Slot
from PySide2.QtWidgets import QApplication, QWidget, QMainWindow, QLabel, QPushButton, QFileDialog,  QLineEdit
from PySide2.QtGui import QPixmap, QTransform
import sys
from sensor_msgs.msg import NavSatFix
import math
import numpy as np

red = '#ff0000'
yellow = '#ffff00'
green = '#00ff00'
lat_deg_km = 110.574     # [km/deg]
lon_deg_km = 111.320    # [km/deg]
meanLat = math.radians(45.454847)   # TODO: CHANGE THIS VALUE WITH THE RIGHT ONE
gps1LedPosition = (100, 100)
gps2LedPosition = (200, 100)
carPosition = (350, 100)
buttonPosition = (400, 200)
lineBagPosition = (130, 250)
ledSize = (50, 50)
carSize = (40, 70)
gps1HzPosition = (100, 300)
buttonSize = (100, 30)
lineBagSize = (200, 30)

class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Dashboard")
        self.setFixedSize(QSize(500, 400))
        self.pixmapGreenLed = QPixmap('/home/simone/Scaricati/green-led.png')
        self.pixmapRedLed = QPixmap('/home/simone/Scaricati/red-led.png')
        self.pixmapYellowLed = QPixmap('/home/simone/Scaricati/yellow-led.png')
        self.pixmapCar = QPixmap('/home/simone/Scaricati/car.png')
        self.pixmapGreenLed = self.pixmapGreenLed.scaled(ledSize[0], ledSize[1])
        self.pixmapRedLed = self.pixmapRedLed.scaled(ledSize[0], ledSize[1])
        self.pixmapCar = self.pixmapCar.scaled(carSize[0], carSize[1])
        self.buttonStart = QPushButton("Start", self)
        self.buttonStart.move(buttonPosition[0], buttonPosition[1])
        self.buttonStart.setFixedSize(QSize(buttonSize[0], buttonSize[1]))
        self.buttonStop = QPushButton("Stop", self)
        self.buttonStop.move(buttonPosition[0], buttonPosition[1])
        self.buttonStop.setFixedSize(QSize(buttonSize[0], buttonSize[1]))
        self.buttonStop.setVisible(0)
        self.buttonBrowse = QPushButton("Select Bag Folder", self)
        self.buttonBrowse.move(buttonPosition[0]-50, buttonPosition[1]+50)
        self.buttonBrowse.setFixedSize(QSize(buttonSize[0]+50, buttonSize[1]))
        self.labelBag = QLabel(self)
        self.labelBag.setText("Bag Name: ")
        self.labelBag.move(lineBagPosition[0]-100, lineBagPosition[1])
        self.gps1Led = QLabel(self)
        self.lineBag = QLineEdit(self)
        self.lineBag.move(lineBagPosition[0], lineBagPosition[1])
        self.lineBag.setFixedSize(QSize(lineBagSize[0], lineBagSize[1]))
        self.gps1Led = QLabel(self)
        self.gps1Led.setFixedSize(QSize(ledSize[0], ledSize[1]))
        self.gps1Led.setPixmap(self.pixmapYellowLed)
        self.gps1Led.move(gps1LedPosition[0], gps1LedPosition[1])
        self.gps1Text = QLabel(self)
        self.gps1Text.setText("GPS 1")
        self.gps1Text.move(gps1LedPosition[0], gps1LedPosition[1]+50)
        self.fix1Text = QLabel(self)
        self.fix1Text.setText("FIX")
        self.fix1Text.move(gps1LedPosition[0], gps1LedPosition[1]-30)
        self.percentage1Text = QLabel(self)
        self.percentage1Text.setText("")
        self.percentage1Text.move(gps1LedPosition[0], gps1LedPosition[1]+80)
        self.gps1HzText = QLabel(self)
        self.gps1HzText.setText("")
        self.gps1HzText.move(gps1HzPosition[0], gps1HzPosition[1])
        self.gps2Led = QLabel(self)
        self.gps2Led.setFixedSize(QSize(ledSize[0], ledSize[1]))
        self.gps2Led.setPixmap(self.pixmapYellowLed)
        self.gps2Led.move(gps2LedPosition[0], gps2LedPosition[1])
        self.gps2Text = QLabel(self)
        self.gps2Text.setText("GPS 2")
        self.gps2Text.move(gps2LedPosition[0]+5, gps2LedPosition[1]+50)
        self.fix2Text = QLabel(self)
        self.fix2Text.setText("FIX")
        self.fix2Text.move(gps2LedPosition[0]+5, gps2LedPosition[1]-30)
        self.percentage2Text = QLabel(self)
        self.percentage2Text.setText("")
        self.percentage2Text.move(gps2LedPosition[0], gps2LedPosition[1]+80)
        self.gps2HzText = QLabel(self)
        self.gps2HzText.setText("")
        self.gps2HzText.move(gps1HzPosition[0], gps1HzPosition[1]+30)
        self.car = QLabel(self)
        self.car.setFixedSize(QSize(carSize[0], carSize[1]))
        self.car.setPixmap(self.pixmapCar)
        self.car.move(carPosition[0], carPosition[1])
        self.egoLatF = 0.0
        self.egoLonF = 0.0
        self.egoLatB = 0.0
        self.egoLonB = 0.0
        self.egoHead = 0.0
        self.old_egoHead = 0.0
        self.r1 = rostopic.ROSTopicHz(-1)
        self.r2 = rostopic.ROSTopicHz(-1)
        self.egoGpsPoseF = np.array([[0.0], [0.0], [0.0]], dtype="float64")
        self.egoGpsPoseB = np.array([[0.0], [0.0], [0.0]], dtype="float64")
        self.fix1 = -100
        self.fix2 = -100
        self.old_fix1 = -10
        self.old_fix2 = -10
        self.gps1Hz = 0.0
        self.gps2Hz = 0.0
        self.countLogGps1 = 0.0  # counter for get overall number GPS1 sample
        self.countLogGps2 = 0.0  # counter for get overall number GPS2 sample
        self.countLogFix1 = 0.0  # counter for get % fix GPS1
        self.countLogFix2 = 0.0  # counter for get % fix GPS2
        self.percentageFix1 = 0.0
        self.percentageFix2 = 0.0
        self.keepCount = 0
        self.bag_dir = ""
        self.bag_command = ""
        self.bag_command_sel = ""
        self.bag_proc = ""  # REQUIRED FOR SAVING BAG
        self.buttonStart.clicked.connect(self.start_logging)
        self.buttonStop.clicked.connect(self.stop_logging)
        self.buttonBrowse.clicked.connect(self.select_bag_folder)
        self.subscribe_data()

    def subscribe_data(self):
        self.subGPS1Data = rospy.Subscriber("/gps1/fix", NavSatFix, self.get_gps1_data, queue_size=1)
        self.subGPS2Data = rospy.Subscriber("/gps2/fix", NavSatFix, self.get_gps2_data, queue_size=1)

    def get_gps1_data(self, msg):
        self.fix1 = msg.status.status
        self.egoLatB = msg.latitude
        self.egoLonB = msg.longitude
        if self.old_fix1 != self.fix1:
            if self.fix1 == -1:
                self.gps1Led.setPixmap(self.pixmapRedLed)
                self.fix1Text.setText("NO FIX")
            if self.fix1 == 0:
                self.gps1Led.setPixmap(self.pixmapYellowLed)
                self.fix1Text.setText("FIX")
            if self.fix1 == 2:
                self.gps1Led.setPixmap(self.pixmapGreenLed)
                self.fix1Text.setText("FIX RTK")
        if self.keepCount:
            self.countLogGps1 += 1  # counter for get % fix GPS1
            if self.fix1 == 2:
                self.countLogFix1 += 1
            if self.countLogGps1 != 0:
                self.percentageFix1 = self.countLogFix1/self.countLogGps1 * 100
                self.percentage1Text.setText(str(round(self.percentageFix1, 2)) + ' %')
        self.r1.callback_hz(NavSatFix, topic='gps1/fix')
        self.gps1HzText.setText('GPS1: ' + str(round(self.r1.get_hz(topic='gps1/fix')[0], 2))+' Hz')
        self.old_fix1 = self.fix1

    def get_gps2_data(self, msg):
        self.fix2 = msg.status.status
        self.egoLatF = msg.latitude
        self.egoLonF = msg.longitude
        if self.old_fix2 != self.fix2:
            if self.fix2 == -1:
                self.gps2Led.setPixmap(self.pixmapRedLed)
                self.fix2Text.setText("NO FIX")
            if self.fix2 == 0:
                self.gps2Led.setPixmap(self.pixmapYellowLed)
                self.fix2Text.setText("FIX")
            if self.fix2 == 2:
                self.gps2Led.setPixmap(self.pixmapGreenLed)
                self.fix2Text.setText("FIX RTK")
        if self.keepCount:
            self.countLogGps2 += 1  # counter for get % fix GPS2
            if self.fix2 == 2:
                self.countLogFix2 += 1
            if self.countLogGps2 != 0:
                self.percentageFix2 = self.countLogFix2/self.countLogGps2 * 100.00
                self.percentage2Text.setText(str(round(self.percentageFix2, 2)) + ' %')
        self.old_fix2 = self.fix2
        self.r2.callback_hz(NavSatFix, topic='gps2/fix')
        self.gps2HzText.setText('GPS2: ' + str(round(self.r2.get_hz(topic='gps2/fix')[0], 2))+' Hz')
        self.old_fix1 = self.fix1
        self.get_heading()

    def get_heading(self):
        self.polar_to_cartesian()
        self.egoHead = -math.atan2(self.egoGpsPoseF[1]-self.egoGpsPoseB[1], self.egoGpsPoseF[0]-self.egoGpsPoseB[0])#  [rad] TODO: CHECK THAT THE SIGN IS CORRECT
        t = QTransform()
        t.rotate(math.degrees(self.egoHead))
        rotated_pixmap = self.pixmapCar.transformed(t)
        self.car.setPixmap(rotated_pixmap.scaled(carSize[0], carSize[1]))

    def polar_to_cartesian(self):
        self.egoGpsPoseF[0] = self.egoLatF*lat_deg_km
        self.egoGpsPoseF[1] = self.egoLonF*lon_deg_km*math.cos(meanLat)
        self.egoGpsPoseB[0] = self.egoLatB*lat_deg_km
        self.egoGpsPoseB[1] = self.egoLonB*lon_deg_km*math.cos(meanLat)

    def start_logging(self):
        self.keepCount = 1
        self.buttonStart.setVisible(0)
        self.buttonStop.setVisible(1)
        # BAG RECORDING
        if self.lineBag.text() != "":
            self.bag_command = "rosbag record -O " + self.bag_dir + "/" + self.lineBag.text() + ".bag /gps1/fix /gps2/fix diagnostics"
            print(self.bag_command)
            self.bag_command_sel = shlex.split(self.bag_command)
            self.bag_proc = subprocess.Popen(self.bag_command_sel)

    def stop_logging(self):
        self.keepCount = 0
        self.countLogGps1 = 0.0  # counter for getting overall number GPS1 sample
        self.countLogGps2 = 0.0  # counter for getting overall number GPS2 sample
        self.countLogFix1 = 0.0  # counter for getting % fix GPS1
        self.countLogFix2 = 0.0  # counter for getting % fix GPS2
        self.buttonStart.setVisible(1)
        self.buttonStop.setVisible(0)
        # STOP BAG RECORDING
        for proc in psutil.process_iter():
            if "record" in proc.name() and set(self.bag_command_sel[2:]).issubset(proc.cmdline()):
                proc.send_signal(subprocess.signal.SIGINT)
        if self.bag_proc != "":
            self.bag_proc.send_signal(subprocess.signal.SIGINT)

    def select_bag_folder(self):
        self.bag_dir = str(QFileDialog.getExistingDirectory(self, "Select Directory"))


if __name__ == '__main__':
    rospy.init_node('dashboard')
    try:
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass
