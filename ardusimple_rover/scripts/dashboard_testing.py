#!/usr/bin/env python
# coding: utf8

import rospy
import rostopic
from PySide2.QtCore import QSize, Qt
from PySide2.QtWidgets import QApplication, QWidget, QMainWindow, QLabel
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
ledSize = (50, 50)
carSize = (40, 70)
gps1HzPosition = (100, 300)

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
        self.r1.callback_hz(NavSatFix, topic='gps1/fix')
        print(self.r1.get_hz(topic='gps1/fix')[0])
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
        self.old_fix2 = self.fix2
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

if __name__ == '__main__':
    rospy.init_node('dashboard')
    try:
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass
