#!/usr/bin/env python
# coding: utf8

import rospy
import rostopic, roslaunch
import subprocess, shlex, psutil
from PySide2.QtCore import QSize, Qt,Slot
from PySide2.QtWidgets import QApplication, QWidget, QMainWindow, QLabel, QPushButton, QFileDialog,  QLineEdit
from PySide2.QtGui import QPixmap, QTransform, QFont
import sys
import os
import timeit
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
buttonPosition = (390, 200)
launchPosition = (280, 200)
browsePosition = (340, 250)
lineBagPosition = (130, 250)
ledSize = (50, 50)
carSize = (40, 70)
gps1HzPosition = (100, 300)
buttonSize = (110, 30)
lineBagSize = (200, 30)
lineUserPosition = (130, 50)
lineUserSize = lineBagSize
nmeaLink = "<a href=\"https://www.nmeagen.org/\">'NMEA GENERATOR'</a>"
package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
launchFIX_path = os.path.dirname(package_path)+"/ntrip_ros/launch/ntrip_ros.launch"
launchGPS_path = package_path + "/launch/ardusimple_rover_pair.launch"

class StartWindow(QMainWindow):

    yamlFile = open(os.path.dirname(package_path) + "/ntrip_ros/config/ntrip_ros.yaml", "r")
    yamlLines = yamlFile.readlines()

    def __init__(self):
        super(StartWindow, self).__init__()
        self.setWindowTitle("Ntrip Params")
        self.w = None
        self.setFixedSize(QSize(500, 400))
        self.labelNmeaLink = QLabel(self)
        self.labelNmeaLink.setText(nmeaLink)
        self.labelNmeaLink.setOpenExternalLinks(True)
        self.labelNmeaLink.move(lineBagPosition[0]-80, lineBagPosition[1])
        self.labelNmeaLink.setFixedSize(QSize(lineBagSize[0],lineBagSize[1]))
        self.titleText = QLabel(self)
        self.titleText.setText("NTRIP Params")
        self.titleText.move(lineUserPosition[0] + 20, lineUserPosition[1]-40)
        self.titleText.setFixedSize(QSize(lineUserSize[0], lineUserSize[1]))
        self.titleText.setFont(QFont('Times', 20))
        self.serverText = QLabel(self)
        self.serverText.setText("Server: ")
        self.serverText.move(lineUserPosition[0] - 50, lineUserPosition[1])
        self.lineServer = QLineEdit(self)
        self.lineServer.move(lineUserPosition[0], lineUserPosition[1])
        self.lineServer.setFixedSize(QSize(lineUserSize[0], lineUserSize[1]))
        self.lineServer.setText(self.yamlLines[1].replace("ntrip_server: ", ""))
        self.userText = QLabel(self)
        self.userText.setText("User: ")
        self.userText.move(lineUserPosition[0] - 50, lineUserPosition[1]+30)
        self.lineUser = QLineEdit(self)
        self.lineUser.move(lineUserPosition[0], lineUserPosition[1]+30)
        self.lineUser.setFixedSize(QSize(lineUserSize[0], lineUserSize[1]))
        self.lineUser.setText(self.yamlLines[2].replace("ntrip_user: ", ""))
        self.passText = QLabel(self)
        self.passText.setText("Pass: ")
        self.passText.move(lineUserPosition[0] - 50, lineUserPosition[1]+60)
        self.linePass = QLineEdit(self)
        self.linePass.move(lineUserPosition[0], lineUserPosition[1]+60)
        self.linePass.setFixedSize(QSize(lineUserSize[0], lineUserSize[1]))
        self.linePass.setText(self.yamlLines[3].replace("ntrip_pass: ", ""))
        self.mountText = QLabel(self)
        self.mountText.setText("Mountpoint: ")
        self.mountText.move(lineUserPosition[0] - 90, lineUserPosition[1]+90)
        self.lineMount = QLineEdit(self)
        self.lineMount.move(lineUserPosition[0], lineUserPosition[1]+90)
        self.lineMount.setFixedSize(QSize(lineUserSize[0], lineUserSize[1]))
        self.lineMount.setText(self.yamlLines[4].replace("ntrip_stream: ", ""))
        self.nmeaText = QLabel(self)
        self.nmeaText.setText("Nmea GGA: ")
        self.nmeaText.move(lineUserPosition[0] - 90, lineUserPosition[1]+120)
        self.lineNmea = QLineEdit(self)
        self.lineNmea.move(lineUserPosition[0], lineUserPosition[1]+120)
        self.lineNmea.setFixedSize(QSize(lineUserSize[0], lineUserSize[1]))
        self.lineNmea.setText(self.yamlLines[5].replace("nmea_gga: ", ""))
        self.changeButton = QPushButton("Change", self)
        self.changeButton.move(launchPosition[0] - 60, lineBagPosition[1])
        self.changeButton.setFixedSize(QSize(buttonSize[0], buttonSize[1]))
        self.changeButton.clicked.connect(self.on_changeButton_clicked)
        self.noChangeButton = QPushButton("Don't Change", self)
        self.noChangeButton.move(launchPosition[0]+60, lineBagPosition[1])
        self.noChangeButton.setFixedSize(QSize(buttonSize[0], buttonSize[1]))
        self.noChangeButton.clicked.connect(self.on_noChangeButton_clicked)

    def on_changeButton_clicked(self):
        if self.isVisible():
            self.hide()
        if self.w is None:
            self.change_yaml()
            self.w = MainWindow()
            self.w.show()

    def on_noChangeButton_clicked(self):
        if self.isVisible():
            self.hide()
        if self.w is None:
            self.w = MainWindow()
            self.w.show()

    def change_yaml(self):
        self.yamlLines[0] = "rtcm_topic: /rtcm \n"
        self.yamlLines[1] = "ntrip_server: " + self.lineServer.text()
        self.yamlLines[2] = "ntrip_user: " + self.lineUser.text()
        self.yamlLines[3] = "ntrip_pass: " + self.linePass.text() +"\n"
        self.yamlLines[4] = "ntrip_stream: " + self.lineMount.text()
        self.yamlLines[5] = "nmea_gga: " + self.lineNmea.text()
        self.yamlFile = open(os.path.dirname(package_path) + "/ntrip_ros/config/ntrip_ros.yaml", "w")
        self.yamlFile.writelines(self.yamlLines)
        self.yamlFile.close()


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
        self.buttonBrowse.move(browsePosition[0], browsePosition[1])
        self.buttonBrowse.setFixedSize(QSize(buttonSize[0]+50, buttonSize[1]))
        self.buttonLaunch = QPushButton("Connect GPS", self)
        self.buttonLaunch.move(launchPosition[0], launchPosition[1])
        self.buttonLaunch.setFixedSize(QSize(buttonSize[0], buttonSize[1]))
        self.buttonStopLaunch = QPushButton("Disconnect GPS", self)
        self.buttonStopLaunch.move(launchPosition[0], launchPosition[1])
        self.buttonStopLaunch.setFixedSize(QSize(buttonSize[0], buttonSize[1]))
        self.buttonStopLaunch.setVisible(0)
        self.labelBag = QLabel(self)
        self.labelBag.setText("Bag Name: ")
        self.labelBag.move(lineBagPosition[0]-80, lineBagPosition[1])
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
        self.logTime = 0.0
        self.logStartTime = 0.0
        self.logEndTime = 0.0
        self.bag_dir = ""
        self.bag_command = ""
        self.bag_command_sel = ""
        self.bag_proc = ""  # REQUIRED FOR SAVING BAG
        self.buttonStart.clicked.connect(self.start_logging)
        self.buttonStop.clicked.connect(self.stop_logging)
        self.buttonBrowse.clicked.connect(self.select_bag_folder)
        self.buttonLaunch.clicked.connect(self.launch_gps_nodes)
        self.buttonStopLaunch.clicked.connect(self.stop_gps_nodes)
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
        self.polar_to_cartesian()
        egoLonF = math.radians(self.egoLonF)
        egoLonB = math.radians(self.egoLonB)
        egoLatF = math.radians(self.egoLatF)
        egoLatB = math.radians(self.egoLatB)
        egoDeltaLon = egoLonF-egoLonB
        X = math.cos(egoLatF)*math.sin(egoDeltaLon)
        Y = math.cos(egoLatB)*math.sin(egoLatF)-math.sin(egoLatB)*math.cos(egoLatF)*math.cos(egoDeltaLon)
        self.egoHead = math.atan2(X, Y)
        #self.egoHead = -math.atan2(self.egoGpsPoseF[1]-self.egoGpsPoseB[1], self.egoGpsPoseF[0]-self.egoGpsPoseB[0])#  [rad] TODO: CHECK THAT THE SIGN IS CORRECT
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
        self.logStartTime = timeit.default_timer()
        # BAG RECORDING
        if self.lineBag.text() != "":
            self.bag_command = "rosbag record -O " + self.bag_dir + "/" + self.lineBag.text() + '.bag record -e "(.*)gps1(.*)"' + ' "(.*)gps2(.*)" diagnostics'
            print(self.bag_command)
            self.bag_command_sel = shlex.split(self.bag_command)
            self.bag_proc = subprocess.Popen(self.bag_command_sel)

    def stop_logging(self):
        self.logEndTime = timeit.default_timer()
        self.logTime = self.logEndTime - self.logStartTime
        txtLines = ['Bag Name: '+ self.lineBag.text(), 'GPS1 FIX: ' + str(self.percentageFix1) + "%", 'GPS2 FIX: ' +
                    str(self.percentageFix2)+ '%', 'TIME: ' + str(self.logTime) + "s"]
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
            with open(self.bag_dir + "/" + self.lineBag.text()+".txt", 'w') as f:
                f.write('\n'.join(txtLines))
            print("Stopped logging " + self.bag_dir + "/" + self.lineBag.text() + ".bag")

    def select_bag_folder(self):
        self.bag_dir = str(QFileDialog.getExistingDirectory(self, "Select Directory"))

    def launch_gps_nodes(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launchGPS = roslaunch.parent.ROSLaunchParent(uuid, [launchGPS_path])
        self.launchFIX = roslaunch.parent.ROSLaunchParent(uuid, [launchFIX_path])
        self.launchGPS.start()
        self.launchFIX.start()
        self.buttonStopLaunch.setVisible(1)

    def stop_gps_nodes(self):
        self.launchGPS.shutdown() #stops the ROSLaunchParent object
        self.launchFIX.shutdown() #stops the ROSLaunchParent object
        self.buttonStopLaunch.setVisible(0)
        self.buttonLaunch.setVisible(1)


if __name__ == '__main__':
    rospy.init_node('dashboard')
    try:
        app = QApplication(sys.argv)
        window = StartWindow()
        window.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass
