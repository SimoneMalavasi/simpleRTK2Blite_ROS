#!/usr/bin/env python
# coding: utf8

import rospy
import rostopic
import wx
from sensor_msgs.msg import NavSatFix
import math
import numpy as np

red = '#ff0000'
yellow = '#ffff00'
green = '#00ff00'
lat_deg_km = 110.574     # [km/deg]
lon_deg_km = 111.320    # [km/deg]
meanLat = math.radians(45.454847)   # TODO: CHANGE THIS VALUE WITH THE RIGHT ONE

class Dashboard(wx.Frame):

    def __init__(self):
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
        self.gps1Hz = 0.0
        self.gps2Hz = 0.0
        wx.Frame.__init__(self, parent=None, title='Dashboard')
        panel = wx.Panel(self)
        self.carImage = '/home/simone/Scaricati/car.png'
        self.png = wx.Image(self.carImage, wx.BITMAP_TYPE_ANY)
        self.png = self.png.Scale(self.png.GetWidth()/20, self.png.GetHeight()/20, wx.IMAGE_QUALITY_HIGH)
        self.png_centre = wx.Point(self.png.GetWidth()/2, self.png.GetHeight()/2)
        self.image = wx.StaticBitmap(panel, -1, self.png.ConvertToBitmap(), (250, 30), (self.png.GetWidth(), self.png.GetHeight()))
        self.led1 = wx.StaticBox(panel, wx.ID_ANY, "", size=(70, 70), pos=(10, 30))
        self.led2 = wx.StaticBox(panel, wx.ID_ANY, "", size=(70, 70), pos=(101, 30))
        #self.textHz1 = wx.StaticText(panel, wx.ID_ANY, "", size=(70, 70), pos=(20, 60))
        self.textgps1 = wx.StaticText(panel, wx.ID_ANY, "GPS 1", size=(70, 70), pos=(20, 100))
        #self.textHz2 = wx.StaticText(panel, wx.ID_ANY, "", size=(70, 70), pos=(110, 60))
        self.textgps2 = wx.StaticText(panel, wx.ID_ANY, "GPS 2", size=(70, 70), pos=(110, 100))
        self.subscribe_data()
        self.Show()

    def subscribe_data(self):
        self.subGPS1Data = rospy.Subscriber("/gps1/fix", NavSatFix, self.get_gps1_data, queue_size=1)
        self.subGPS2Data = rospy.Subscriber("/gps2/fix", NavSatFix, self.get_gps2_data, queue_size=1)

    def get_gps1_data(self, msg):
        self.fix1 = msg.status.status
        self.egoLatB = msg.latitude
        self.egoLonB = msg.longitude
        if self.fix1 == -1:
            self.led1.SetBackgroundColour(red)
        if self.fix1 == 0:
            self.led1.SetBackgroundColour(yellow)
        if self.fix1 == 2:
            self.led1.SetBackgroundColour(green)

    def get_gps2_data(self, msg):
        self.fix2 = msg.status.status
        self.egoLatF = msg.latitude
        self.egoLonF = msg.longitude
        if self.fix2 == -1:
            self.led2.SetBackgroundColour(red)
        if self.fix2 == 0:
            self.led2.SetBackgroundColour(yellow)
        if self.fix2 == 2:
            self.led2.SetBackgroundColour(green)
        self.get_heading()

    def get_heading(self):
        self.polar_to_cartesian()
        self.egoHead = -math.atan2(self.egoGpsPoseF[1]-self.egoGpsPoseB[1], self.egoGpsPoseF[0]-self.egoGpsPoseB[0])#  [rad] TODO: CHECK THAT THE SIGN IS CORRECT
        self.temp_png = self.png.Rotate(self.egoHead, self.png_centre)
        self.image.SetBitmap(self.temp_png.ConvertToBitmap())

    def polar_to_cartesian(self):
        self.egoGpsPoseF[0] = self.egoLatF*lat_deg_km
        self.egoGpsPoseF[1] = self.egoLonF*lon_deg_km*math.cos(meanLat)
        self.egoGpsPoseB[0] = self.egoLatB*lat_deg_km
        self.egoGpsPoseB[1] = self.egoLonB*lon_deg_km*math.cos(meanLat)


if __name__ == '__main__':
    rospy.init_node('dashboard')
    try:
        app = wx.App()
        frame = Dashboard()
        app.MainLoop()
    except rospy.ROSInterruptException:
        pass
