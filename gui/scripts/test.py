#!/usr/bin/env python
#----------------------------------------------------------------------------
# Name:         test1.py
# Purpose:      A minimal wxPython program
#
# Author:       Robin Dunn
#
# Created:
# Licence:      wxWindows license
#----------------------------------------------------------------------------

import wx
import roslib
import rospy
import serial

class MyFrame(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title)
        self.Bind(wx.EVT_MOVE, self.OnMove)
        self.Bind(wx.EVT_SIZE, self.OnSize)

    def OnSize(self, event):
        size = event.GetSize()
        print "size:", size.width, size.height

    def OnMove(self, event):
        pos = event.GetPosition()
        print "pos:", pos.x, pos.y



class MyApp(wx.App):
    def OnInit(self):
        frame = MyFrame(None, -1, "This is a test")
        frame.Show(True)
        self.SetTopWindow(frame)
        return True


def main():
    rospy.init_node('gui')
    ser = serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A8008FpQ-if00-port0', 9600)
    x = ser.read() #read a byte
    print "read ", x
    ser.close()
    app = MyApp(0)
    app.MainLoop()

if __name__ == "__main__":
    main()
