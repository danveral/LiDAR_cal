#!/usr/bin/env python
# -*- coding:utf-8 -*-

# This script is just for VLP-16 Puck (0x22)
# the sensor should be in Strongest Return mode.  (0x37)
# GPS synchronization is advised (0x02)
# the pcap should be recorded while the sensor has steady spinning

# 600 RPM is needed.
# DO NOT use pcapng, only pcap.

import time, sys, os
import dpkt
import numpy as np
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D

udpDestPort = 2368

VerticalAngle16Lasers = np.array([-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0])
VerticalAnglefor32lasers = np.hstack((VerticalAngle16Lasers, VerticalAngle16Lasers))

cosVerticalAngle = np.cos(VerticalAnglefor32lasers*np.pi/180.0)


# this processRawData function process each package and return
# np.array([
# [ts0,  ts1,  ts2,  ts3,  ... , ts384 ],
# [azi0, azi1, azi2, azi3, ... , azi384],
# [x0,   x1,   x2,   x3,   ... , x384  ],
# [y0,   y1,   y2,   y3,   ... , y384  ],
# [z0,   z1,   z2,   z3,   ... , z384  ],
# [int0, int1, int2, int3, ... , int384]])
def processRawData(udpData):
    tmpArray = np.array([ord(i) for i in udpData[:1200]]).reshape((12,100))
    
    return 0

# startFrame should be equal or greater than 1
# I didn't put parameters check, just use reasonable value.
def parsePcapGetUdp(filename, startFrame, frameNumber):
    pcapHandler = dpkt.pcap.Reader(open(filename,'rb'))
    try:
        tmpAzimuth = 0   # I am going to check the azimuth to decide which frame I need
        k = 0
        rawDataperFrame = np.array([[],[],[],[],[],[]])
        tmpFrames = []
        m = 0

        for ts, pkt in pcapHandler:
            ethFrame = dpkt.ethernet.Ethernet(pkt)
            if ethFrame.type != 2048:
                continue
            ipPkt = ethFrame.data
            if ipPkt.data.dport == udpDestPort:
                udpData = ipPkt.data.data
                azimuthForFirstBlock = ord(udpData[3])*256 + ord(udpData[2])
                if azimuthForFirstBlock < tmpAzimuth:
                    k = k + 1
                tmpAzimuth = azimuthForFirstBlock
                if k <= startFrame:   # the first frame would lost the first point
                    continue
                rawDataPerPkg = processRawData(ipPkt.data.data)
                frameStartCursor = np.argsort(rawDataPerPkg[1])[0]  # Azimuth changes from 0 - 360 degree
                rawDataperpkg = rawDataPerPkg[..., frameStartCursor:]
                rawDataperFrame = np.column_stack((rawDataperpkg, rawDataperFrame))

                if frameStartCursor != 0:
                    m = m + 1
                    if m >= frameNumber:
                        break
                    if m >= 1:
                        tmpFrames.append(rawDataperFrame)

        frames = np.array(tmpFrames)
    except:
        pass

    return 0

parsePcapGetUdp(sys.argv[1],30,2)
