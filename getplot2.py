#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Author: Guang Ling
# Any Question, shoot email to: 15654181@qq.com

# This script is just for VLP-16 Puck (0x22)
# the sensor should be in Strongest Return mode.  (0x37)
# GPS synchronization is advised (0x02)
# the pcap should be recorded while the sensor has be steady spinning

# 600 RPM is needed.
# DO NOT use pcapng, only pcap.

import time, sys, os
import dpkt
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

udpDestPort = 2368

VerticalAngle16Lasers = np.array([-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0])
VerticalAnglefor32lasers = np.hstack((VerticalAngle16Lasers, VerticalAngle16Lasers))
cosVerticalAngle = np.cos(VerticalAnglefor32lasers*np.pi/180.0)
sinVerticalAngle = np.sin(VerticalAnglefor32lasers*np.pi/180.0)

timeListForEachFiring_block = np.hstack((np.arange(16)*2.304, np.arange(16)*2.304+55.296))
timeListForEachFiring_pkg = (np.arange(12) * 110.592).reshape((12,1)) + timeListForEachFiring_block

# This processRawData function process each package for pcap and return
# np.array([
# [ts0,  ts1,  ts2,  ts3,  ... , ts384 ],
# [azi0, azi1, azi2, azi3, ... , azi384],
# [x0,   x1,   x2,   x3,   ... , x384  ],
# [y0,   y1,   y2,   y3,   ... , y384  ],
# [z0,   z1,   z2,   z3,   ... , z384  ],
# [int0, int1, int2, int3, ... , int384]])
def processRawData(udpData):
    # get each point's timestamp
    timeForEachPkg = 256**3*ord(udpData[1203]) + 256**2*ord(udpData[1202]) + 256*ord(udpData[1201]) + ord(udpData[1200])
    pointTimeStamp = timeListForEachFiring_pkg.flatten() + timeForEachPkg

    # get each point's azimuth
    tmpArray = np.array([ord(i) for i in udpData[:1200]]).reshape((12, 100))[..., 2:]
    tmpArray = tmpArray*1.0
    azimuthForEachBlock = (tmpArray[:,1]*256 + tmpArray[:,0])/100.0
    t = azimuthForEachBlock[1:] - azimuthForEachBlock[:-1]
    first_11_Azimuth_Gap_ForEachBlock = np.where(t<0, t+360, t) # gap may less than 0
    azimuth_Gap_ForEachBlock = np.append(first_11_Azimuth_Gap_ForEachBlock,first_11_Azimuth_Gap_ForEachBlock[-1])
    pointAzimuth_raw = (azimuth_Gap_ForEachBlock.reshape((12,1))*timeListForEachFiring_block/110.592 + azimuthForEachBlock.reshape((12,1))).flatten()
    pointAzimuth = np.where(pointAzimuth_raw >= 360, pointAzimuth_raw-360, pointAzimuth_raw)

    # get XYZ
    distArray = 0.002*(tmpArray[..., 2:][:,1::3]*256 + tmpArray[..., 2:][:,::3])
    pointX = (distArray * cosVerticalAngle).flatten() * np.sin(pointAzimuth)
    pointY = (distArray * cosVerticalAngle).flatten() * np.cos(pointAzimuth)
    pointZ = (distArray * sinVerticalAngle).flatten()

    # get intensity, change to float64
    pointIntensity = (tmpArray[..., 2:][:,2::3].flatten())*1.0

    # get the final array of everything
    ret = np.vstack((pointTimeStamp, pointAzimuth, pointX, pointY, pointZ, pointIntensity))

    return ret

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
                        rawDataperFrame = np.array([[], [], [], [], [], []])
    except:
        pass

    return tmpFrames

def getPlotXYZ(filename, startFrame, frameNumber, getplotfromFrame):
    points = parsePcapGetUdp(filename, startFrame, frameNumber)
    pointsFrame = points[getplotfromFrame]
    x = pointsFrame[2]
    y = pointsFrame[3]
    z = pointsFrame[4]

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, z, marker='.', s=4, color='r')

    ax.set_zlabel('Z')
    ax.set_ylabel('Y')
    ax.set_xlabel('X')

    plt.show()

getPlotXYZ(sys.argv[1],30,4,2)


