#!/usr/bin/env python
# -*- coding:utf-8 -*-

# This script is just for VLP-16 Puck (0x22)
# the sensor should be in Strongest Return mode.  (0x37)
# GPS synchronization is advised (0x02)
# the pcap should be recorded while the sensor has steady spinning

# 600 RPM is needed.

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

def processRawData(udpData):
    print "ok"
    return 0

def parsePcapGetUdp(filename, startFrame, frameNumber):
    pcapHandler = dpkt.pcap.Reader(open(filename,'rb'))
    try:
        tmpAzimuth = 0   # I am going to check the azimuth to decide which frame I need
        k = 0
        startFlag = 0   # start record points
        rawDataperFrame = np.array([[],[],[],[],[],[]])
        m = 0
        tmp_m = 1

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
                if k <= startFrame:
                    continue
                tmpRawPerPkg = processRawData(ipPkt.data.data)
                frameStartIndex = np.argsort(tmpRawPerPkg[1])[0]
                if frameStartIndex != 0:
                    startFlag = 1
                    m = m + 1
                if startFlag == 1 and m == tmp_m:
                    rawDataperpkg = tmpRawPerPkg[..., frameStartIndex:]
                    rawDataperFrame = np.column_stack((rawDataperpkg, rawDataperFrame))
                if k > startFrame + frameNumber:
                    break
    except:
        pass

    return 0

parsePcapGetUdp(sys.argv[1],30,2)
