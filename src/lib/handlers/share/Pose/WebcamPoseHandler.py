#!/usr/bin/env python
"""
=========================================================================
WebcamPoseHandler.py - Pose Handler using video stream from static webcam
=========================================================================
"""

import sys, time
from numpy import *
from lib.regions import *
import cv2

import logging

import lib.handlers.handlerTemplates as handlerTemplates

class WebcamPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data, initial_region):
        """
        Webcam pose handler - generate robot pose from video stream
        
        initial_region (region): Starting position for robot
        

        """
        self.window = cv2.namedWindow("Video Stream")
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3,400)
        self.cap.set(4,300)
        ret, self.frame = self.cap.read()
        
        
        r = executor.proj.rfiold.indexOfRegionWithName(initial_region)
        center = executor.proj.rfiold.regions[r].getCenter()
        self.x = center[0]
        self.y = center[1]
        self.theta = 0
        

    def getPose(self, cached=False):
        if self.frame is not None:
            cv2.imshow("Video Stream",self.frame)
            ret,self.frame = cap.read()
            
        x=0
        y=0
        o=0

        return array([x, y, o])


