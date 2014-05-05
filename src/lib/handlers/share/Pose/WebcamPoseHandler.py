#!/usr/bin/env python
"""
=========================================================================
WebcamPoseHandler.py - Pose Handler using video stream from static webcam
=========================================================================
"""

import sys, time
import numpy as np
from lib.regions import *
import cv2
import traceback

from _planeTracker import TrackerApp
import logging

import lib.handlers.handlerTemplates as handlerTemplates

class WebcamPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data):
        """
        Webcam pose handler - generate robot pose from video stream        

        """
        self.findHomography()
        self.trackerApp = TrackerApp()
        try:
            self.trackerApp.run()
        except:
            traceback.print_exc()
            raise
        
    
    def findHomography(self):
        #Helper function to parse .txt file
        def getCoords(coords):
            coords = coords.split(",")
            x = coords[0].replace("(","")
            y = coords[1].replace(")","")
            return x, y
            
        calib = open('lib/handlers/share/Pose/HomographyCalib.txt', 'r')
        anchors = np.zeros((4, 2))
        points = np.zeros((4, 2))
        idx=0
        for line in calib:
            line= line.split(' --> ') 
            points[idx,0], points[idx,1] = getCoords(line[0])
            anchors[idx,0], anchors[idx,1] = getCoords(line[1])
            idx = idx+1

        #create matrix used to calculate homography
        a= np.zeros((8, 8))
        for i in range(0, 4):
            a[2*i, :] = np.array([[points[i, 0], points[i,1], 1, 0, 0, 0, -points[i,0]*anchors[i,0], -points[i, 1]*anchors[i, 0]]])
            a[i*2+1, :] = np.array([[i, 0, 0, points[i, 0], points[i,1], 1, -points[i,0]*anchors[i,1], -points[i, 1]*anchors[i, 1]]])
        
        b = np.zeros((8, 1))
        for i in range(0, 8):
            b[i, 0] = anchors[i/2,i%2]
    
        hom = np.linalg.solve(a, b) #Derive Homography Matrix
        hom = np.vstack([hom, 1])
        homography = np.zeros((3, 3))
        for i in range(0,3):
            homography[i, :] = hom[(3*i):(3*i+3), 0]
        self.homography= homography
        
    def findPoint(self, coord):
        sol = np.dot(self.homography, coord)
        x_coord = sol[0, 0]/sol[2,0]
        y_coord = sol[1, 0]/sol[2,0]
        return x_coord, y_coord
    
        

    def getPose(self, cached=False):
        robotImgPoints = self.trackerApp.run()
        [r1x, r1y] = self.findPoint(robotImgPoints[0])
        [r2x, r2y] = self.findPoint(robotImgPoints[1])
        
        theta = math.atan2((r1y-r2y),(r1x-r2x))
        rx_m = (r1x+r2x)/2
        ry_m = (r1y+r2y)/2
        logging.info("Current Pose: (%d, %d, %d)", rx_m, rx_m, theta)
        return np.array([rx_m, ry_m, theta])


