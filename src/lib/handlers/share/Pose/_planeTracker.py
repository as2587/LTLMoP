#!/usr/bin/env python

'''
Multitarget planar tracking
==================

Example of using features2d framework for interactive video homography matching.
ORB features and FLANN matcher are used. This sample provides PlaneTracker class
and an example of its usage.

video: http://www.youtube.com/watch?v=pzVbhxx6aog

Usage
-----
plane_tracker.py [<video source>]

Keys:
   SPACE  -  pause video
   c      -  clear targets

Select a textured planar object to track by drawing a box with a mouse.
'''

import numpy as np
import cv2

# built-in modules
from collections import namedtuple

# local modules
import _video
import _common

import logging

FLANN_INDEX_KDTREE = 1
FLANN_INDEX_LSH    = 6
flann_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2

MIN_MATCH_COUNT = 10

'''
  image     - image to track
  rect      - tracked rectangle (x1, y1, x2, y2)
  keypoints - keypoints detected inside rect
  descrs    - their descriptors
  data      - some user-provided data
'''
PlanarTarget = namedtuple('PlaneTarget', 'image, rect, keypoints, descrs, data')

'''
  target - reference to PlanarTarget
  p0     - matched points coords in target image
  p1     - matched points coords in input frame
  H      - homography matrix from p0 to p1
  quad   - target bounary quad in input frame
'''
TrackedTarget = namedtuple('TrackedTarget', 'target, p0, p1, H, quad')

class PlaneTracker:
    def __init__(self):
        self.detector = cv2.ORB( nfeatures = 1000 )
        self.matcher = cv2.FlannBasedMatcher(flann_params, {})  # bug : need to pass empty dict (#1329)
        self.targets = []

    def add_target(self, image, rect, data=None):
        self.clear()
        '''Add a new tracking target.'''
        x0, y0, x1, y1 = rect
        raw_points, raw_descrs = self.detect_features(image)
        points, descs = [], []
        for kp, desc in zip(raw_points, raw_descrs):
            x, y = kp.pt
            if x0 <= x <= x1 and y0 <= y <= y1:
                points.append(kp)
                descs.append(desc)
        descs = np.uint8(descs)
        self.matcher.add([descs])
        target = PlanarTarget(image = image, rect=rect, keypoints = points, descrs=descs, data=None)
        self.targets.append(target)

    def clear(self):
        '''Remove all targets'''
        self.targets = []
        self.matcher.clear()

    def track(self, frame):
        '''Returns a list of detected TrackedTarget objects'''
        self.frame_points, self.frame_descrs = self.detect_features(frame)
        if len(self.frame_descrs) < MIN_MATCH_COUNT:
            return []
        matches = self.matcher.knnMatch(self.frame_descrs, k = 2)
        matches = [m[0] for m in matches if len(m) == 2 and m[0].distance < m[1].distance * 0.75]
        if len(matches) < MIN_MATCH_COUNT:
            return []
        matches_by_id = [[] for _ in xrange(len(self.targets))]
        for m in matches:
            matches_by_id[m.imgIdx].append(m)
        tracked = []
        for imgIdx, matches in enumerate(matches_by_id):
            if len(matches) < MIN_MATCH_COUNT:
                continue
            target = self.targets[imgIdx]
            p0 = [target.keypoints[m.trainIdx].pt for m in matches]
            p1 = [self.frame_points[m.queryIdx].pt for m in matches]
            p0, p1 = np.float32((p0, p1))
            H, status = cv2.findHomography(p0, p1, cv2.RANSAC, 3.0)
            status = status.ravel() != 0
            if status.sum() < MIN_MATCH_COUNT:
                continue
            p0, p1 = p0[status], p1[status]

            x0, y0, x1, y1 = target.rect
            quad = np.float32([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
            quad = cv2.perspectiveTransform(quad.reshape(1, -1, 2), H).reshape(-1, 2)

            track = TrackedTarget(target=target, p0=p0, p1=p1, H=H, quad=quad)
            tracked.append(track)
        tracked.sort(key = lambda t: len(t.p0), reverse=True)
        return tracked

    def detect_features(self, frame):
        '''detect_features(self, frame) -> keypoints, descrs'''
        keypoints, descrs = self.detector.detectAndCompute(frame, None)
        if descrs is None:  # detectAndCompute returns descs=None if not keypoints found
            descrs = []
        return keypoints, descrs


class TrackerApp:
    def __init__(self):
        print "Tracker initalized"
        self.cap = _video.cv2.VideoCapture(0)
        self.frame = None
        self.paused = False
        self.tracker = PlaneTracker()
        ret, self.frame = self.cap.read()
        
        old_w = self.cap.get(3)#w
        old_h = self.cap.get(4)#h
        
        new_w = 700.0
        new_h=int(new_w*(old_h/old_w))
        self.cap.set(3, new_w)
        self.cap.set(4, new_h)
        
        cv2.namedWindow('plane')
        self.rect_sel = _common.RectSelector('plane', self.on_rect)

    def on_rect(self, rect):
        self.tracker.add_target(self.frame, rect)
    
    def get_robotPoints(self, homRect):
        r_pts = []
        r_pts.append(((int(homRect[0][0]+homRect[1][0])/2), int((homRect[0][1]+homRect[1][1])/2)))
        r_pts.append(((int(homRect[2][0]+homRect[3][0])/2), int((homRect[2][1]+homRect[3][1])/2)))
        return r_pts
        
    def resizedFrame(self,frameIn):
        r = 700.0/frameIn.shape[1]
        dim = (700,int(frameIn.shape[0]*r))
        frameOut = cv2.resize(frameIn, dim, interpolation = cv2.INTER_AREA)
        return frameOut
        
    def run(self):
        iteration_complete = False
        while not iteration_complete:
            if self.frame is not None:
                playing = not self.paused and not self.rect_sel.dragging

                vis = self.frame.copy()
                if playing:
                    tracked = self.tracker.track(self.frame)
                    if (len(tracked))>0:
                        tr = tracked[len(tracked)-1]
                        r_pts = self.get_robotPoints(tr.quad)
                        iteration_complete = True
                        for pt in r_pts:
                            cv2.circle(vis, pt, 5, (0, 255, 255))
                        cv2.polylines(vis, [np.int32(tr.quad)], True, (255, 255, 255), 2)
                        for (x, y) in np.int32(tr.p1):
                            cv2.circle(vis, (x, y), 2, (255, 255, 255))

                self.rect_sel.draw(vis)
                cv2.imshow('plane', vis)
            ret, self.frame = self.cap.read()
            ch = cv2.waitKey(1)
            if ch == ord(' '):
                self.paused = not self.paused
            if ch == ord('c'):
                self.tracker.clear()
            if ch == 27:
                break
        p1 = np.ones((3, 1))
        p2 = np.ones((3, 1))
        p1[0,0] = r_pts[0][0]
        p1[1,0] = r_pts[0][1]
        p2[0,0] = r_pts[1][0]
        p2[1,0] = r_pts[1][1]
        return [p1,p2]

if __name__ == '__main__':
    print __doc__
    myTrack = TrackerApp()
    myTrack.run()
    while True:
        myTrack.run()
