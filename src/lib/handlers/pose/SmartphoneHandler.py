import socket
import numpy as np
import math

#Recieves Marker information from a UDP messgae which is used to calculate the robot's position and heading. 
class poseHandler:
    def __init__(self, proj, shared_data, port):
        """
        Pose handler for Smartphone Localization

        port (int): The port of VICON system (default=5005)
        """
        #Initialize UDP connection with LTLMoP Localize App
        self.port = port
        self.IP = socket.gethostbyname(socket.gethostname())
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.IP, self.port))

    #Derive transformed coordinates using homography and image coordinates
    def findPoint(self, homography, coord):
        sol = np.dot(homography, coord)
        x_coord = sol[0, 0]/sol[2,0]
        y_coord = sol[1, 0]/sol[2,0]
        return x_coord, y_coord

    def getPose(self, cached=False):
        #Data is recieved in the following form:
        #"X1:Y1_a1x:a1y_X2:Y2_a2x:a2y_X3:Y3_a3x:a3y_X4:Y4_a4x:a4y_r1x:r1y_0:0_r2x_r2y"
        #{x,y}_{1, 2, 3, 4} correspond to the image coordinates at which the field markers are identified
        #r{1,2}_{x,y} correspond to the image coordinates at which the robot markers are identified
        #'a' values are the anchor values that correspond to each image point i.e. x_1:y_1 corresponds to a1x_a2x

        #If all the ID tags have not been defined on the LTLMoPLocalize App, a string "nan_nan_nan" is sent 
        data, addr = self.sock.recvfrom(1024)
        if(data[0:3]!="nan"):
            allPts = data.split('_')

            anchors = np.zeros((4, 2))
            points = np.zeros((4, 2))
            for k in range(0, 8): #k=0, 1, 2, 3
                temp = allPts[k].split(":")
                if ((k%2)==0):
                    points[k/2, 0] = float(temp[0])
                    points[k/2, 1] = float(temp[1])
                else:
                    anchors[k/2, 0] = float(temp[0])
                    anchors[k/2, 1] = float(temp[1])

            temp = allPts[8].split(":")
            r1 = np.ones((3, 1))
            r1[0, 0] = float(temp[0])
            r1[1, 0] = float(temp[1])

            temp = allPts[10].split(":")
            r2 = np.ones((3, 1))
            r2[0, 0] = float(temp[0])
            r2[1, 0] = float(temp[1])

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
            
            # if (first==0):
            #   for i in range (0, 4):
            #       print "Anchor " + str(i) + ": (" + str(anchors[i,0]) + ", " + str(anchors[i,1]) +")"
            #   for i in range (0, 4):
            #       print "Point " + str(i) + ": (" + str(points[i,0]) + ", " + str(points[i,1]) +")"
            #   first=1
            #   print "Robot 1: (" + str(r1[0,0]) + ", " + str(r1[1,0]) +")"
            #   print "Robot 2: (" + str(r2[0,0]) + ", " + str(r2[1,0]) +")"
        
            [r1x, r1y] = self.findPoint(homography, r1) #Transform coordinates of first robot tag to user-defined coordinates
            [r2x, r2y] = self.findPoint(homography, r2) #Transform coordinates of second robot tag to user-defined coordinates

            #Using transformed coordinates of both robot tags, calculate robot's center position and angle
            theta = math.atan2((r1y-r2y),(r1x-r2x))
            rx_m = (r1x+r2x)/2
            ry_m = (r1y+r2y)/2
        else:
            ry_m = 0
            rx_m = 0
            theta = float('nan')
        return np.array([rx_m, ry_m, theta])



