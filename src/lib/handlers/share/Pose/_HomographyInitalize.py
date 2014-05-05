import sys
import numpy as np
import cv2

print "****************************"
print "HOMOGRAPHY INITALIZE"
print "Use this tool to prepare your webcam for detecting robot pose. "
print "****************************"

global allImagePt, allCoordPt
allImagePt=[]
allCoordPt = []

def printAllPoints():
    print ""
    allString = ""
    for l in range(0, len(allImagePt)):
        info = str(allImagePt[l]) + " --> " + str(allCoordPt[l])
        print info
        allString = allString+info + "\n"
    print ""
    return allString
    
def mouseClicked(event,x,y,flags,param):
    global allImagePt, allCoordPt
    if event == 1:
        print "Clicked image at " + "(" + str(x)+","+str(y)+")"
        print "What coordinate should this point represent? Enter \'x,y\'"
        query = raw_input()
        query = query.split(",")
        pt = (int(query[0]), int(query[1]))
        allImagePt.append((x,y))
        allCoordPt.append(pt)
        printAllPoints()
        
def writeTxtFile():
    file = open("HomographyCalib.txt", "w")
    file.write(printAllPoints())
    file.close

def resizedFrame(frameIn):
    r = 700.0/frameIn.shape[1]
    dim = (700,int(frameIn.shape[0]*r))
    frameOut = cv2.resize(frameIn, dim, interpolation = cv2.INTER_AREA)
    return frameOut 
    
def mainloop():
    win =cv2.namedWindow("Video Stream")
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    # cap.set(3, 600)
    # cap.set(4, 500)
    cv2.setMouseCallback("Video Stream", mouseClicked)
    
        
    while(len(allImagePt)<4):
        if frame is not None:
            frame = resizedFrame(frame)
            for point in allImagePt:
                cv2.circle(frame,point, 5, (0,0,255),thickness=-1)
            # Our operations on the frame come here            

            # Display the resulting frame
            cv2.imshow("Video Stream",frame)


        ret,frame = cap.read()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    writeTxtFile()

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    
if __name__=="__main__":
    mainloop()
    

