#!/usr/bin/env python

# Code taken form sashagaz hand detection on github

from __future__ import print_function

import roslib
roslib.load_manifest('ardrone_autonomy')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

HULL_PROX = 20      #How close hull points can be to eachother and still be considered seperate fingers

FINGER_EXTENDED = -10 #extra buffer added on to distance used to check if a finger is extended or not

CM_FLOOR_SUBTRACT = 30 #distance below center of mass below which we are willing to allow it to ignore any "fingers"

BW_THRESHOLD = 110   # black/white threshold for threshold function

COLOR_AREA = 500   # amount of color it will see before signaling it sees the hand
dotFound = False

class AnalyzeHand:

  def __init__(self):
    self.numFingersPub = rospy.Publisher("numFingers",Int8, queue_size=10)
    self.landPub = rospy.Publisher("ardrone/land",Empty, queue_size=10)
    
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("ardrone/front/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    #still_cv_image = cv2.imread("zero.jpg")
    #cv2.imshow("Raw", cv_image)
    
    cv2.waitKey(3)    
   
    
    #color detection code----------------------------------
    #color order is b, g, r
    colorBlur = cv2.GaussianBlur(cv_image,(5,5),0)
#    cv2.imshow("blur", colorBlur)

    lower = np.array([0,60,110], dtype = "uint8")
    upper = np.array([40,100,200], dtype = "uint8")
    mask = cv2.inRange(cv_image, lower, upper)
#    output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
#    cv2.imshow("Color detect", np.hstack([cv_image, output]))
    _, colorContours, colorHeirarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c_max_area=0
    c_ci = 0
    if len(colorContours):  #if colorContours is not empty
      for i in range(len(colorContours)):
        colorCnt = colorContours[i]
        c_area = cv2.contourArea(colorCnt)
        if c_area > c_max_area:
          c_max_area = c_area
          c_ci = i
    
      cv2.drawContours(cv_image, [colorContours[c_ci]], 0, (200,255,0), 2)
    #end of if statement
    
    dotFound = False  #redundancy just to make sure that a dot isn't found when it's not supposed to be

    if c_max_area > COLOR_AREA:
      print("Found dot")
      dotFound = True
    else:
      print("No dot")
      dotFound = False

    
#---------------------------------------------------------
    
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#   cv2.imshow("Grayscale", gray)

    blur = cv2.GaussianBlur(gray,(5,5),0)
#    cv2.imshow("blur", blur)
    
    
    ret, thresh = cv2.threshold(blur, BW_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

#    cv2.imshow("threshold", thresh)
    
#Skin color code-----------------------------------------------
     
    """
#    cv2.imshow("Raw", cv_image)
    cv2.waitKey(3)    
    blur = cv2.blur(cv_image, (3,3))
#    cv2.imshow("blur", blur)
    
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)

    #Create a binary image with where white will be skin colors and rest 
#    mask1 = cv2.inRange(hsv,np.array([50,10,60]),np.array([140,255,255]))
#    cv2.imshow("skinOnly1", mask1)

    mask2 = cv2.inRange(hsv,np.array([2,20,20]),np.array([100,255,255]))
    cv2.imshow("skinOnly2", mask2)
  
    #Kernel matrices for morphological transformation    
    kernel_square = np.ones((11,11),np.uint8)
    kernel_ellipse= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    
    #Perform morphological transformations to filter out the background noise
    #Dilation increase skin color area
    #Erosion increase skin color area
    dilation = cv2.dilate(mask2, kernel_ellipse,iterations = 1)
    erosion = cv2.erode(dilation,kernel_square,iterations = 1)    
    dilation2 = cv2.dilate(erosion,kernel_ellipse,iterations = 1)       
#    cv2.imshow("dilation2", dilation2)
    filtered = cv2.medianBlur(dilation2,5)
    kernel_ellipse= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(8,8))
    dilation2 = cv2.dilate(filtered,kernel_ellipse,iterations = 1)
    kernel_ellipse= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    dilation3 = cv2.dilate(filtered,kernel_ellipse,iterations = 1)
    median = cv2.medianBlur(dilation2,5)
    ret, thresh2 = cv2.threshold(median,127,255,0)
   
    """
#----------------------------------------------------------
        
    #Find contours of the filtered frame
    _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)   
    
    #Find Max contour area (Assume that hand is in the frame)
    
    max_area=100
    ci=0	
    for i in range(len(contours)):
      cnt = contours[i]
      area = cv2.contourArea(cnt)
      if(area>max_area):
        max_area=area
        ci=i  
            
    #Largest area contour 			  
    cnts = contours[ci]

    #Find convex hull
    hull = cv2.convexHull(cnts)
    
    #display contour and hull
    cv2.drawContours(cv_image, [cnts], 0, (0,255,0), 2)
    cv2.drawContours(cv_image, [hull], 0, (0,255,200), 2)
    
    #Find convex defects
    hull2 = cv2.convexHull(cnts,returnPoints = False)
    defects = cv2.convexityDefects(cnts,hull2)

    #Get defect points and draw them in the original image
    FarDefect = []
    for i in range(defects.shape[0]):
      s,e,f,d = defects[i,0]
      start = tuple(cnts[s][0])
      end = tuple(cnts[e][0])
      far = tuple(cnts[f][0])
      FarDefect.append(far)
      #cv2.line(cv_image,start,end,[0,255,0],1)
      #cv2.circle(cv_image,far,10,[100,255,255],3)
      
    
    
    #Find moments of the largest contour
    moments = cv2.moments(cnts)
    #Central mass of first order moments
    if moments['m00']!=0:
      cx = int(moments['m10']/moments['m00']) # cx = M10/M00
      cy = int(moments['m01']/moments['m00']) # cy = M01/M00
    centerMass=(cx,cy)    
    
    #Draw center mass
    cv2.circle(cv_image,centerMass,7,[100,0,255],2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    #cv2.putText(cv_image,'Center',tuple(centerMass),font,2,(255,255,255),2)     
    

    #Distance from each finger defect(finger webbing) to the center mass
    distanceBetweenDefectsToCenter = []
    for i in range(0,len(FarDefect)):
      x =  np.array(FarDefect[i])
      centerMass = np.array(centerMass)
      distance = np.sqrt(np.power(x[0]-centerMass[0],2)+np.power(x[1]-centerMass[1],2))
      distanceBetweenDefectsToCenter.append(distance)
    
    #Get an average of three shortest distances from finger webbing to center mass
    sortedDefectsDistances = sorted(distanceBetweenDefectsToCenter)
    AverageDefectDistance = np.mean(sortedDefectsDistances[0:2])
    cv2.circle(cv_image,tuple(centerMass),int(AverageDefectDistance),[255,0,0],2)
    #Get fingertip points from contour hull
    #If points are in proximity of 80 pixels, consider as a single point in the group
    finger = []
    for i in range(0,len(hull)-1):
      if (np.absolute(hull[i][0][0] - hull[i+1][0][0]) > HULL_PROX) or ( np.absolute(hull[i][0][1] - hull[i+1][0][1]) > HULL_PROX):  #HULL_PROX defined at top
        if hull[i][0][1] < 500 :  
          finger.append(hull[i][0])

    #The fingertip points are 5 hull points with largest y coordinates  
    finger =  sorted(finger,key=lambda x: x[1])   
    fingers = finger[0:5]
    for i in range(len(fingers)):
      pass
      #draw fingertips on image
      cv2.circle(cv_image,tuple(fingers[i]),3,[255,0,0],-1)
    #Calculate distance of each finger tip to the center mass
    fingerDistance = []
    for i in range(0,len(fingers)):
      distance = np.sqrt(np.power(fingers[i][0]-centerMass[0],2)+np.power(fingers[i][1]-centerMass[1],2))    #changed second expression to centerMass[1] from centerMass[0]
      fingerDistance.append(distance)
    
    #Finger is pointed/raised if the distance of between fingertip to the center mass is larger
    #than the distance of average finger webbing to center mass by FINGER_EXTENDED pixels
    result = 0
    fingerDetectionFloor = centerMass[1] + CM_FLOOR_SUBTRACT #add because y coordinate increases going down
    for i in range(0,len(fingers)):
      if fingerDistance[i] > AverageDefectDistance*2 + FINGER_EXTENDED:
        if fingers[i][1] < fingerDetectionFloor: #if y coordinate is above fingerDetectionFloor. Less than because y coordinate increases going down
          result = result + 1
        
    #draw finger detection radius
    cv2.circle(cv_image,tuple(centerMass),int(AverageDefectDistance*2 + FINGER_EXTENDED),[0,0,255],2)    
    #draw finger detection floor
    cv2.line(cv_image,tuple([0, fingerDetectionFloor]),tuple([1000, fingerDetectionFloor]),[0,0,255],2)

    #Print number of pointed fingers
    cv2.putText(cv_image,str(result),(100,100),font,2,(255,0,0),2)
    #Publish number of fingers
#    print(result)
    if dotFound:
      self.numFingersPub.publish(result)

    cv2.imshow("Contours and Hull", cv_image)
#    self.landPub.publish(Empty())

def main(args):
  rospy.init_node('AnalyzeHand', anonymous=True)
  ah = AnalyzeHand()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
