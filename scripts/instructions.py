#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('ardrone_autonomy')
import sys
import rospy
import cv2
import re
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from drone_controller import BasicDroneController

class instructions:

  def __init__(self):
    self.fingerNumber = 0
    self.pubLand = rospy.Publisher("/ardrone/land",Empty, queue_size=10)	
    self.numFingersSub = rospy.Subscriber("numFingers",Int8,self.callback)
    self.movePub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

  def callback(self,data):
    self.fingerNumber = data.data
#    print(self.fingerNumber)
#   print(data)
    #self.movePub.publish(-linear.x)
    
  def land(self):
#    print("in land")
    self.pubLand.publish(Empty())
    
  def backward(self):
    print("in backward")
    move = Twist()
    move.linear.x = -0.2
    self.movePub.publish(move)

  def forward(self):
    print("in forward")
    move = Twist()
    move.linear.x = 0.2
    self.movePub.publish(move)

  def left(self):
    print("in left")
    move = Twist()
    move.linear.y = 0.2
    self.movePub.publish(move)

  def right(self):
    print("in right")
    move = Twist()
    move.linear.y = -0.2
    self.movePub.publish(move)

  def turnLeft(self):
    print("in turnLeft")
    move = Twist()
    move.angular.z = 0.5
    self.movePub.publish(move)

  def turnRight(self):
    print("in backward")
    move = Twist()
    move.angular.z = -0.5
    self.movePub.publish(move)

  def movements(self):
    print("in movements")
    while(1):
#      print("fingerNumber:")
#      print(self.fingerNumber)
      if (self.fingerNumber == 5):
        print("in if 5")
        i = 0
        while(i < 10000):
          self.backward()
          i += 1
        i = 0
        while(i < 20000):
          self.turnRight()
          i += 1
        i = 0
        while(i < 20000):
          self.land()
          i += 1
        
#      if self.fingerNumber == 2 :
#        i = 0
#        while(i < 20000):
#          self.turnLeft()
#          i += 1
#        i = 0
#        while(i < 8000):
#          self.forward()
#          i += 1
#        i = 0
#        while(i < 20000):
#          self.turnLeft()
#          i += 1
#        i = 0
#        while(i < 8000):
#          self.forward()
#          i += 1
#        i = 0
#        while(i < 20000):
#          self.Land()
#          i += 1

#      if self.fingerNumber == 3 :
#        i = 0
#        while(i < 20000):
#          self.turnLeft()
#          i += 1
#        i = 0
#        while(i < 8000):
#          self.backward()
#          i += 1
#        i = 0
#        while(i < 20000):
#          self.turnLeft()
#          i += 1
#        i = 0
#        while(i < 8000):
#          self.forward()
#          i += 1
#        i = 0
#        while(i < 20000):
#          self.Land()
#          i += 1
        

#      if self.fingerNumber == 4 :
#        i = 0
#        while(i < 15000):
#          self.turnLeft()
#          i += 1
#        i = 0
#        while(i < 8000):
#          self.forward()
#          i += 1
#        i = 0
#        while(i < 40000):
#          self.turnRight()
#          i += 1
#        i = 0
#        while(i < 20000):
#          self.Land()
#          i += 1

#      if self.fingerNumber == 1 :
#        i = 0
#        while(i < 20000):
#          self.turnRight()
#          i += 1
#        i = 0
#        while(i < 4000):
#          self.forward()
#          i += 1
#        i = 0
#        while(i < 20000):
#          self.turnLeft()
#          i += 1
#        i = 0
#        while(i < 4000):
#          self.backward()
#          i += 1
#        i = 0
#        while(i < 10000):
#          self.Land()
#          i += 1
        
      else:
        pass
        
def main(args):
  print("In Main")
  rospy.init_node('instructions', anonymous=True)
  r = instructions()
  r.movements()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
