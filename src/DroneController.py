#!/usr/bin/env python
import roslib

import sys
import rospy
import cv2
import numpy as np

from math import pow
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Joy
from ardrone_autonomy.srv import CamSelect

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DroneController:
    #           _________
    #          |        |
    #width =  |  | 0   |
    #        |________|
    #
    #           ____
    #height =  | o |
    #         | - |
    #        |___|
    # Constructor
    def __init__(self):
        self.land = rospy.Publisher("/ardrone/land",Empty, queue_size=1)
        self.takeoff = rospy.Publisher("/ardrone/takeoff",Empty, queue_size=1)
        self.navdata = rospy.Subscriber("/ardrone/navdata", Navdata, self.navCallback)
        self.image = rospy.Publisher("/ardrone/tag_tracker",Image, queue_size=1)
        self.move = rospy.Publisher("cmd_vel",Twist, queue_size=1)
        self.joy = rospy.Subscriber("/joy",Joy, self.callback)
        self.camera= rospy.Subscriber("/ardrone/bottom/image_raw", Image, self.imageCallback)
        self.twist = Twist()
        self.bridge = CvBridge()
        self.empty = Empty()
        self.autonomy = False
        self.centered = False
        self.newWidth = 0
        self.newHeight = 0
        self.angle=0
        self.x = 500;
        self.y= 500;
        self.distance = 0;
    def navCallback(self, data):
        if self.autonomy:
            if data.tags_count==1:
                self.x = data.tags_xc[0]*640/1000
                self.y = data.tags_yc[0]*360/1000
                height = data.tags_height[0]*640/1000
                width = data.tags_width[0]*360/1000
                self.distance = data.tags_distance[0]-20
                self.angle = data.tags_orientation[0]
                self.twist.angular.z=0
                #print str(distance) +','+str(height)+','+str(width)
                try:
                    image_b = self.bridge.imgmsg_to_cv2(self.imageview, "bgr8")
                    #Dibuja el recatangulo.
                    cv2.rectangle(image_b,(self.x-height/2,self.y-width),(self.x+height/2,self.y+width),(0,255,0),1)
                    self.newWidth = int(round(5688.2*pow(self.distance,-1.035)))
                    self.newHeight = int(round(13107*pow(self.distance,-1.028)))
                    cv2.rectangle(image_b,(320-self.newHeight/2,180-self.newWidth),(320+self.newHeight/2,180+self.newWidth),(0,0,255),1)
                    self.image.publish(self.bridge.cv2_to_imgmsg(image_b, "bgr8"))

                    #if angle >5 and angle < 90:
                    #    self.twist.angular.z=0.5
                    #elif angle >= 90 and angle < 175:
                    #    self.twist.angular.z = -0.5
                    #elif angle > 185 and angle <= 270:
                    #    self.twist.angular.z = 0.5
                    #elif angle > 270 and angle < 355:
                    #    self.twist.angular.z = -0.5
                    if (self.distance <= 45 and self.centered):
                        self.land.publish(self.empty)
                    else:
                        self.tryCenter(self.x,self.y,self.distance)
                except CvBridgeError as e:
                  print(e)
                except AttributeError as e:
                  print(e)
            elif data.tags_count==0 and not self.centered:
                self.twist.angular.z=0
                self.tryCenter(self.x,self.y,self.distance)
                self.update()

    def imageCallback(self,data):
        self.imageview = data
    def tryCenter(self,x,y,distance):
        centered = False
        if x<320-self.newHeight/2:
            #estoy a la izquierda
            self.twist.linear.y = -0.1
            self.twist.linear.x = 0

        elif x>320+self.newHeight/2:
            #estoy a la derecha
            self.twist.linear.y = 0.1
            self.twist.linear.x = 0

        elif y>180+self.newWidth:
            #estoy atras
            self.twist.linear.x = 0.1
            self.twist.linear.y = 0

        elif y<180-self.newWidth:
            #estoy adelante
            self.twist.linear.x = -0.1
            self.twist.linear.y = 0

        if x>320-self.newHeight/2 and x<320+self.newHeight/2 and y>180-self.newWidth and y< 180+self.newWidth :
            #baja
            rospy.loginfo("angle is:" + str(self.angle))
            self.twist.linear.y = 0
            self.twist.linear.z = -1
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.centered = True
            rospy.loginfo('x:'+str(x)+' y:'+str(y)+' distance:'+str(distance))

    def update(self):
        self.move.publish(self.twist)

    def adjust(self):
        twist2 = Twist()
        twist2.linear.x = -1.0*self.twist.linear.x
        twist2.linear.y = -1.0*self.twist.linear.y
        self.move.publish(twist2)

    def callback(self, data):
        if not self.autonomy:
            self.twist.linear.x = 0.1*(data.buttons[13]-data.buttons[14])
            self.twist.linear.y = (data.buttons[11]-data.buttons[12])*0.1
            self.twist.linear.z = data.axes[4]
            self.twist.angular.z = data.axes[3]*0.1
            if (data.buttons[10]):
                self.takeoff.publish(self.empty)
        if (data.buttons[9]):
            self.land.publish(self.empty)
        if (data.buttons[0]):
            self.autonomy= not self.autonomy
            if self.autonomy:
                rospy.loginfo('autonomy drone!')
            else:
                rospy.loginfo('user controller!')

def main(args):


  # iniciamos el nodo con su nombre
  rospy.init_node('drone_controller')

  # creamos instancia de clase DroneController
  drone = DroneController()
  #try:
#      rospy.wait_for_service('ardrone/setcamchannel', timeout = 2)
#  except rospy.ROSException:
    #  rospy.loginfo("The set_destination service never showed up!")

  #try:
    #  change_camera = rospy.ServiceProxy('ardrone/setcamchannel',CamSelect)
     # change_camera.cam_select(1)
  #except rospy.ServiceException as e:
    #  rospy.loginfo("Service call failed: %s"%e)

  # spin esperara por los nuevos mensajes
  r=rospy.Rate(105)
  r2= rospy.Rate(95)
  try:
    while not rospy.is_shutdown():
          drone.update()
          r.sleep()
          if drone.autonomy:
            drone.adjust()
          r2.sleep()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
