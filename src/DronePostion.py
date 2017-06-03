#!/usr/bin/env python
import roslib

import sys
import rospy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class DronePositionate:

    def __init__(self):
        self.land = rospy.Publisher("/ardrone/land",Empty, queue_size=1)
        self.move = rospy.Publisher("cmd_vel",Twist, queue_size=1)
        self.joy = rospy.Subscriber("/ardrone/navdata",Navdata, self.callback)
        self.twist = Twist()
        self.empty = Empty()
        self.r = rospy.Rate(15) #200hz

    def callback(self, data):
        if data.tags_count==1:
            self.twist.linear.x=0
            self.twist.linear.y=0
            self.twist.linear.z=0
            x=data.tags_xc[0]-500
            y=data.tags_yc[0]-500
            w=data.tags_width[0]+20
            h=data.tags_height[0]+20
            if y<-w/2:
                print "arriba a ",y
                self.twist.linear.y=0
                self.twist.linear.z=0
                self.twist.linear.x=0.1
            elif y>w/2:
                print "abajo a",y
                self.twist.linear.y=0
                self.twist.linear.z=0
                self.twist.linear.x=-0.1
            else:
                self.twist.linear.x=0
            if x<-h/2:
                print "izquierda a",x
                self.twist.linear.y=-0.1
            elif x>h/2:
                print "derecha a ",x
                self.twist.linear.y=0.1
            else:
                self.twist.linear.y=0
            self.move.publish(self.twist)
        else:
            self.land.publish(self.empty)
        self.r.sleep()
def main(args):

  # iniciamos el nodo con su nombre
  rospy.init_node('drone_positionate')
  takeoff = rospy.Publisher("/ardrone/takeoff",Empty, queue_size=1)
  takeoff.publish(Empty())
  rate = rospy.Rate(0.1)
  rate.sleep()
  takeoff.publish(Empty())
  rate = rospy.Rate(0.2)
  rate.sleep()
  rate = rospy.Rate(60)
  move = rospy.Publisher("cmd_vel",Twist, queue_size=10)
  twist = Twist()
  twist.linear.z= 0.5
  twist.linear.x = 0
  twist.linear.y = 0
  now = rospy.get_rostime()
  while rospy.get_rostime().secs-now.secs<12 :
      move.publish(twist)
      rate.sleep()
  move.publish(Twist())
  # creamos instancia de clase DroneController
  drone = DronePositionate()
  # spin esperara por los nuevos mensajes
  try:
    rospy.spin()
  except KeyboardInterrupt:
    drone.land.publish(drone.empty)
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
