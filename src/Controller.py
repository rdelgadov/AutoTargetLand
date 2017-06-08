#!/usr/bin/env python
import roslib
import sys
import rospy
import math

from ardrone_autonomy.srv import CamSelect
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist



class Controller:

    def __init__(self):
        self.STATE="LANDED"
        self.land = rospy.Publisher("/ardrone/land",Empty, queue_size=1)
        self.takeoff = rospy.Publisher("/ardrone/takeoff",Empty, queue_size=1)
        self.navdata = rospy.Subscriber("/ardrone/navdata", Navdata, self.navCallback)
        #self.image = rospy.Publisher("/ardrone/tag_tracker",Image, queue_size=1)
        self.move = rospy.Publisher("cmd_vel",Twist, queue_size=1)
        self.joy = rospy.Subscriber("/joy",Joy, self.joyCallback)
        self.twist = Twist()
        self.empty = Empty()
        self.TARGET_SEE = False
        self.t = 1.0/30
        self.last_x = 0
        self.last_y = 0
        self.time = 0
        self.vel_x = 0
        self.vel_y = 0
        self.last_time = 0
        self.x = 0
        self.y = 0
        self.distance = 0
        self.angle = 0
    def navCallback(self,data):
        if self.STATE == "AUTONOMOUS":
            if data.tags_count==1:
                self.last_time = self.time
                self.last_x = self.x
                self.last_y = self.y
                self.time = rospy.get_rostime()
                self.x = data.tags_xc[0]*640/1000
                self.y = data.tags_yc[0]*360/1000
                height = data.tags_height[0]*640/1000
                width = data.tags_width[0]*360/1000
                self.distance = data.tags_distance[0]-20
                self.angle = data.tags_orientation[0]
                if self.TARGET_SEE:
                    delta_x = self.x-self.last_x
                    delta_y = self.y-self.last_y
                    velocity_x = (20.0*delta_x/height)*self.t
                    velocity_y = (15.3*delta_y/width)*self.t
                #    if delta_x > 1:
                        #rospy.loginfo("Delta x: "+str(20.0*delta_x/height))
                        #rospy.loginfo("Velocity x: "+str((20.0*delta_x/height)*self.t))
                #    if delta_y > 1:
                        #rospy.loginfo("Delta y:"+str(15.3*delta_y/width))
                        #rospy.loginfo("Velocity x: "+str((15.3*delta_y/width)*self.t))
                    nested_aceleration_x = 2.0*((340-self.x)*20.0/height - velocity_x*1.0/30 )*self.t*self.t
                    rospy.loginfo("Aceleration in x: "+ str(nested_aceleration_x))
                    linear_y = math.atan((nested_aceleration_x*1.0/981))*(180.0/math.pi)/4
                    rospy.loginfo("linear y: " + str(linear_y))
                    nested_aceleration_y = 0
                self.TARGET_SEE = True

            elif self.TARGET_SEE:
                pass
            else:
                self.TARGET_SEE = False
                self.STATE = "USER_CONTROLL"
                rospy.loginfo("I can't see the target, controll me!")


    def joyCallback(self,data):
        if self.STATE == "LANDED":
            if data.buttons[10]:
                takeoff_time = rospy.get_rostime()
                #self.takeoff.publish(self.empty)
                while (rospy.get_rostime().secs-takeoff_time.secs<3):
                    rospy.loginfo("takeoff, wait: "+ str(rospy.get_rostime().secs-takeoff_time.secs)+" secs and "+ str(rospy.get_rostime().nsecs-takeoff_time.nsecs) +" nsecs")
                rospy.loginfo("Ardrone is USER_CONTROLL!")
                self.STATE = "USER_CONTROLL"

        elif self.STATE == "USER_CONTROLL":
            self.twist.linear.x = 0.1*(data.buttons[13]-data.buttons[14])
            self.twist.linear.y = (data.buttons[11]-data.buttons[12])*0.1
            self.twist.linear.z = data.axes[4]
            self.twist.angular.z = data.axes[3]*0.1
            if data.buttons[0]:
                rospy.loginfo('autonomy drone!')
                self.STATE = "AUTONOMOUS"

        elif self.STATE == "AUTONOMOUS" and data.buttons[0]:
            rospy.loginfo('user controller!')
            self.STATE = "USER_CONTROLL"
        if not(self.STATE == "LANDED") and data.buttons[9]:
            self.land.publish(self.empty)
            self.STATE = "LANDED"
            rospy.loginfo('landed')


def main(args):

    #Iniciamos el nodo con su nombre
    rospy.init_node("ardrone_controller")

    #Iniciamos una instancia del controlador
    ardrone_controller = Controller()

    try:
    #Usar camara inferior del drone.
        rospy.wait_for_service('/ardrone/setcamchannel',timeout=10)
    #0 camara frontal, 1 camara inferior.
        camera_select = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)

        resp1 = camera_select(1)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    except rospy.exceptions.ROSException as exc:
        print("The camera change is not working, do it manually")
    r=rospy.Rate(1)
    try:
      while not rospy.is_shutdown():
            r.sleep()
    except KeyboardInterrupt:
      print("Shutting down")
if __name__ == '__main__':
    main(sys.argv)
