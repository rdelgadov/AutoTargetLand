#!/usr/bin/env python
import roslib
import sys
import rospy

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
        self.autonomy = False
    def navCallback(self,data):
        if self.STATE == "AUTONOMOUS":
            if data.tags_count==1:
                self.x = data.tags_xc[0]*640/1000
                self.y = data.tags_yc[0]*360/1000
                height = data.tags_height[0]*640/1000
                width = data.tags_width[0]*360/1000
                self.distance = data.tags_distance[0]-20
                self.angle = data.tags_orientation[0]

    def joyCallback(self,data):
        if self.STATE == "LANDED":
            if data.buttons[10]:
                takeoff_time = rospy.get_rostime()
                self.takeoff.publish(self.empty)
                while (rospy.get_rostime().secs-takeoff_time.secs<5):
                    rospy.loginfo("takeoff, wait: "+ str(rospy.get_rostime().secs-takeoff_time.secs)+" secs and "+ str(rospy.get_rostime().nsecs-takeoff_time.nsecs) +" nsecs")
                rospy.loginfo("Ardrone is flying!")
                self.STATE = "FLYING"

        elif self.STATE == "FLYING":
            self.twist.linear.x = 0.1*(data.buttons[13]-data.buttons[14])
            self.twist.linear.y = (data.buttons[11]-data.buttons[12])*0.1
            self.twist.linear.z = data.axes[4]
            self.twist.angular.z = data.axes[3]*0.1
            if data.buttons[0]:
                rospy.loginfo('autonomy drone!')
                self.STATE = "AUTONOMOUS"

        elif self.STATE == "AUTONOMOUS" and data.buttons[0]:
            rospy.loginfo('user controller!')
            self.STATE = "FLYING"
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
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
