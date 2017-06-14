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
        self.t = 1.0/15
        self.last_x = 0
        self.last_y = 0
        self.last_vx = 0
        self.last_vy = 0
        self.last_error_roll = 0
        self.last_error_pitch = 0
        self.time = 0
        self.accum_error_roll = 0
        self.accum_error_pitch = 0
        self.kp = 0.001
        self.ki = 0.001
        self.kd = 0.001
        self.last_time = 0
        self.x = 0
        self.y = 0
        self.distance = 0
        self.angle = 0
        self.newWidth = 0
        self.newHeight = 0
    def navCallback(self,data):
        if self.STATE == "TEST":
            self.STATE = "AUTONOMOUS"
            self.x = (data.tags_xc[0]*640/1000-320) #Pixels
            self.y = (180-data.tags_yc[0]*360/1000) #pixels
            self.y = self.y*0.158/width
            self.x = self.x*0.2/height
            self.last_vx = data.vx*1.0/1000
            self.last_vy = data.vy*1.0/1000
            self.twist.linear.x=1
            self.twist.linear.y=1
            self.update()
        elif self.STATE == "AUTONOMOUS":
            if data.tags_count==1:
                self.last_x = self.x
                self.last_y = self.y
                height = data.tags_height[0]*640/1000
                width = data.tags_width[0]*360/1000
                self.x = (data.tags_xc[0]*640/1000-320) #Pixels
                self.y = (180-data.tags_yc[0]*360/1000) #pixels
                self.y = self.y*0.158/width
                self.x = self.x*0.2/height
                self.distance = data.tags_distance[0]-20
                self.newWidth = int(round(5688.2*pow(self.distance,-1.035)))*0.158/width
                self.newHeight = int(round(13107*pow(self.distance,-1.028)))*0.2/height
                rospy.loginfo("tag is in : %d, %d",self.x,self.y)
                # Todo medido desde el drone.
                delta_y = self.x-self.last_x    #Delta en m del target.
                delta_x = self.y-self.last_y    #Delta en m del target.
                self.error_roll = self.twist.linear.y*4.0-data.rotX
                self.error_pitch = self.twist.linear.x*4.0-data.rotY
                #delta_yaw = self.twist.angular.z - data.rotZ
                rospy.loginfo("delta_x: %d delta_y: %d", delta_x, delta_y)
                velocity_x = -delta_x*1.0/self.t #Velocidad en m/s del drone
                velocity_y = -delta_y*1.0/self.t #Velocidad en m/s del drone
                rospy.loginfo("velocity_x: %d, velocity_y: %d", velocity_x, velocity_y)
                # Si hay datos de la velocidad en el drone, se toman esas velocidades
                self.accum_error_roll = self.accum_error_roll + error_roll*self.t
                self.accum_error_pitch = self.accum_error_pitch + error_pitch*self.t
                derivate_error_roll = (self.error_roll - self.last_error_roll)/self.t
                derivate_error_pitch = (self.error_pitch - self.last_error_pitch)/self.t
                self.last_error_roll = self.error_roll
                self.last_error_pitch = self.error_pitch
                rospy.loginfo("errors angles accum: %d, %d", self.accum_error_roll, self.accum_error_pitch)
                self.accum_vel_x = self.accum_vel_x + error_velocity_x #Sumatoria caso discreto
                self.accum_vel_y = self.accum_vel_y + error_velocity_y #Sumatoria caso discreto
                aceleration_x = 2.0*(self.x/100 - velocity_x*self.t)/(self.t*self.t)
                aceleration_y = 2.0*(self.y/100 - velocity_y*self.t)/(self.t*self.t)
                self.last_vx = velocity_x
                self.last_vy = velocity_y
                rospy.loginfo("Aceleration: x: %s, y: %s", str(aceleration_x), str(aceleration_y))
                linear_y = math.atan((aceleration_x*1.0/9.81))*(180.0/math.pi)/4 + self.kp*error_velocity_x+self.kd*self.accum_vel_x + self.ki*(error_velocity_x-self.last_error_x)/self.t
                linear_x = math.atan((aceleration_y*1.0/9.81))*(180.0/math.pi)/4 + self.kp*error_velocity_y+self.kd*self.accum_vel_y + self.ki*(error_velocity_y-self.last_error_y)/self.t
                rospy.loginfo("\n x: "+str(linear_y)+" with error: " + str(error_velocity_x*self.kp) + " and accum error: " + str(self.accum_vel_x*self.ki) + "\n y: " + str(linear_x) + " with error: " + str(error_velocity_y*self.kp) + " and accum error: "+str(self.accum_vel_y*self.ki))
                self.twist.linear.y = -linear_y/2.0
                self.twist.linear.x = linear_x/2.0
            else
                self.TARGET_SEE = False
                self.STATE = "USER_CONTROLL"
                rospy.loginfo("I can't see the target, controll me!")


    def joyCallback(self,data):
        if self.STATE == "LANDED":
            if data.buttons[10]:
                takeoff_time = rospy.get_rostime()
                self.takeoff.publish(self.empty)
                while (rospy.get_rostime().secs-takeoff_time.secs<3):
                    rospy.loginfo("takeoff, wait: "+ str(rospy.get_rostime().secs-takeoff_time.secs)+" secs and "+ str(rospy.get_rostime().nsecs-takeoff_time.nsecs) +" nsecs")
                rospy.loginfo("Ardrone is USER_CONTROLL!")
                self.STATE = "USER_CONTROLL"

        elif self.STATE == "USER_CONTROLL":
            self.twist.linear.x = 0.1*(data.buttons[13]-data.buttons[14])
            self.twist.linear.y = (data.buttons[11]-data.buttons[12])*0.1
            self.twist.linear.z = data.axes[4]*0.1
            self.twist.angular.z = data.axes[3]*0.1
            self.update()
            if data.buttons[0]:
                rospy.loginfo('autonomy drone!')
                self.STATE = "AUTONOMOUS"
            elif data.buttons[1]:
                rospy.loginfo('ardrone test!')
                self.STATE = "TEST"

        elif self.STATE == "AUTONOMOUS" and data.buttons[0]:
            rospy.loginfo('user controller!')
            self.STATE = "USER_CONTROLL"
        if not(self.STATE == "LANDED") and data.buttons[9]:
            self.land.publish(self.empty)
            self.STATE = "LANDED"
            rospy.loginfo('landed')

    def update(self):
        if self.x>-1.0*self.newWidth/2 and self.x<1.0*self.newWidth/2 and self.y<1.0*self.newHeight/2 and self.y > -1.0*self.newHeight/2:
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.linear.z = -0.1
        if self.distance < 45 and self.x>-1.0*self.newWidth/2 and self.x<1.0*self.newWidth/2 and self.y<1.0*self.newHeight/2 and self.y > -1.0*self.newHeight/2:
            self.land.publish(self.empty)
        self.move.publish(self.twist)


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
