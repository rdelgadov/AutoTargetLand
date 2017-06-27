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
from ardrone_ps3_controller.msg import Error
from ardrone_ps3_controller.msg import Target


class Controller:

    def __init__(self):
        self.STATE="LANDED"
        self.land = rospy.Publisher("/ardrone/land",Empty, queue_size=1)
        self.takeoff = rospy.Publisher("/ardrone/takeoff",Empty, queue_size=1)
        self.navdata = rospy.Subscriber("/ardrone/navdata", Navdata, self.navCallback)
        self.error_pub = rospy.Publisher("errors", Error, queue_size=1)
        self.pos_pub = rospy.Publisher("pos", Target, queue_size=1)
        #self.image = rospy.Publisher("/ardrone/tag_tracker",Image, queue_size=1)
        self.move = rospy.Publisher("cmd_vel",Twist, queue_size=1)
        self.joy = rospy.Subscriber("/joy",Joy, self.joyCallback)

        self.pos = Target()
        self.twist = Twist()
        self.empty = Empty()
        self.error = Error()
        self.TARGET_SEE = False
        self.t = 0.6
        self.break_time = 1.0/15
        self.time = 0
        self.last_x = 0
        self.last_y = 0
        self.last_vx = 0
        self.last_vy = 0
        self.last_error_roll = 0
        self.last_error_pitch = 0
        self.time = 0
        self.accum_error_roll = 0
        self.accum_error_pitch = 0
        self.roll_kp = 0.05
        self.roll_ki = 0.02
        self.roll_kd = 0.008
        self.pitch_kp = 0.02
        self.pitch_ki = 0.008
        self.pitch_kd = 0.02
        self.last_time = 0
        self.x = 0
        self.y = 0
        self.distance = 0
        self.angle = 0
        self.newWidth = 0
        self.newHeight = 0
    def navCallback(self,data):
        if self.STATE == "TEST":
            height = data.tags_height[0]*640/1000
            width = data.tags_width[0]*360/1000
            self.x = (data.tags_xc[0]*640/1000-320) #Pixels
            self.y = (180-data.tags_yc[0]*360/1000) #pixels
            self.y = self.y*0.158/width
            self.x = self.x*0.2/height
            self.last_vx = data.vx*1.0/1000
            self.last_vy = data.vy*1.0/1000
            self.twist.linear.x = 1
            self.twist.linear.y = 1
            self.twist.angular.z = 0
            self.update()
            self.STATE = "AUTONOMOUS"
        elif self.STATE == "AUTONOMOUS" and data.tm/1000-self.time>600:
            self.time = data.tm/1000
            self.last_x = self.x
            self.last_y = self.y
            if data.tags_count==1:
                height = data.tags_height[0]*640/1000
                width = data.tags_width[0]*360/1000
                self.x = (data.tags_xc[0]*640/1000-320)*0.2/height #m
                self.y = (180-data.tags_yc[0]*360/1000)*0.158/width #m
                self.distance = data.tags_distance[0]-20
                self.newWidth = int(round(5688.2*pow(self.distance,-1.035)))*0.158/width
                self.newHeight = int(round(13107*pow(self.distance,-1.028)))*0.2/height
                rospy.loginfo("tag is in : %f, %f",self.x,self.y)
                #
            else:
                self.x = self.x + data.vy*self.t/1000
                self.y = self.y - data.vx*self.t/1000
            self.pos.x_pos = self.y
            self.pos.y_pos = self.x
            #Todo medido desde el drone.
            delta_y = self.x-self.last_x    #Delta en m del target.
            delta_x = self.y-self.last_y    #Delta en m del target.
            error_roll = self.twist.linear.y*4.0-data.rotX
            error_pitch = self.twist.linear.x*4.0-data.rotY+0.2
            self.error.error_roll = error_roll
            self.error.error_pitch = error_pitch
            #rospy.loginfo("error roll, pitch: %f,%f", error_roll, error_pitch)
            #delta_yaw = self.twist.angular.z - data.rotZ
            rospy.loginfo("delta_x: %f delta_y: %f", delta_x, delta_y)
            velocity_x = -delta_x*1.0/self.t #Velocidad en m/s del drone
            velocity_y = -delta_y*1.0/self.t #Velocidad en m/s del drone
            rospy.loginfo("velocity_x: %f, velocity_y: %f", velocity_x, velocity_y)
            # Si hay datos de la velocidad en el drone, se toman esas velocidades
            self.accum_error_roll = self.accum_error_roll + error_roll*self.t
            self.accum_error_pitch = self.accum_error_pitch + error_pitch*self.t
            derivate_error_roll = (error_roll - self.last_error_roll)/self.t
            derivate_error_pitch = (error_pitch - self.last_error_pitch)/self.t
            self.last_error_roll = error_roll
            self.last_error_pitch = error_pitch
            self.error.accum_error_roll = self.accum_error_roll
            self.error.accum_error_pitch = self.accum_error_pitch
            self.error.derivate_error_roll = derivate_error_roll
            self.error.derivate_error_pitch = derivate_error_pitch
            #rospy.loginfo("errors angles accum: %f, %f", self.accum_error_roll, self.accum_error_pitch)
            pitch = 0.1*(self.y) + error_pitch*self.pitch_kp  + self.accum_error_pitch*self.pitch_ki + derivate_error_pitch*self.pitch_kd
            roll = 0.1*(self.x) + error_roll*self.roll_kp + self.accum_error_roll*self.roll_ki + derivate_error_roll*self.roll_kd
            #try :
            #    roll = 0.5*(self.x)-self.x*0.32*data.vy/(1000*math.fabs(self.x)) + error_roll*self.kp + self.accum_error_roll*self.ki + derivate_error_roll*self.kd
            #except ZeroDivisionError as e:
            #    roll = 0.5*(self.x)-0.32*data.vy/1000 + error_roll*self.kp + self.accum_error_roll*self.ki + derivate_error_roll*self.kd
            #try :
            #    pitch = 0.5*(self.y)-self.y*0.32*data.vx/(1000*math.fabs(self.y)) + error_pitch*self.kp  + self.accum_error_pitch*self.ki + derivate_error_pitch*self.kd
            #except ZeroDivisionError as e:
            #    pitch = 0.5*(self.y)-0.32*data.vx/1000 + error_pitch*self.kp  + self.accum_error_pitch*self.ki + derivate_error_pitch*self.kd

            inside = self.inside()
            self.twist.linear.z = 0
            self.twist.angular.z = 0
            try:
                roll = math.fabs(roll)*(self.x)/(math.fabs(self.x))
                self.twist.linear.y = -roll
            except ZeroDivisionError as e:
                self.twist.linear.y = -roll
            try:
                pitch = math.fabs(pitch)*(self.y)/(math.fabs(self.y))
                self.twist.linear.x = pitch
            except ZeroDivisionError as e:
                self.twist.linear.x = pitch
            if  inside and self.distance > 45 :
                self.twist.linear.z = -0.1
                self.twist.linear.x = self.twist.linear.x/2
                self.twist.linear.y = self.twist.linear.y/2
            elif inside and self.distance < 45:
                self.land.publish(self.empty)
                self.STATE = "LANDED"
                rospy.loginfo('landed')
            self.update()
            self.pos_pub.publish(self.pos)
            self.error_pub.publish(self.error)
        elif state == "AUTONOMOUS":
            self.x = self.x + data.vy*self.break_time/1000
            self.y = self.y - data.vx*self.break_time/1000
            inside = self.inside()
            self.distance = data.tags_distance[0]-20
            if  inside and self.distance > 45 :
                self.twist.linear.z = -0.1
                self.twist.linear.x = self.twist.linear.x/2
                self.twist.linear.y = self.twist.linear.y/2
                self.update()
            elif inside and self.distance < 45:
                self.land.publish(self.empty)
                self.STATE = "LANDED"
                rospy.loginfo('landed')
    def inside(self):
        return self.x>-self.newHeight/2 and self.x<self.newHeight/2 and self.y>-self.newWidth and self.y<self.newWidth

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
            self.last_error_roll = 0
            self.last_error_pitch = 0
            self.accum_error_roll = 0
            self.accum_error_pitch = 0
        if not(self.STATE == "LANDED") and data.buttons[9]:
            self.land.publish(self.empty)
            self.STATE = "LANDED"
            rospy.loginfo('landed')

    def update(self):
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
