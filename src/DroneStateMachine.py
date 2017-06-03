#!/usr/bin/env python


import roslib;
import rospy
import smach_ros
import smach
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist
import threading


class Move(smach.State):
    def __init__(self):
        #smach.State.__init__(self, outcomes=['move_x','move_y','move_z','user'])
        smach.State.__init__(self, outcomes=['user'])
        self.mutex = threading.Lock()
        self.my_x = 500
        self.my_y = 500
        self.loss = False
        self.move_to = ''
        self.nav_data = rospy.Subscriber('/ardrone/navdata',Navdata, self.callback)
    def callback(self,data):
        if data.tags_count == 0:
            self.mutex.acquire()
            self.loss = True
            self.mutex.release()
        else:
            self.mutex.acquire()
            if abs(data.tags_xc[0]-self.my_x)>data.tags_width[0]*1.0/2+20.0:
                self.move_to = 'move_x'
            elif abs(data.tags_yc[0]-self.my_y)>data.tags_height[0]*1.0/2+20.0:
                self.move_to = 'move_y'
            else:
                self.move_to = 'move_z'
            self.mutex.release()

    def execute(self, userdata):
        rospy.loginfo("searching...")
        if self.loss:
            rospy.loginfo("losted...")
            return 'user'
        elif self.move_to!='':
            rospy.loginfo("founded moving to:" + self.move_to)
        self.mutex.release()

class UserControlling(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['move'])
        self.nav_data = rospy.Subscriber('/ardrone/navdata',Navdata, self.callback)
        self.mutex = threading.Lock()
        self.my_data = Navdata()
        self.my_data.tags_count=0
    def callback(self,data):
        self.mutex.acquire()
        if data.tags_count==1:
            self.my_data=data
        self.mutex.release()
    def execute(self,userdata):
        while True:
            self.mutex.acquire()
            if self.my_data.tags_count==1:
                return 'move'
            self.mutex.release()

def main():
    rospy.init_node('smach_drone_landed')
    sm = smach.StateMachine(outcomes=['outcome'])
    with sm:
        smach.StateMachine.add('User', UserControlling(), transitions={'move':'Move'})
        smach.StateMachine.add('Move',Move(),transitions={'user':'User'})

    outcome = sm.execute()
if __name__ == '__main__':
    main()


#class MovingInX(smach.State):
#    def __init__(self):
#        smach.State.__init__(self,outcomes=['move'])
#        self.nav_data = rospy.Subscriber('/ardrone/navdata',Navdata, self.callback)
#        self.x_pos=0
#        self.height=0
#        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
#        self.mutex = threading.Lock()
#    def callback(self,data):
#        if data.tags_count==1:
#            self.mutex.acquire()
#            self.x_pos = 500-data.tags_yc[0]
#            self.height = data.tags_height[0]
#            self.mutex.release()
#    def execute(self,userdata):
#        self.mutex.acquire()
#        if abs(self.x_pos) < self.height+10:
#            return 'move'
#        else:
            #twist = Twist()
            #twist.linear.x = self.x_pos/abs(self.x_pos)
            #self.cmd_vel.publish(twist)
#            rospy.loginfo("target in: "+ self.x_pos)
#        self.mutex.release()
