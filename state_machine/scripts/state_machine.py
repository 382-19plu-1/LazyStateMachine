#!/usr/bin/env python3

import rospy
import message_filters # To Achieve Multiple subscriber
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import String

class state_machine_node(object):
    def __init__(self):

        self.forwardSpeed=2000.0
        self.turnSpeed=200.0

        rospy.init_node("state_machine_node", anonymous=True, disable_signals=True)

        # Rate
        self.loop_rate = rospy.Rate(60)
        

        #get VESC path
        vesc1_ns = 'wheel_left'
        vesc2_ns = 'wheel_right'
        if not rospy.has_param('~vesc1_ns'):
            #rospy.signal_shutdown('Please specific the namespace of VESC 1')
            rospy.loginfo("Something Wrong %s", vesc1_ns)
        else:
            vesc1_ns=rospy.get_param('~vesc1_ns')

        if not rospy.has_param('~vesc2_ns') :
            #rospy.signal_shutdown('Please specific the namespace of VESC 2')
            rospy.loginfo("Something Wrong %s", vesc1_ns)
        else:
            vesc2_ns=rospy.get_param('~vesc2_ns')

        rospy.loginfo("vesc 1 namespace: %s", vesc1_ns)
        rospy.loginfo("vesc 2 namespace: %s", vesc2_ns)

        # Node is subscribing to the topic
        self.vesc1_sub = message_filters.Subscriber('vesc1_speed', Float64)
        self.vesc2_sub = message_filters.Subscriber('vesc2_speed', Float64)
        self.state_sub = message_filters.Subscriber('state', String)
        #self.vesc1_sub = rospy.Subscriber('vesc1_speed', Float32, self.callback)
        #self.vesc2_sub = rospy.Subscriber('vesc2_speed', Float32, self.callback)


        # Node is publishing to the topic
        self.vesc1_pub = rospy.Publisher(vesc1_ns + '/commands/motor/speed', Float64, queue_size=10)
        self.vesc2_pub = rospy.Publisher(vesc2_ns + '/commands/motor/speed', Float64, queue_size=10)
        rospy.loginfo("vesc 1 node: %s", vesc1_ns)
        rospy.loginfo("vesc 2 node: %s", vesc2_ns)

        self.state = "C"
        self.inputSpeed_v1=0.0
        self.inputSpeed_v2=0.0


    def callback(self,vesc1_data, vesc2_data, state_sub):
        #rospy.loginfo("vesc1 speed: %s", vesc1_data.data)
        self.inputSpeed_v1 = vesc1_data.data
        #rospy.loginfo("vesc2 speed: %s", vesc2_data.data)
        self.inputSpeed_v2 = vesc2_data.data
        rospy.loginfo("State :%s", state_sub.data)
        self.state = state_sub.data
    def start(self):

        # Tells rospy the name of the node.
        # Anonymous = True makes sure the node has a unique name. Random
        # numbers are added to the end of the name. 

        #Only for multiple subscribers
        ts = message_filters.ApproximateTimeSynchronizer([self.vesc1_sub, self.vesc2_sub, self.state_sub], 10, 0.01, allow_headerless=True)
        ts.registerCallback(self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        while not rospy.is_shutdown():
            if self.state == "C":#custom speed for PID
                self.vesc1_pub.publish(self.inputSpeed_v1)
                self.vesc2_pub.publish(self.inputSpeed_v2)
                rospy.loginfo("State: C\n vesc1 speed: %s\n vesc2 speed: %s", self.inputSpeed_v1, self.inputSpeed_v2)
            elif self.state == "W": #forward
                self.vesc1_pub.publish(self.forwardSpeed)
                self.vesc2_pub.publish(self.forwardSpeed)
            elif self.state == "E": #forward + right
                self.vesc1_pub.publish(self.forwardSpeed+self.turnSpeed)
                self.vesc2_pub.publish(self.forwardSpeed)
            elif self.state == "Q": #forward + left
                self.vesc1_pub.publish(self.forwardSpeed)
                self.vesc2_pub.publish(self.forwardSpeed+self.turnSpeed)
            elif self.state == "S": #Stop
                self.vesc1_pub.publish(0)
                self.vesc2_pub.publish(0)
            elif self.state == "A": #rotaed left
                self.vesc1_pub.publish(-self.forwardSpeed)
                self.vesc2_pub.publish(self.forwardSpeed)
            elif self.state == "D": #rotaed right
                self.vesc1_pub.publish(self.forwardSpeed)
                self.vesc2_pub.publish(-self.forwardSpeed)
            elif self.state == "X": #backword
                self.vesc1_pub.publish(-self.forwardSpeed)
                self.vesc2_pub.publish(-self.forwardSpeed)
            
                
        
        #while not rospy.is_shutdown():
            #self.vesc1_pub.publish(1.0324)
            #self.vesc2_pub.publish(9.7777)

        #    self.loop_rate.sleep()
        

if __name__ == '__main__':
    my_node = state_machine_node()
    my_node.start()
    
