#!/usr/bin/env python3

import rospy
import message_filters # To Achieve Multiple subscriber
from std_msgs.msg import Float64
from std_msgs.msg import String

class VESC_Node(object):
    def __init__(self):

        rospy.init_node("test_pub_py", anonymous=True)

        # Rate
        self.loop_rate = rospy.Rate(60)

        # Node is subscribing to the topic
        #self.rgb_sub = message_filters.Subscriber('rgb/image_raw', Image)
        #self.depth_sub = message_filters.Subscriber('depth_to_rgb/image_raw', Image)

        # Node is publishing to the topic
        self.vesc1_pub = rospy.Publisher('vesc1_speed', Float64, queue_size=10)
        self.vesc2_pub = rospy.Publisher('vesc2_speed', Float64, queue_size=10)
        self.state_pub = rospy.Publisher('state', String, queue_size=10)

    def callback(self):
        rospy.loginfo("this is a info - 1")

        

    def start(self):

        # Tells rospy the name of the node.
        # Anonymous = True makes sure the node has a unique name. Random
        # numbers are added to the end of the name. 

        #Only for multiple subscribers
        #ts = message_filters.ApproximateTimeSynchronizer([self.vesc1_pub, self.vesc2_pub], 10, 0.01)
        #ts.registerCallback(self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        while not rospy.is_shutdown():
            self.vesc1_pub.publish(1.0324)
            self.vesc2_pub.publish(9.7777)
            self.state_pub.publish("C")
            self.loop_rate.sleep()
        

if __name__ == '__main__':
    my_node = VESC_Node()
    my_node.start()
    
