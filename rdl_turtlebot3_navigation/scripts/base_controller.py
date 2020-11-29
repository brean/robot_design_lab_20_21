#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

class MessagePasser(object):
    def __init__(self):
        # subscriptions
        rospy.Subscriber("chatter1", String, self.callback)
        # publications

        self.chatter2_pub = rospy.Publisher('chatter2', String, queue_size=1)

    def callback(self, msg):
        '''
        This callback gets triggered every time a /chatter1 msg is received
        '''
        
        two_var = Float64()
        two_var.data = 2
        # publish
        self.chatter2_pub.publish(two_var)

    def start_message_passer(self):
        # wait for ctrl + c, prevent the node from dying (to allow callbacks to be received)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('message_passer', anonymous=True)
    message_passer = MessagePasser()
    message_passer.start_message_passer()
