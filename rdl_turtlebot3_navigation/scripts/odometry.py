#!/usr/bin/env python

import math

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class tbOdometry(object):
    def __init__(self):
        # control the frequency at which this node will run
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        self.distance_between_wheels = rospy.get_param('~distance_between_wheels', None)
        if not self.distance_between_wheels:
            rospy.logerr('Missing required parameter: distance_between_wheels')
            raise Exception('Missing required parameter')
        self.wheel_diameter = rospy.get_param('~wheel_diameter', None)
        if not self.wheel_diameter:
            rospy.logerr('Missing required parameter: wheel_diameter')
            raise Exception('Missing required parameter')
        # publications
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        # subscriptions
        rospy.Subscriber("/joint_states", JointState, self.jointStatesCB, queue_size=1)
        # init robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # initialize flag of joint states received msg
        self.is_joint_state_msg_available = False
        self.is_total_traveled_initialised = False
        # keep track of the total angle traveled to be able to compute the delta (angle increment)
        self.total_traveled_angle_left_wheel = 0.0
        self.total_traveled_angle_right_wheel = 0.0
        # init odom msg with constant data one time only
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        # give some time for the node to subscribe to the topic
        rospy.sleep(0.2)
        rospy.logdebug("Ready to compute odometry")

    def publish_robot_pose(self, rx, ry, rtheta):
        '''
        publish nav_msgs/Odometry to /odom topic
        '''
        self.odom_msg.header.stamp = rospy.Time.now()
        pose_msg = Pose()
        pose_msg.position.x = rx
        pose_msg.position.y = ry
        pose_msg.position.z = 0.0
        # convert heading angle from roll pitch yaw to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, rtheta)
        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]
        self.odom_msg.pose.pose = pose_msg
        # self.odom_msg.pose.covariance = ... # leaving covariance empty for now
        # self.odom_msg.twist = ... # leaving odom speed information empty for now...
        self.odom_pub.publish(self.odom_msg)

    def update_robot_pose(self, distance_traveled, rotation_applied):
        '''
        we update the robot pose, based on the distance traveled by the wheels
        It is important that the input distances are small and sampled
        at a high frequency to minimize the error introduced by the linearization of the model
        '''

        self.theta += rotation_applied
        self.x += math.cos(self.theta) * distance_traveled
        self.y += math.sin(self.theta) * distance_traveled

        # publish robot pose
        self.publish_robot_pose(self.x, self.y, self.theta)
        # broadcast transform
        tf.TransformBroadcaster().sendTransform((self.x, self.y, 0.0),
            tf.transformations.quaternion_from_euler(0.0, 0.0, self.theta),
            rospy.Time.now(), 'base_footprint', 'odom')

    def compute_distance_and_rotation(self, delta_traveled_angle_left_wheel, delta_traveled_angle_right_wheel):
        '''
        based on wheel diameter and traveled angle, compute the distance
        traveled by the center point and the change in heading angle
        '''
        # ============= YOUR CODE GOES HERE! =====
        #note that you can use self.distance_between_wheels and self.wheel_diameter
        vl = delta_traveled_angle_left_wheel
        vr = delta_traveled_angle_right_wheel

        dist = 0
        angle = 0
        # ============= YOUR CODE ENDS HERE! =====
        return dist,angle

    def jointStatesCB(self, msg):
        '''
        This callback gets triggered every time a joint state msg arrives
        '''

        self.is_joint_state_msg_available = True
        self.joint_state = msg

    def update(self):
        #get the current rotational positions
        left_wheel_state = self.joint_state.position[0]
        right_wheel_state = self.joint_state.position[1]
        #check if we have already initialised our totals
        #this avoids having the total traveled distances applied all at once,
        #keeping the robot at position 0 with every restart of the node
        if not self.is_total_traveled_initialised:
            self.total_traveled_angle_left_wheel = left_wheel_state
            self.total_traveled_angle_right_wheel = right_wheel_state
            self.is_total_traveled_initialised = True
        #calculate change in rotation position relative to the last calculation
        delta_traveled_angle_left_wheel = left_wheel_state - self.total_traveled_angle_left_wheel
        delta_traveled_angle_right_wheel = right_wheel_state - self.total_traveled_angle_right_wheel
        #calculate the center point distance and change in rotation
        tdist_rot = self.compute_distance_and_rotation(delta_traveled_angle_left_wheel, delta_traveled_angle_right_wheel)
        self.update_robot_pose(tdist_rot[0], tdist_rot[1])
        # save state for next loop
        self.total_traveled_angle_left_wheel = left_wheel_state
        self.total_traveled_angle_right_wheel = right_wheel_state

    def start_odometry(self):
        while not rospy.is_shutdown():
            if self.is_joint_state_msg_available:
                # lower flag
                self.is_joint_state_msg_available = False
                # update
                self.update()
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('turtlebot3_odometry', anonymous=False)
    tb_odometry = tbOdometry()
    tb_odometry.start_odometry()
