import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from scripted_bot_interfaces.msg import GenericTurtlesimDebug
from tf2_ros import TransformBroadcaster
import math
import numpy as np

"""
This node attempts to make the turtlesim_node compatible with nodes that
expect to interact with a canonical robot.

A canonical robot accepts movement commands in the form of Twist messages
on /cmd_vel, and reports its pose and velocity derived from wheel encoders in the form of
Odometry messages on /odom. It also publishes the transform between the
odom frame and the base_link frame on /tf. The turtlebot simulator, turtlesim,
does not conform completely to that convention - it uses a simplified custom
message for odometry. This node transforms the simplified message into
the canonical Odometry message and a tf transform and a PoseStamped message on /map.
The map and odom frames are forced to be aligned.
"""
class GenericTurtlesim(Node):

    def __init__(self):
        super().__init__('generic_turtlesim')
        self.get_logger().info('Starting generic_turtlesim')

        # set up subscription to turtle1/pose and /cmd_vel
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_listener_cb, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_listener_cb, 10)

        # set up publication to /odom, /map, /sim_gps and /turtle1/cmd_vel and debug msgs
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.map_pub = self.create_publisher(PoseStamped, 'map', 10)  # map publishes at incoming rate
        self.sim_gps_pub = self.create_publisher(PoseStamped, 'sim_gps', 10)  # sim_gps publishes at a gps rate
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.debug_pub = self.create_publisher(GenericTurtlesimDebug, 'generic_turtlesim_debug', 10)

        self.last_gps_pub_time = self.get_clock().now()

        # set up transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
    

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q
    
    def pose_listener_cb(self, msg):
        # received pose message, publish data in an Odometry msg
        tb_odom = Odometry()
        tb_odom.header.frame_id = 'odom'
        tb_odom.child_frame_id = 'base_link'
        tb_odom.header.stamp = self.get_clock().now().to_msg()
        # linear & angular pose
        tb_odom.pose.pose.position.x = msg.x
        tb_odom.pose.pose.position.y = msg.y
        q_pose = self.quaternion_from_euler(0.0, 0.0, msg.theta)

        tb_odom.pose.pose.orientation.x = q_pose[0]
        tb_odom.pose.pose.orientation.y = q_pose[1]
        tb_odom.pose.pose.orientation.z = q_pose[2]
        tb_odom.pose.pose.orientation.w = q_pose[3]
        # linear & angular velocity
        # WARNING: I'm not sure the following code is correct;
        # the wording of "relative to child_frame_id in the msg" is confusing
        tb_odom.twist.twist.linear.x = msg.linear_velocity
        tb_odom.twist.twist.angular.z = msg.angular_velocity

        self.odom_pub.publish(tb_odom)

        # publish transform in tf
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.rotation.x = q_pose[0]
        t.transform.rotation.y = q_pose[1]
        t.transform.rotation.z = q_pose[2]
        t.transform.rotation.w = q_pose[3]

        self.tf_broadcaster.sendTransform(t)

        # publish pose on map topic
        tb_map = PoseStamped()
        tb_map.header.stamp = self.get_clock().now().to_msg()
        tb_map.header.frame_id = 'map'
        tb_map.pose.position.x = msg.x
        tb_map.pose.position.y = msg.y
        tb_map.pose.orientation.x = q_pose[0]
        tb_map.pose.orientation.y = q_pose[1]
        tb_map.pose.orientation.z = q_pose[2]
        tb_map.pose.orientation.w = q_pose[3]

        self.map_pub.publish(tb_map)

        # publish slower on sim_gps topic
        now = self.get_clock().now()
        time_since_gps_pub = (now - self.last_gps_pub_time).nanoseconds / 1e9
        if (time_since_gps_pub > 1):
            self.sim_gps_pub.publish(tb_map)
            self.last_gps_pub_time = now

        # This debug publisher is an example of how to publish your own robot data.
        # It uses the GenericTurtlesimDebug message defined in the scripted_bot_interfaces
        # package
        gts_debug = GenericTurtlesimDebug()
        gts_debug.x = msg.x
        gts_debug.y = msg.y
        gts_debug.theta = msg.theta
        gts_debug.linear_speed = msg.linear_velocity
        gts_debug.angular_speed = msg.angular_velocity
        # publish debug data
        self.debug_pub.publish(gts_debug)

    def cmd_vel_listener_cb(self, msg):
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    generic_turtlesim = GenericTurtlesim()

    rclpy.spin(generic_turtlesim)


if __name__ == '__main__':
    main()
