import numpy as np

import rclpy
from rclpy.node import Node

import geometry_msgs.msg
import nav_msgs.msg
import dyn_ctrl_msgs.msg

def quat_2_heading(q):
    return np.arctan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))

def builtin_time_2_time(t_builtin):
    return t_builtin.sec + t_builtin.nanosec*1e-9

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # subscribers
        self.motion_plan_sub = self.create_subscription(dyn_ctrl_msgs.msg.RigidBodyTraj, 'motion_plan', self.motion_plan_callback, 10)
        self.odom_sub = self.create_subscription(nav_msgs.msg.Odometry, 'mocap', self.odom_callback, 10)


        # parameters
        self.declare_parameter('kth', 0.0)
        self.declare_parameter('kt', 0.0)
        self.declare_parameter('kn', 0.0)
        self.declare_parameter('dT', 1.0)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('logging_file', 'file.txt')
        self.declare_parameter('twist_stamped',0)

        self.kth = self.get_parameter('kth').get_parameter_value().double_value
        self.kt = self.get_parameter('kt').get_parameter_value().double_value
        self.kn = self.get_parameter('kn').get_parameter_value().double_value
        self.dT = self.get_parameter('dT').get_parameter_value().double_value
        self.v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self.logging_file = self.get_parameter('logging_file').get_parameter_value().string_value
        self.twist_stamped = self.get_parameter('twist_stamped').get_parameter_value().integer_value


        # publishers
        if self.twist_stamped:
            self.twist_pub = self.create_publisher(geometry_msgs.msg.TwistStamped, 'cmd_vel', 10)
        else:
            self.twist_pub = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

        self.get_logger().info(f'Path follower params: kth = {self.kth}, kt = {self.kt}, kn = {self.kn}, dT = {self.dT}, v_min = {self.v_min}, twist-stamped = {self.twist_stamped}')

        # spline objects
        self.x_sp = None
        self.y_sp = None
        self.v_sp = None
        self.th_sp = None
        self.om_sp = None

        self.t_plan = None
        self.x_plan = None
        self.y_plan = None
        self.v_plan = None
        self.th_plan = None
        self.om_plan = None

        # feedback
        self.t = None
        self.x = None
        self.y = None
        self.v = None
        self.th = None
        self.om = None

        # timer
        self.timer = self.create_timer(self.dT, self.timer_callback)

        # logging file
        self.fid = open(self.logging_file, 'w')

        # log
        self.get_logger().info('Path Follower node has been initialized')

    def destroy_node(self):
        self.fid.close()
        super().destroy_node()

    def motion_plan_callback(self, msg):
        # get position, velocity, heading, heading rate from motion plan
        N = len(msg.traj)

        self.x_plan = np.zeros((N,))
        self.y_plan = np.zeros((N,))
        self.v_plan = np.zeros((N,))
        self.th_plan = np.zeros((N,))
        self.om_plan = np.zeros((N,))
        self.t_plan = np.zeros((N,))
        for i in range(N):
            self.x_plan[i] = msg.traj[i].state.pose.position.x
            self.y_plan[i] = msg.traj[i].state.pose.position.y
            self.v_plan[i] = msg.traj[i].state.twist.linear.x
            self.th_plan[i] = quat_2_heading(msg.traj[i].state.pose.orientation)
            self.om_plan[i] = msg.traj[i].state.twist.angular.z
            self.t_plan[i] = builtin_time_2_time(msg.traj[i].header.stamp)

    def odom_callback(self, msg):
        # get current position, velocity, heading, heading rate
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.v = msg.twist.twist.linear.x
        self.th = quat_2_heading(msg.pose.pose.orientation)
        self.om = msg.twist.twist.angular.z
        self.t = builtin_time_2_time(msg.header.stamp)

    def timer_callback(self):
        if self.t_plan is None or self.x is None:
            self.get_logger().debug('Waiting for motion plan and odometry...')
            return

        # get reference position, velocity, heading, heading rate
        x_ref = np.interp(self.t, self.t_plan, self.x_plan)
        y_ref = np.interp(self.t, self.t_plan, self.y_plan)
        v_ref = np.interp(self.t, self.t_plan, self.v_plan)
        th_ref = np.interp(self.t, self.t_plan, self.th_plan)
        om_ref = np.interp(self.t, self.t_plan, self.om_plan)

        self.get_logger().debug(f'x_ref = {x_ref}, y_ref = {y_ref}, v_ref = {v_ref}, th_ref = {th_ref}, om_ref = {om_ref}')

        # get rotation matrix based on th_ref
        C_th = np.array([[np.cos(th_ref), np.sin(th_ref)], [-np.sin(th_ref), np.cos(th_ref)]])

        # rotate positions into path-aligned frame
        pos_ref = np.array([x_ref, y_ref])
        pos = np.array([self.x, self.y])
        pos_ref_rot = np.dot(C_th,pos_ref)
        pos_rot = np.dot(C_th,pos)

        # get trangent and normal errors
        e_t = pos_ref_rot[0] - pos_rot[0]
        e_n = pos_ref_rot[1] - pos_rot[1]

        # get velocity setpoint
        v_cmd = self.tangential_tracking_controller(e_t, v_ref, th_ref-self.th)

        # check for low speed control
        if np.abs(self.v) < self.v_min:
            th_cmd = self.th
            om_cmd = 0.0
        else:
            # get heading setpoint
            th_cmd = self.normal_tracking_controller(e_n, th_ref, self.v)

            # get turn rate setpoint
            om_cmd = self.heading_controller(th_cmd, self.th, om_ref)

        # publish twist message
        twist_msg = geometry_msgs.msg.Twist()
        twist_msg.linear.x = v_cmd
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = om_cmd
        if self.twist_stamped:
            twist_msg_stamped = geometry_msgs.msg.TwistStamped()
            twist_msg_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_msg_stamped.twist = twist_msg
            self.twist_pub.publish(twist_msg_stamped)
        else:
            self.twist_pub.publish(twist_msg)

        # log
        self.get_logger().info(f'commanding v = {v_cmd}, om = {om_cmd}')

        self.fid.write(f'{self.t} {self.x} {self.y} {self.v} {self.th} {self.om} {x_ref} {y_ref} {v_ref} {th_ref} {om_ref} {v_cmd} {th_cmd} {om_cmd} \n')

    def tangential_tracking_controller(self, e_t, v_ff, e_th):
        v_cmd = self.kt*e_t*np.cos(e_th) + v_ff
        return v_cmd

    def normal_tracking_controller(self, e_n, th_ff, v):

        # feedback control
        th_fb = np.arctan((1/v)*self.kn*e_n)

        th_cmd = th_fb + th_ff
        return th_cmd

    def heading_controller(self, th_ref, th, om_ff):
        # error
        e_th = th_ref - th
        if e_th > np.pi:
            e_th -= 2*np.pi
        elif e_th < -np.pi:
            e_th += 2*np.pi

        # compute control input
        om_fb = self.kth*e_th
        om_cmd = om_fb + om_ff

        return om_cmd


def main(args=None):
    rclpy.init(args=args)

    path_follower = PathFollower()

    try:
        rclpy.spin(path_follower)
    except KeyboardInterrupt:
        path_follower.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()