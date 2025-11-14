#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np

from geometry_msgs.msg import Twist, Pose

def T_DH(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),               d],
        [0,              0,                           0,                           1]
    ])


def rotation_matrix_to_quaternion(R):
    trace = np.trace(R)

    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

    return np.array([x, y, z, w])

class ri_msrpr_dk_node_ex04p02(Node):

    def __init__(self):
        super().__init__("ri_msrpr_dk_node_ex04p02")

        qos = QoSProfile(depth=10)
        self.a = np.array([-6.0, 0.0, -2.5, 2.0])
        self.alpha = np.array([1.5708, 1.5708, 1.5708, 0.0])
        self.d = np.array([-3.5, 2.5, 0.5, 0.0])
        self.theta = np.array([-2.0944, 3.1416, 2.0944, 0.0])

        # Twist subscriber
        self.cspace_sub = self.create_subscription(
            Twist,
            "cspace_cmd",
            self.cspace_callback,
            qos
        )

        # Pose publisher
        self.ef_pose_pub = self.create_publisher(
            Pose,
            "ef_pose",
            qos
        )

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def cspace_callback(self, msg: Twist):
        self.theta[0] = msg.linear.x
        self.theta[1] = msg.linear.y
        self.theta[2] = msg.linear.z
        self.get_logger().info(f"Nuevos θ: {self.theta}")

    def timer_callback(self):

        T = np.eye(4)

        # Forward kinematics
        for i in range(4):
            Ai = T_DH(self.a[i], self.alpha[i], self.d[i], self.theta[i])
            T = T @ Ai

        # Position
        px, py, pz = T[0:3, 3]

        # Orientation quaternion
        R = T[0:3, 0:3]
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        # Logging
        self.get_logger().info(
            f"EF Position → x:{px:.3f}, y:{py:.3f}, z:{pz:.3f}"
        )

        # Publish Pose
        pose = Pose()
        pose.position.x = float(px)
        pose.position.y = float(py)
        pose.position.z = float(pz)

        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)

        self.ef_pose_pub.publish(pose)

def main():
    rclpy.init()
    node = ri_msrpr_dk_node_ex04p02()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
