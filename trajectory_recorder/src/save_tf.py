#!/usr/bin/env python3
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


def quat_xyzw_to_rotmat(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    n = qx*qx + qy*qy + qz*qz + qw*qw
    if n < 1e-12:
        return np.eye(3)

    s = 2.0 / n
    xx, yy, zz = qx*qx*s, qy*qy*s, qz*qz*s
    xy, xz, yz = qx*qy*s, qx*qz*s, qy*qz*s
    wx, wy, wz = qw*qx*s, qw*qy*s, qw*qz*s

    return np.array([
        [1.0 - (yy + zz),       xy - wz,         xz + wy],
        [xy + wz,               1.0 - (xx + zz), yz - wx],
        [xz - wy,               yz + wx,         1.0 - (xx + yy)],
    ])


class SaveStaticMapGlobalTF(Node):
    def __init__(self):
        super().__init__("save_static_map_global_tf")

        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "global")
        

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.output_file = Path("./src/Floorplan-Alignment/trajectory_recorder/trajectories/orientations/orientation.json")

        # Important for /tf_static so late subscribers can still receive static TFs
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub = self.create_subscription(
            TFMessage,
            "/tf_static",
            self.callback,
            qos,
        )

        self.get_logger().info(
            f"Waiting for static transform {self.parent_frame} -> {self.child_frame} on /tf_static"
        )

    def callback(self, msg: TFMessage):
        for tf in msg.transforms:
            if tf.header.frame_id == self.parent_frame and tf.child_frame_id == self.child_frame:
                tx = tf.transform.translation.x
                ty = tf.transform.translation.y
                tz = tf.transform.translation.z

                qx = tf.transform.rotation.x
                qy = tf.transform.rotation.y
                qz = tf.transform.rotation.z
                qw = tf.transform.rotation.w

                T = np.eye(4)
                T[:3, :3] = quat_xyzw_to_rotmat(qx, qy, qz, qw)
                T[:3, 3] = [tx, ty, tz]

                yaw = float(np.arctan2(T[1, 0], T[0, 0]))

                payload = {
                    "parent_frame": self.parent_frame,
                    "child_frame": self.child_frame,
                    "translation_xyz": [tx, ty, tz],
                    "quaternion_xyzw": [qx, qy, qz, qw],
                    "yaw_rad": yaw,
                    "yaw_deg": float(np.degrees(yaw)),
                    "T_parent_child": T.tolist(),
                }

                with open(self.output_file, "w") as f:
                    json.dump(payload, f, indent=2)

                self.get_logger().info(f"Saved transform to {self.output_file}")
                self.get_logger().info(f"Yaw [deg]: {payload['yaw_deg']:.3f}")
                self.get_logger().info(
                    f"Translation: x={tx:.6f}, y={ty:.6f}, z={tz:.6f}"
                )

                rclpy.shutdown()
                return


def main(args=None):
    rclpy.init(args=args)
    node = SaveStaticMapGlobalTF()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()