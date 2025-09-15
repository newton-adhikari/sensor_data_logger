#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Imu, Image
from cv_bridge import CvBridge
import csv
import cv2
from datetime import datetime
from pathlib import Path

class SensorLogger(Node):
    def __init__(self):
        super().__init__("sensor_logger")
        
        #declaring params for the sensors here
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_dir', '/workspace/sensor_logs')

        self.bridge = CvBridge()

        #data ouptput directory
        base_dir = os.path.expanduser(self.get_parameter('output_dir').get_parameter_value().string_value)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = Path(base_dir) / f'run_{ts}'
        self.img_dir = self.run_dir / 'images'
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.img_dir.mkdir(parents=True, exist_ok=True)

        #write csv files
        self.lidar_csv = open(self.run_dir / 'lidar.csv', 'w', newline='')
        self.imu_csv = open(self.run_dir / 'imu.csv', 'w', newline='')
        self.lidar_writer = csv.writer(self.lidar_csv)
        self.imu_writer = csv.writer(self.imu_csv)

        self.lidar_writer.writerow(['stamp_sec', 'stamp_nanosec', 'angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max', 'ranges_count', 'ranges'])
        self.imu_writer.writerow(['stamp_sec', 'stamp_nanosec', 'ori_x', 'ori_y', 'ori_z', 'ori_w', 'ang_v_x', 'ang_v_y', 'ang_v_z', 'lin_a_x', 'lin_a_y', 'lin_a_z'])

        #subscribe for data
        self.create_subscription(LaserScan, self.get_parameter('lidar_topic').value, self.lidar_cb, 10)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu_cb, 10)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.image_cb, 10)

        self.get_logger().info(f'Logging to {self.run_dir}')

    def _stamp_fields(self, msg):
        return msg.header.stamp.sec, msg.header.stamp.nanosec

    def lidar_cb(self, msg: LaserScan):
            sec, nsec = self._stamp_fields(msg)
            self.lidar_writer.writerow([
                sec, nsec,
                msg.angle_min, msg.angle_max, msg.angle_increment,
                msg.range_min, msg.range_max,
                len(msg.ranges),
                ';'.join(f'{r:.3f}' for r in msg.ranges)
            ])
    
    def imu_cb(self, msg: Imu):
        sec, nsec = self._stamp_fields(msg)
        o = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration
        self.imu_writer.writerow([
            sec, nsec,
            o.x, o.y, o.z, o.w,
            av.x, av.y, av.z,
            la.x, la.y, la.z
        ])

    def image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CV bridge error: {e}')
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        out_path = self.img_dir / f'{stamp}.png'
        cv2.imwrite(str(out_path), cv_img)


    def destroy_node(self):
        super().destroy_node()
        self.lidar_csv.close()
        self.imu_csv.close()


def main(args=None):
    rclpy.init(args=args)
    node = SensorLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()