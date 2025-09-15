import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.lidar_data = []

    def lidar_callback(self, msg):
        self.lidar_data = msg.ranges

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizer()

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'r-')
    ax.set_ylim(0, 4)
    ax.set_xlim(0, 360)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            data = node.lidar_data
            if data:
                x = np.arange(len(data))
                y = np.array(data)
                line.set_data(x, y)
                ax.set_xlim(0, len(data))
                fig.canvas.draw()
                fig.canvas.flush_events()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
