import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__("obstacle_detection")

        self.laser_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)

    def scan_callback(self, msg):
        pass


def main():
    rclpy.init()


if __name__ == "__main__":
    main()

