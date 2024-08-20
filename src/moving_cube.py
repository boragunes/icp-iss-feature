import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class ComplexDataPublisher(Node):
    def __init__(self):
        super().__init__('complex_data_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'ls128/lslidar', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds
        self.time = 0.0

    def timer_callback(self):
        self.time += 0.1  # Increment time for dynamic elements

        # Parameters
        num_points = 5000  # Number of points in the point cloud
        size = 10.0  # Size of the environment
        obstacle_radius = 1.0  # Radius of obstacles

        points = []

        # Create a grid of obstacles
        for x in np.arange(-size / 2, size / 2, obstacle_radius * 2):
            for y in np.arange(-size / 2, size / 2, obstacle_radius * 2):
                for z in np.arange(-size / 2, size / 2, obstacle_radius * 2):
                    # Randomize obstacle position slightly
                    noise = np.random.uniform(-0.1, 0.1, 3)
                    cx = x + noise[0]
                    cy = y + noise[1]
                    cz = z + noise[2]
                    # Add points within the obstacle radius
                    for _ in range(num_points // (size * size)):
                        theta = np.random.uniform(0, 2 * np.pi)
                        phi = np.random.uniform(0, np.pi)
                        r = np.random.uniform(0, obstacle_radius)
                        x_p = cx + r * np.sin(phi) * np.cos(theta)
                        y_p = cy + r * np.sin(phi) * np.sin(theta)
                        z_p = cz + r * np.cos(phi)
                        points.append((x_p, y_p, z_p))

        # Convert the points to PointCloud2
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'lslidar'

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud_msg)
        self.get_logger().info(f'Publishing PointCloud2 with {len(points)} points')

def main(args=None):
    rclpy.init(args=args)
    node = ComplexDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
