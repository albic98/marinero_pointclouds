#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf_transformations as tf
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class PCDPublisher(Node):
    def __init__(self):
        super().__init__("pcd_publisher")

        # Create a publisher for PointCloud2 messages
        self.pointcloud_publisher = self.create_publisher(PointCloud2, "/marina_punat_pc", 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # self.declare_parameter("ply_file_path", "/home/albert/LIDAR_data/Marina_Punat_zona_A_6M_remapped.ply")
        # self.declare_parameter("translation", [0.2, 0.14, -1.38]) # -0.12])
        # self.declare_parameter("euler_angles", [0.0, -0.135, 1.326])
        
        # self.declare_parameter("pcd_file_path", "/home/albert/LIDAR_data/Marina_Punat_zona_B_6M_remapped.pcd")
        # self.declare_parameter("translation", [170.45, 357.53, -1.01]) # 0.25])
        # self.declare_parameter("euler_angles", [0.0, -0.0725, 1.3332])

        # self.declare_parameter("pcd_file_path", "/home/albert/LIDAR_data/Marina_Punat_zona_C_6M_remapped.pcd")
        # self.declare_parameter("translation", [196.435, 661.85, -1.095]) # 0.165])
        # self.declare_parameter("euler_angles", [-0.138, 0.0, 1.177])

        self.pcd_file_path = self.get_parameter("pcd_file_path").get_parameter_value().string_value
        self.translation = self.get_parameter("translation").get_parameter_value().double_array_value
        self.euler_angles = [angle * math.pi / 180 for angle in self.get_parameter("euler_angles").get_parameter_value().double_array_value]

        # Combine the static transform and point cloud publishing
        self.publish_pointcloud_with_transform()

    def publish_pointcloud_with_transform(self):
        try:
            # Broadcast static transform (once before publishing the point cloud)
            rotation_angle = tf.quaternion_from_euler(self.euler_angles[0], self.euler_angles[1], self.euler_angles[2])
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "pointcloud_frame"
            t.transform.translation.x = self.translation[0]
            t.transform.translation.y = self.translation[1]
            t.transform.translation.z = self.translation[2]
            t.transform.rotation.x = rotation_angle[0]
            t.transform.rotation.y = rotation_angle[1]
            t.transform.rotation.z = rotation_angle[2]
            t.transform.rotation.w = rotation_angle[3]
            self.tf_broadcaster.sendTransform(t)

            # Start reading the PCD file and create PointCloud2
            points = []
            with open(self.pcd_file_path, "r") as pcd_file:
                reading_points = False
                for line in pcd_file:
                    # Skip headers until "DATA" line
                    if line.startswith("#") or line.startswith(("VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS")):
                        continue

                    if line.startswith("DATA"):
                        reading_points = True
                        continue

                    if reading_points:
                        # Extract point data: x, y, z, r, g, b (assuming r, g, b are in separate columns)
                        values = line.strip().split()
                        x, y, z = map(float, values[:3])
                        r, g, b = map(int, values[3:6])

                        # Pack RGB values into a single integer
                        rgb = (r << 16) | (g << 8) | b

                        # Append to the points list
                        points.append([x, y, z, rgb])

            # Create PointCloud2 message
            header = Header(stamp=self.get_clock().now().to_msg(), frame_id="pointcloud_frame")
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
            ]

            cloud_data = pc2.create_cloud(header, fields, points)
            self.pointcloud_publisher.publish(cloud_data)
            self.get_logger().info("Publishing PointCloud2 data")

        except Exception as e:
            self.get_logger().error(f"Failed to publish PCD file: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()