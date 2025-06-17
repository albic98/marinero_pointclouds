#!/usr/bin/env python3

import rclpy
import math
import numpy as np
import tf_transformations as tf
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from plyfile import PlyData
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class PLYToPointCloud2(Node):
    def __init__(self):
        super().__init__("ply_to_pointcloud2")
        self.declare_parameter("ply_file_path", "/home/albert/LIDAR_data/Marina_Punat_zona_A_6M.ply")
        self.declare_parameter("euler_angles", [0.0, -0.135, 1.326])
        self.declare_parameter("translation", [0.2, 0.14, -1.38]) # -0.12])
        self.declare_parameter("origin", [352306.109894, 4989164.188812, 1.398855])
        
        self.ply_file_path = self.get_parameter("ply_file_path").get_parameter_value().string_value
        self.euler_angles = [angle * math.pi / 180 for angle in self.get_parameter("euler_angles").get_parameter_value().double_array_value]
        self.translation = self.get_parameter("translation").get_parameter_value().double_array_value
        self.origin = self.get_parameter("origin").get_parameter_value().double_array_value
        
        self.pointcloud_publisher = self.create_publisher(PointCloud2, "/marina_punat_pc", 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_pointcloud()
        # self.timer = self.create_timer(2.0, self.publish_pointcloud)

    def static_transform_publisher(self):
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


    def ply_to_pointcloud2(self):
        # sourcery skip: inline-immediately-returned-variable
        try:
            # Load the ply file
            ply_data = PlyData.read(self.ply_file_path)

            # Extract vertex data
            vertex = ply_data["vertex"]

            # Define the origin point
            origin_x, origin_y, origin_z = self.origin

            # Create numpy array for the PointCloud2 fields, adjusting the coordinates
            points = np.array([(
                float(vertex[i][0] - origin_x), float(vertex[i][1] - origin_y), float(vertex[i][2]),  # Adjusted x, y, z as float32
                (vertex[i][3] << 16) | (vertex[i][4] << 8) | vertex[i][5],  # rgb
                float(vertex[i][6]), float(vertex[i][7]), float(vertex[i][8]), float(vertex[i][9]), float(vertex[i][10])  # rest of the fields
            ) for i in range(len(vertex))], dtype=[
                ("x", "float32"), ("y", "float32"), ("z", "float32"),
                ("rgb", "uint32"), ("intensity", "float32"), ("return_number", "float32"),
                ("number_of_returns", "float32"), ("scanner_channel", "float32"), ("scan_angle", "float32")
            ])
            

            # Calculate point step (total size of one point in bytes)
            point_step = 36  # 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4

            # Create the PointCloud2 message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "pointcloud_frame"  # Ensure this frame exists in your TF tree

            point_cloud = PointCloud2()
            point_cloud.header = header
            point_cloud.height = 1
            point_cloud.width = points.shape[0]
            point_cloud.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
                # PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
                # PointField(name="return_number", offset=20, datatype=PointField.FLOAT32, count=1),
                # PointField(name="number_of_returns", offset=24, datatype=PointField.FLOAT32, count=1),
                # PointField(name="scanner_channel", offset=28, datatype=PointField.FLOAT32, count=1),
                # PointField(name="scan_angle", offset=32, datatype=PointField.FLOAT32, count=1),
            ]
            point_cloud.is_bigendian = False
            point_cloud.point_step = point_step
            point_cloud.row_step = point_step * points.shape[0]
            point_cloud.data = points.tobytes()
            point_cloud.is_dense = True

            return point_cloud

        except Exception as e:
            self.get_logger().error(f"Failed to read PLY file: {e}")
            return None


    def publish_pointcloud(self):
        self.static_transform_publisher()
        point_cloud = self.ply_to_pointcloud2()
        if point_cloud:
            self.pointcloud_publisher.publish(point_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = PLYToPointCloud2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()