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
        
        self.declare_parameter("ply_file_path", ["/home/albert/marinero_ws/src/LIDAR_data/Marina_Punat_zona_A_6M_remapped.ply",
                                                "/home/albert/marinero_ws/src/LIDAR_data/Marina_Punat_zona_B_6M_remapped.ply",
                                                "/home/albert/marinero_ws/src/LIDAR_data/Marina_Punat_zona_C_6M_remapped.ply"])

        # self.declare_parameter("translation", [-100.0, -48.18, -0.12,
        #                                         94.10, 297.05, 0.25, 
        #                                         140.70, 598.88, 0.175])

        self.declare_parameter("euler_angles", [0.0, -0.135, 1.326,
                                                0.0, -0.0725, 1.3332,
                                                -0.138, 0.0, 1.355
                                            ])

        self.declare_parameter("translation", [0.2, 0.14, -1.38, # -0.12,
                                                170.45, 357.53, -1.01, # 0.25, 
                                                196.435, 661.85, -1.095, # 0.165
                                            ])

        self.labels = ["A", "B", "C"]
        self.ply_file_path = list(self.get_parameter("ply_file_path").get_parameter_value().string_array_value)
        translations_inline = self.get_parameter("translation").get_parameter_value().double_array_value
        self.translation = [translations_inline[i:i+3] for i in range(0, len(translations_inline), 3)]
        euler_inline = self.get_parameter("euler_angles").get_parameter_value().double_array_value
        euler_radians = [angle * math.pi / 180 for angle in euler_inline]
        self.euler_angles = [euler_radians[i:i+3] for i in range(0, len(euler_radians), 3)]

        self.pointcloud_publishers = [self.create_publisher(PointCloud2, f"/marina_punat_zone_{self.labels[i]}", 10) for i in range(len(self.ply_file_path))]

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.publish_pointclouds()

    def static_transform_publisher(self, frame_id, translation, euler_angles):
        rotation_angle = tf.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = frame_id
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation_angle[0]
        t.transform.rotation.y = rotation_angle[1]
        t.transform.rotation.z = rotation_angle[2]
        t.transform.rotation.w = rotation_angle[3]
        self.tf_broadcaster.sendTransform(t)


    def ply_to_pointcloud2(self, frame_id, ply_file_path):
        try:
            # Load the ply file
            ply_data = PlyData.read(ply_file_path)

            # Extract vertex data
            vertex = ply_data["vertex"]

            # Create numpy array for the PointCloud2 fields, adjusting the coordinates
            points = np.array([(
                float(vertex[i][0]), float(vertex[i][1]), float(vertex[i][2]),  # Adjusted x, y, z as float32
                (vertex[i][3] << 16) | (vertex[i][4] << 8) | vertex[i][5],  # rgb
                float(vertex[i][6]), float(vertex[i][7]), float(vertex[i][8]), float(vertex[i][9]), float(vertex[i][10])  # rest of the fields
            ) for i in range(len(vertex))], dtype=[
                ("x", "float32"), ("y", "float32"), ("z", "float32"),
                ("rgb", "uint32"), 
                ("intensity", "float32"), ("return_number", "float32"),
                ("number_of_returns", "float32"), ("scanner_channel", "float32"), ("scan_angle", "float32")
            ])

            # Create PointCloud2 message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = frame_id

            # Calculate point step (total size of one point in bytes)
            point_step = 36  # 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4

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

            self.get_logger().info(f"Generated pointcloud: {ply_file_path}")

            return point_cloud
        
        except Exception as e:
            self.get_logger().error(f"Failed to read PLY file: {e}")
            return None


    def publish_pointclouds(self):
        for i in range(len(self.ply_file_path)):
            ply_file_path = self.ply_file_path[i]
            translation = self.translation[i]
            euler_angles = self.euler_angles[i]
            frame_id = f"pointcloud_frame_zone_{self.labels[i]}"

            self.static_transform_publisher(frame_id, translation, euler_angles)
            point_cloud = self.ply_to_pointcloud2(frame_id, ply_file_path)
            if point_cloud:
                self.pointcloud_publishers[i].publish(point_cloud)


def main(args=None):
    
    rclpy.init(args=args)
    node = PLYToPointCloud2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()