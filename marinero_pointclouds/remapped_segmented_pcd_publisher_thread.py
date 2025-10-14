#!/usr/bin/env python3

import math
import rclpy
import threading
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf_transformations as tf
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class PCDPublisher(Node):
    def __init__(self):
        super().__init__("pcd_publisher")

        self.zone_A = {
            "reduced_pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_A_500K_remapped.pcd",
            "pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_A_10M_remapped.pcd",
            "euler_angles": [0.0, -0.135, 1.326],
            "translation": [0.2, 0.14, -1.38] # -0.12],
        }
        self.zone_B = {
            "reduced_pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_B_500K_remapped.pcd",
            "pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_B_15M_remapped.pcd",
            "euler_angles": [0.0, -0.0725, 1.3332],
            "translation": [170.45, 357.53, -1.01] # 0.25],
        }
        self.zone_C = {
            "reduced_pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_C_200K_remapped.pcd",
            "pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_C_6M_remapped.pcd",
            "euler_angles": [-0.138, 0.0, 1.355],
            "translation": [196.435, 661.85, -1.095] # 0.165],
        }
        self.current_zone = self.zone_A  # Start in zone A
        self.reduced_pcd_published = False
        self.large_pcd_published = False
        self.repeat_pointcloud2_flag = False

        self.stop_current_publishing = False  # Flag to stop current publishing thread
        self.publish_thread = None  # Store the thread for publishing
        
        self.pose_subsriber = self.create_subscription(Odometry, "/marinero/odom", self.odom_callback, 50)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, "/marina_punat_pc", 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.create_timer(60.0, self.large_pointcloud2_repeater)


    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        
        zone_A_limit_1, zone_A_limit_2 = 301.0, 357.45
        zone_B_limit_1, zone_B_limit_2, zone_B_limit_3 = 357.45, 661.1, 668.5
        zone_C_limit = 661.1
        zone_x_min_1, zone_x_max_1 = -23.25, -4.0
        x_pose_condition_1 = zone_x_min_1 < self.pose_x < zone_x_max_1
        zone_x_min_2, zone_x_max_2 = -44.0, -38.0
        x_pose_condition_2 = zone_x_min_2 < self.pose_x < zone_x_max_2
        
        if zone_A_limit_1 <= self.pose_y < zone_A_limit_2 and x_pose_condition_1:
            if self.current_zone != self.zone_B:
                self.switch_to_zone(self.zone_B, "Opening zone B.")
                
        elif self.pose_y < zone_A_limit_2 and self.current_zone != self.zone_A:
            self.switch_to_zone(self.zone_A, "Opening zone A.")
            
        elif zone_B_limit_1 <= self.pose_y < zone_B_limit_2 and self.current_zone != self.zone_B:
            self.switch_to_zone(self.zone_B, "Opening zone B.")
            
        elif zone_B_limit_2 <= self.pose_y < zone_B_limit_3 and x_pose_condition_2:
            if self.current_zone != self.zone_B:
                self.switch_to_zone(self.zone_B, "Opening zone B.")
                
        elif self.pose_y > zone_C_limit and self.current_zone != self.zone_C:
            self.switch_to_zone(self.zone_C, "Opening zone C.")
        
        self.declare_parameter_if_not_declared("reduced_pcd_file_path", self.current_zone["reduced_pcd_file_path"])
        self.declare_parameter_if_not_declared("pcd_file_path", self.current_zone["pcd_file_path"])
        self.declare_parameter_if_not_declared("euler_angles", self.current_zone["euler_angles"])
        self.declare_parameter_if_not_declared("translation", self.current_zone["translation"])

        self.reduced_pcd_file_path = self.get_parameter("reduced_pcd_file_path").get_parameter_value().string_value
        self.pcd_file_path = self.get_parameter("pcd_file_path").get_parameter_value().string_value
        self.euler_angles = [angle * math.pi / 180 for angle in self.get_parameter("euler_angles").get_parameter_value().double_array_value]
        self.translation = self.get_parameter("translation").get_parameter_value().double_array_value
        
        if not self.reduced_pcd_published:
            # self.get_logger().info("Publishing reduced PCD file.")
            self.reduced_pointcloud2_publisher(self.reduced_pcd_file_path)
            self.reduced_pcd_published = True
            self.repeat_pointcloud2_flag = False
        
        if not self.large_pcd_published:
            # self.get_logger().info("Publishing larger PCD file.")
            self._large_pointcloud2_publisher(self.pcd_file_path)
            self.large_pcd_published = True
            self.repeat_pointcloud2_flag = True

    def large_pointcloud2_repeater(self):
        if self.repeat_pointcloud2_flag:
            self.large_pointcloud2_publisher(self.pcd_file_path)
            # self.get_logger().info(f"Republishing current zone: {self.current_zone['pcd_file_path']}")

    def switch_to_zone(self, new_zone, log_message):
        self.current_zone = new_zone
        self.stop_current_publishing = True  # Signal to stop the current publishing
        self.reduced_pcd_published = False
        self.large_pcd_published = False
        self.get_logger().info(log_message)

    def declare_parameter_if_not_declared(self, param_name, value):
        if not self.has_parameter(param_name):
            self.declare_parameter(param_name, value)
        else:
            self.set_parameters([Parameter(param_name, Parameter.Type.from_parameter_value(value), value)])

    def _large_pointcloud2_publisher(self, file_path):
        # Stop any ongoing publishing thread
        if self.publish_thread is not None and self.publish_thread.is_alive():
            self.stop_current_publishing = True
            self.publish_thread.join()  # Wait for the current thread to finish

        # Reset the stop flag
        self.stop_current_publishing = False

        # Start a new thread to publish the point cloud
        self.publish_thread = threading.Thread(target=self.large_pointcloud2_publisher, args=(file_path,))
        self.publish_thread.start()


    def reduced_pointcloud2_publisher(self, file_path):
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

            points = []
            with open(file_path, "r") as pcd_file:
                reading_points = False
                for line in pcd_file:
                    # Skip headers until "DATA" line
                    if line.startswith("#") or line.startswith(("VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS")):
                        continue

                    if line.startswith("DATA"):
                        reading_points = True
                        continue

                    if reading_points:
                        # Extract point data: x, y, z, r, g, b
                        values = line.strip().split()
                        x, y, z = map(float, values[:3])
                        r, g, b = map(int, values[3:6])
                        rgb = (r << 16) | (g << 8) | b
                        points.append([x, y, z, rgb])

            self.publish_pointcloud2(points, file_path)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish PCD file: {e}")


    def large_pointcloud2_publisher(self, file_path):
        
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

            points = []
            with open(file_path, "r") as pcd_file:
                reading_points = False
                for line in pcd_file:
                    
                    if self.stop_current_publishing:  # Check if the process should be stopped
                        self.get_logger().info(f"Stopping publication of {file_path} due to zone change.")
                        return  # Exit the function to stop publishing

                    # Skip headers until "DATA" line
                    if line.startswith("#") or line.startswith(("VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS")):
                        continue

                    if line.startswith("DATA"):
                        reading_points = True
                        continue

                    if reading_points:
                        # Extract point data: x, y, z, r, g, b
                        values = line.strip().split()
                        x, y, z = map(float, values[:3])
                        r, g, b = map(int, values[3:6])
                        rgb = (r << 16) | (g << 8) | b
                        points.append([x, y, z, rgb])

            self.publish_pointcloud2(points, file_path)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish PCD file: {e}")


    def publish_pointcloud2(self, points, file_path):
        
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="pointcloud_frame")
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        cloud_data = pc2.create_cloud(header, fields, points)
        self.pointcloud_publisher.publish(cloud_data)
        # self.get_logger().info(f"Published point cloud from {file_path}")
        
        
def main(args=None):
    
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
