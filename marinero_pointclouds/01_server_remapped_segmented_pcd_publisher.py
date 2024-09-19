#!/usr/bin/env python3

import math
import rclpy
import asyncio
from rclpy.task import Future
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from rclpy.action.server import ServerGoalHandle
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from publish_pointcloud_interfaces.action import PublishPointCloud


class PublishPointCloudServer(Node):
    def __init__(self):
        super().__init__("publish_pointcloud_server")
        
        # Define zones with configurations
        self.zones = {
            "zone_A": {
                "reduced_pcd_file_path": "/home/albert/marinero_ws/src/LIDAR_data/Marina_Punat_zona_A_500K_remapped.pcd",
                "pcd_file_path": "/home/albert/marinero_ws/src/LIDAR_data/Marina_Punat_zona_A_10M_remapped.pcd",
                "euler_angles": [0.0, -0.135, -2.57],
                "translation": [-100.0, -48.0, -0.12],
            },
            "zone_B": {
                "reduced_pcd_file_path": "/home/albert/marinero_ws/src/LIDAR_data/Marina_Punat_zona_B_500K_remapped.pcd",
                "pcd_file_path": "/home/albert/marinero_ws/src/LIDAR_data/Marina_Punat_zona_B_10M_remapped.pcd",
                "euler_angles": [0.0, -0.135, -2.57],
                "translation": [94.2, 297.05, 0.3],
            },
            "zone_C": {
                "reduced_pcd_file_path": "/home/albert/marinero_ws/src/LIDAR_data/Marina_Punat_zona_C_200K_remapped.pcd",
                "pcd_file_path": "/home/albert/marinero_ws/src/LIDAR_data/Marina_Punat_zona_C_6M_remapped.pcd",
                "euler_angles": [0.0, -0.135, -2.57],
                "translation": [140.85, 598.8, 0.35],
            }
        }

        self.pointcloud_server_ = ActionServer(
            self,
            PublishPointCloud,
            "publish_pointcloud",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.pointcloud_publisher = self.create_publisher(PointCloud2, "/marina_punat_pc", 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.current_goal_handle = None
        
    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request for zone: {goal_request.zone_name}")
        if goal_request.zone_name in self.zones:
            return GoalResponse.ACCEPT
        self.get_logger().error("Invalid zone name.")
        return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.current_goal_handle = goal_handle
        zone_name = goal_handle.request.zone_name
        zone_config = self.zones.get(zone_name)

        if not zone_config:
            # Abort if the zone is invalid
            if not goal_handle.is_cancel_requested:
                goal_handle.abort()
            return PublishPointCloud.Result(success=False, message="Invalid zone.")

        self.get_logger().info(f"Publishing point cloud for zone: {zone_name}")
        
        try:
            # Get the translation and rotation
            translation = zone_config["translation"]
            euler_angles = [angle * math.pi / 180 for angle in zone_config["euler_angles"]]
            rotation_angle = quaternion_from_euler(*euler_angles)

            # Broadcast the static transform
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = "pointcloud_frame"
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.x = rotation_angle[0]
            t.transform.rotation.y = rotation_angle[1]
            t.transform.rotation.z = rotation_angle[2]
            t.transform.rotation.w = rotation_angle[3]
            self.tf_broadcaster.sendTransform(t)
            
            self.publish_reduced_pointcloud(zone_config["reduced_pcd_file_path"], goal_handle)
            
            # Check if the goal was canceled before proceeding
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal was canceled after reduced point cloud publication.")
                goal_handle.canceled()
                return PublishPointCloud.Result(success=False, message="Goal was canceled.")

            await self.publish_larger_pointcloud(zone_config["pcd_file_path"], goal_handle)
            
            # Final check before marking success
            if not goal_handle.is_cancel_requested:
                goal_handle.succeed()
                return PublishPointCloud.Result(success=True, message=f"Successfully published point cloud for {zone_name}.")
            else:
                # Goal was canceled during the larger point cloud publication
                self.get_logger().info("Goal was canceled during larger point cloud publication.")
                goal_handle.canceled()
                return PublishPointCloud.Result(success=False, message="Goal was canceled.")
        
        except Exception as e:
            self.get_logger().error(f"Failed to publish PCD file: {e}")
            
            # If the goal is already canceled, don"t try to abort it again
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal was already canceled, no need to abort.")
                goal_handle.canceled()
                return PublishPointCloud.Result(success=False, message="Goal was canceled due to an error.")

            # Only abort if the goal is still active
            goal_handle.abort()
            return PublishPointCloud.Result(success=False, message=str(e))


    def publish_reduced_pointcloud(self, file_path, goal_handle: ServerGoalHandle):
        points = []
        total_points = 0
        with open(file_path, "r") as pcd_file:
            reading_points = False
            for line in pcd_file:
                
                if line.startswith("DATA"):
                    reading_points = True
                    continue

                if reading_points:
                    values = line.strip().split()
                    x, y, z = map(float, values[:3])
                    r, g, b = map(int, values[3:6])
                    rgb = (r << 16) | (g << 8) | b
                    points.append([x, y, z, rgb])
                    
        # Publish the point cloud
        self.publish_pointcloud2(points, file_path)
    
    async def publish_larger_pointcloud(self, file_path, goal_handle: ServerGoalHandle):
        points = []
        total_points = 0
        self.get_logger().info("Publishing pointcloud from larger PCD file.")
        with open(file_path, "r") as pcd_file:
            reading_points = False
            for line in pcd_file:
                
                self.get_logger().info("Before asyncio.sleep.")
                await asyncio.sleep(0.1)
                self.get_logger().info("After asyncio.sleep.")
                
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal has been canceled during file processing.")
                    return
                
                if line.startswith("DATA"):
                    reading_points = True
                    continue

                if reading_points:
                    self.get_logger().info("Reading_points.")
                    values = line.strip().split()
                    x, y, z = map(float, values[:3])
                    r, g, b = map(int, values[3:6])
                    rgb = (r << 16) | (g << 8) | b
                    points.append([x, y, z, rgb])

                total_points += 1
                if total_points % 500000 == 0:
                    feedback_msg = PublishPointCloud.Feedback()
                    feedback_msg.progress = float(total_points)
                    goal_handle.publish_feedback(feedback_msg)
                

        if not goal_handle.is_cancel_requested:
            self.publish_pointcloud2(points, file_path)
        
        
    def publish_pointcloud2(self, points, file_path):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "pointcloud_frame"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        point_cloud_msg = pc2.create_cloud(header, fields, points)
        self.pointcloud_publisher.publish(point_cloud_msg)

        
def main(args=None):
    
    rclpy.init(args=args)
    node = PublishPointCloudServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()