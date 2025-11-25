#!/usr/bin/env python3

import math
import rclpy
import asyncio
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
                "reduced_pcd_file_path": "/home/LIDAR_data/Marina_Punat_zona_A_500K_remapped.pcd",
                "pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_A_10M_remapped.pcd",
                "euler_angles": [0.0, -0.135, 1.326],
                "translation": [0.2, 0.14, -1.38] # -0.12],
            },
            "zone_B": {
                "reduced_pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_B_500K_remapped.pcd",
                "pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_B_15M_remapped.pcd",
                "euler_angles": [0.0, -0.0725, 1.3332],
                "translation": [170.45, 357.53, -1.01] # 0.25],
            },
            "zone_C": {
                "reduced_pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_C_200K_remapped.pcd",
                "pcd_file_path": "/home/albert/LIDAR_data/Marina_Punat_zona_C_6M_remapped.pcd",
                "euler_angles": [-0.138, 0.0, 1.355],
                "translation": [196.435, 661.85, -1.095] # 0.165],
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

        self.check_cancel_timer = None
        self.current_goal_handle = None
        
    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request for zone: {goal_request.zone_name}")
        if goal_request.zone_name in self.zones:
            # Cancel the current goal if it exists
            if self.current_goal_handle:
                self.get_logger().info("Canceling the current goal before accepting a new one.")
                self.current_goal_handle.abort()  # Use abort instead of canceled
                self.current_goal_handle = None  # Clear the current goal handle

            return GoalResponse.ACCEPT
        self.get_logger().error("Invalid zone name.")
        return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT
        
    async def execute_callback(self, goal_handle: ServerGoalHandle):
        # Set the current goal handle
        self.current_goal_handle = goal_handle

        zone_name = goal_handle.request.zone_name
        zone_config = self.zones.get(zone_name)

        self.get_logger().info(f"Publishing point cloud for zone: {zone_name}")
        
        try:
            # Get the translation and rotation
            translation = zone_config["translation"]                                            # type: ignore
            euler_angles = [angle * math.pi / 180 for angle in zone_config["euler_angles"]]     # type: ignore
            rotation_angle = quaternion_from_euler(*euler_angles)

            # Broadcast the static transform
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "pointcloud_frame"
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.x = rotation_angle[0]
            t.transform.rotation.y = rotation_angle[1]
            t.transform.rotation.z = rotation_angle[2]
            t.transform.rotation.w = rotation_angle[3]
            self.tf_broadcaster.sendTransform(t)
            
            self.publish_reduced_pointcloud(zone_config["reduced_pcd_file_path"], goal_handle)                  # type: ignore
            
            # Check if the goal was canceled before proceeding
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal was canceled after reduced point cloud publication.")
                goal_handle.canceled()
                self.current_goal_handle = None  # Clear the current goal handle
                return PublishPointCloud.Result(success=False, message="Goal was canceled.")

            # Start the timer to check for cancel requests
            self.cancel_check_timer = self.create_timer(0.1, lambda: self.check_cancel_condition(goal_handle))
            await self.publish_larger_pointcloud(zone_config["pcd_file_path"], goal_handle)                     # type: ignore  
        
            # Stop the timer after processing is complete
            self.cancel_check_timer.cancel()
            
            # Final check before marking success
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal was canceled during larger point cloud publication.")
                goal_handle.canceled()
                self.current_goal_handle = None  # Clear the current goal handle
                return PublishPointCloud.Result(success=False, message="Goal was canceled.")
            else:
                goal_handle.succeed()
                self.current_goal_handle = None  # Clear the current goal handle
                return PublishPointCloud.Result(success=True, message=f"Successfully published point cloud for {zone_name}.")
        
        except Exception as e:
            self.get_logger().error(f"Failed to publish PCD file: {e}")
            
            # If the goal is already canceled, don't try to abort it again
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal was already canceled, no need to abort.")
                self.current_goal_handle = None  # Clear the current goal handle
                return PublishPointCloud.Result(success=False, message="Goal was canceled due to an error.")

            # Only abort if the goal is still active
            if goal_handle.is_active:
                goal_handle.abort()
            self.current_goal_handle = None  # Clear the current goal handle
            return PublishPointCloud.Result(success=False, message=str(e))

    def check_cancel_condition(self, goal_handle: ServerGoalHandle):
        if goal_handle and goal_handle.is_cancel_requested:
            self.get_logger().info("Goal was canceled by external request.")
            if goal_handle.is_active:
                goal_handle.abort()

    def publish_reduced_pointcloud(self, file_path, goal_handle):
        points = []
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

    async def publish_larger_pointcloud(self, file_path, goal_handle):
        points = []
        total_points = 0
        check_size = 1500000
        self.get_logger().info("Publishing pointcloud from larger PCD file.")

        try:
            with open(file_path, "r") as pcd_file:
                reading_points = False
                for line in pcd_file:
                    # Check if a cancel request has been received
                    if goal_handle.is_cancel_requested:
                        self.get_logger().info("Goal has been canceled during file processing.")
                        # Stop the timer and mark the goal as canceled
                        if self.cancel_check_timer is not None:
                            self.cancel_check_timer.cancel()
                        goal_handle.canceled()
                        return
                    
                    if line.startswith("DATA"):
                        reading_points = True
                        continue

                    if reading_points:
                        values = line.strip().split()
                        x, y, z = map(float, values[:3])
                        r, g, b = map(int, values[3:6])
                        rgb = (r << 16) | (g << 8) | b
                        points.append([x, y, z, rgb])

                    total_points += 1
                    if total_points % check_size == 0:
                        # Check if a cancel request has been received again before publishing feedback
                        if goal_handle.is_cancel_requested:
                            self.get_logger().info("Goal has been canceled during feedback publication.")
                            # Stop the timer and mark the goal as canceled
                            if self.cancel_check_timer is not None:
                                self.cancel_check_timer.cancel()
                            goal_handle.canceled()
                            return

                        # Provide feedback on progress
                        feedback_msg = PublishPointCloud.Feedback()
                        feedback_msg.progress = float(total_points)
                        goal_handle.publish_feedback(feedback_msg)
                        
                        await asyncio.sleep(0.0)  # Yield control to allow other tasks to run

            # Final check for cancellation before publishing the point cloud
            if not goal_handle.is_cancel_requested:
                self.publish_pointcloud2(points, file_path)
        
        except Exception as e:
            self.get_logger().error(f"Exception during point cloud publishing: {e}")
            # Ensure the goal is marked appropriately if an exception occurs
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif goal_handle.is_active:
                goal_handle.abort()
        
        
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