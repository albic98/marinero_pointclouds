#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from publish_pointcloud_interfaces.action import PublishPointCloud

class PublishPointCloudClient(Node):
    def __init__(self):
        super().__init__("publish_pointcloud_client")
        
        self.current_zone = None
        self.goal_handle = None     # type: ignore
        self.success = True
        
        self.odom_subscriber = self.create_subscription(Odometry, "/marinero/odom", self.odom_callback, 50)
        self.pointcloud_client_ = ActionClient(self, PublishPointCloud, "publish_pointcloud")
        self.get_logger().info("Action client has been started.")
    
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
            if self.current_zone != "zone_B":
                self.switch_to_zone("zone_B", "Opening zone B.")
                
        elif self.pose_y < zone_A_limit_2 and self.current_zone != "zone_A":
            self.switch_to_zone("zone_A", "Opening zone A.")
            
        elif zone_B_limit_1 <= self.pose_y < zone_B_limit_2 and self.current_zone != "zone_B":
            self.switch_to_zone("zone_B", "Opening zone B.")
            
        elif zone_B_limit_2 <= self.pose_y < zone_B_limit_3 and x_pose_condition_2:
            if self.current_zone != "zone_B":
                self.switch_to_zone("zone_B", "Opening zone B.")
                
        elif self.pose_y > zone_C_limit and self.current_zone != "zone_C":
            self.switch_to_zone("zone_C", "Opening zone C.")

    def switch_to_zone(self, new_zone, log_message):
        self.current_zone = new_zone
        
        self.cancel_goal()
        
        self.success = False
        
        self.send_goal(self.current_zone)
        self.get_logger().info(log_message)
        
    def send_goal(self, zone_name):
        self.pointcloud_client_.wait_for_server()
        
        goal_msg = PublishPointCloud.Goal()
        goal_msg.zone_name = zone_name
        self.pointcloud_client_.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        self.goal_handle = None  # type: ignore
        self.goal_handle: ClientGoalHandle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info(f"Goal accepted.")
            self.get_result_future = self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Received feedback: {feedback_msg.feedback.progress}")

    def goal_result_callback(self, future):
        result = future.result().result
        self.success = bool(result.success)
        self.get_logger().info(f"Result: {result.message}")

    def cancel_goal(self):
        if self.goal_handle and not self.success:
            self.get_logger().info("Sending cancel request...")
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)
            
    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response is not None:
            self.get_logger().info("Cancel request processed.")
        else:
            self.get_logger().info("Cancel request failed.")
            
def main(args=None):
    
    rclpy.init(args=args)
    node = PublishPointCloudClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
