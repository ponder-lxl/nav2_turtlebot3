#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavCancelClient(Node):
    def __init__(self):
        super().__init__('nav_cancel_client')

        # 创建 action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None

    def send_dummy_goal(self):
        # 发送一个空目标只是为了获取 goal handle 示例（可选）
        from geometry_msgs.msg import PoseStamped
        from rclpy.duration import Duration

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # 等待 action server 可用
        self._action_client.wait_for_server()

        # 发送 goal
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted, waiting to cancel...')

        # 延迟几秒再取消，模拟在导航中取消
        self.cancel_goal()

    def cancel_goal(self):
        if self._goal_handle is None:
            self.get_logger().info('No active goal to cancel')
            return

        self.get_logger().info('Cancelling goal...')
        future_cancel = self._goal_handle.cancel_goal_async()
        future_cancel.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled!')
        else:
            self.get_logger().info('Failed to cancel goal.')

def main(args=None):
    rclpy.init(args=args)
    node = NavCancelClient()

    # 如果你已经有一个导航 goal_handle，可以直接调用 cancel_goal()
    # 这里演示完整流程：发送 dummy goal 再取消
    node.send_dummy_goal()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

