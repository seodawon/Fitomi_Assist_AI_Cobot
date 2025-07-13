import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_interfaces.action import TaskCommand
from rclpy.callback_groups import ReentrantCallbackGroup

class TaskCommandActionServer(Node):
    def __init__(self):
        super().__init__('task_command_action_server')
        self._action_server = ActionServer(
            self,
            TaskCommand,
            '/task_command',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("TaskCommand Action Server 시작됨")

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Goal 수신: {goal_handle.request.keyword_type}, {goal_handle.request.keyword}")
        
        # 피드백 처리
        feedback_msg = TaskCommand.Feedback()
        feedback_msg.message = f"{goal_handle.request.keyword_type} 전달 작업을 진행 중입니다."
        print(feedback_msg.message)
        goal_handle.publish_feedback(feedback_msg)

        # 결과 처리
        print("결과 처리 시작")
        result = TaskCommand.Result()
        result.success = True
        result.scan_list = ["반팔", "반바지", "긴바지"]

        print("성공 결과 반환")
        goal_handle.succeed()
        
        return result

def main():
    rclpy.init()
    action_server = TaskCommandActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass

    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
