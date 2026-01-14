import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading

class MoveRobotServerNode(Node):
    def __init__(self):
        super().__init__("move_robot_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.position = 50
        self.count_until_server_ = ActionServer(
            self,
            MoveRobot,
            "move_robot",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Action server has been started")

    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Recieved a goal")
        target_position = goal_request.position

        if target_position > 100 or target_position < 0:
            self.get_logger().warn("Rejecting the goal")
            return GoalResponse.REJECT

        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handle_.abort()
                return GoalResponse.ACCEPT
        return GoalResponse.ACCEPT
    

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        
        self.get_logger().info("Executing the goal")
        feedback = MoveRobot.Feedback()
        result = MoveRobot.Result()
        target_position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        # Perform execution 
        while self.position < target_position and self.position + velocity <= target_position:
            if goal_handle.is_cancel_requested or not goal_handle.is_active:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.position = self.position
                result.message = "Failed"
                return result
            self.position += velocity
            feedback.current_position = self.position
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("Moved " + str(velocity) + " distance" )
            time.sleep(1)
        diff = target_position - self.position
        if diff > 0:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.position = self.position
                result.message = "Failed"
                return result
            self.position = target_position
            feedback.current_position = self.position
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("Moved " + str(diff) + " distance")

        goal_handle.succeed()
        result.position = self.position
        result.message = "Succeeded"
        return result
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Recieved a cancel request")
        return CancelResponse.ACCEPT

        
def main(args=None):
        rclpy.init(args=args)
        node = MoveRobotServerNode()
        rclpy.spin(node, MultiThreadedExecutor())
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()


