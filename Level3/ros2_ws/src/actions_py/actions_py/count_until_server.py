import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.count_until_server_ = ActionServer(
            self,
            CountUntil,
            "count_until",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback)
        self.get_logger().info("Action server has been started")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Recieved a goal")
        # Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT
        else: 
            self.get_logger().info("Accepting the goal")
            return GoalResponse.ACCEPT
    

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Get request
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.reached_number = counter
                return result
            counter += 1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
        # Set goal final state
        goal_handle.succeed()
        # Send result 
        result.reached_number = counter
        return result

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Recieved a cancel request")
        return CancelResponse.ACCEPT

    
def main(args=None):
        rclpy.init(args=args)
        node = CountUntilServerNode()
        rclpy.spin(node)
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()