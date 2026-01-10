import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.count_until_server_ = ActionServer(
            self,
            CountUntil,
            "count_until",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action server has been started")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Recieved a goal")

        # #Policy: refuse new goal if current goal still active
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Goal is already active")
        #         return GoalResponse.REJECT

        # Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT
        
        # Policy: prempt existing goal when recieving new goal
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                # Makes the goal inactive
                self.goal_handle_.abort()

                return GoalResponse.ACCEPT

        
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
    

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        self.goal_handle_ = goal_handle
        # Get request
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            # We always have to return a result in execute_callback and here we abort before done
            # Here is_active becomes false when aborted. That can be used to abort
            if not goal_handle.is_active:
                result.reached_number = counter
                return result
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
        rclpy.spin(node, MultiThreadedExecutor())
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()