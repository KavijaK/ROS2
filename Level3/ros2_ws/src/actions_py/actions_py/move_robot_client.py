import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import MoveRobot
from std_msgs.msg import String

class MoveRobotClientNode(Node):
    def __init__(self):
        super().__init__("move_robot_client")
        self.move_robot_client = ActionClient(self, MoveRobot, "move_robot")
        self.cancel_move = self.create_subscription(String, "cancel_move", self.cancel_move_callback, 10)
    
    def send_goal(self, target_position, velocity):
        # wait until server is available before sending goal
        self.move_robot_client.wait_for_server()
        goal = MoveRobot.Goal()
        goal.position = target_position
        goal.velocity = velocity
        # send the goal
        self.get_logger().info("Sending goal")
        self.move_robot_client.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_ACCEPTED:
            self.get_logger().info("Success")
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Aborted")
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Cancelled")
        self.get_logger().info("Result: " + str(result.position))
    
    def goal_feedback_callback(self, feedback_msg):
        current_position = feedback_msg.feedback.current_position
        self.get_logger().info("Got feedback: " + str(current_position))

    def cancel_move_callback(self, msg):
        self.get_logger().info("Recieved cancel request")
        self.goal_handle_.cancel_goal_async()

def main(args=None):
        rclpy.init(args=args)
        node = MoveRobotClientNode()
        node.send_goal(100, 1)
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    main()

        