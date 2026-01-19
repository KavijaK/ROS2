import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveTurtle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
import threading

class TurtleControllerServerNode(Node):
    def __init__(self):
        super().__init__("turtle_control_server")
        self.turtle_name_ = "turtle1"
        self.cb_group = ReentrantCallbackGroup()
        self.spawn_client = self.create_client(Spawn, '/spawn', callback_group=self.cb_group)
        self.kill_client = self.create_client(Kill, '/kill', callback_group=self.cb_group)
        self.kill_turtle()
        time.sleep(1.5)
        self.declare_parameter("turtle_name", "turtle1")
        self.turtle_name_ = self.get_parameter("turtle_name").get_parameter_value().string_value
        self.goal_handle_: ServerGoalHandle = None
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/" + self.turtle_name_ + "/cmd_vel", 10)
        self.goal_lock_ = threading.Lock()
        self.count_until_server_ = ActionServer(
            self,
            MoveTurtle,
            "move_turtle",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info("Turtle Controller for " + self.turtle_name_  + " started running")
        self.spawn_turtle()

    def goal_callback(self, goal_request: MoveTurtle.Goal):
        self.get_logger().info("Recieved a goal")
        linear_vel_x = goal_request.linear_vel_x
        angular_vel_z = goal_request.angular_vel_z
        duration = goal_request.duration_sec

        if duration < 0:
            self.get_logger().warn("Rejecting the goal")
            return GoalResponse.REJECT

        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Rejecting the goal")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        
        self.get_logger().info("Executing the goal")
        result = MoveTurtle.Result()
        linear_vel_x = goal_handle.request.linear_vel_x
        angular_vel_z = goal_handle.request.angular_vel_z
        duration = goal_handle.request.duration_sec
        twist = Twist()
        twist.linear.x = linear_vel_x
        twist.angular.z = angular_vel_z
        # Perform execution
        start_time = time.time()
        rate = 0.1
        while time.time() - start_time <= duration:
            if goal_handle.is_cancel_requested or not goal_handle.is_active:
                self.get_logger().info("Cancelling the goal")
                self.goal_handle_ = None
                self.cmd_vel_pub_.publish(Twist())
                goal_handle.canceled()
                result.success = False
                result.message = "Turtle movement cancelled"
                return result
            self.cmd_vel_pub_.publish(twist)
            time.sleep(rate)
        self.cmd_vel_pub_.publish(Twist())
        goal_handle.succeed()
        result.success = True
        result.message = "Successfully moved turtle"
        return result
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Recieved a cancel request")
        return CancelResponse.ACCEPT
    
    def spawn_turtle(self):
        self.spawn_client.wait_for_service()
        req = Spawn.Request()
        req.x = 4.0
        req.y = 2.0
        req.theta = 0.0
        req.name = self.turtle_name_
        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.on_done_spawn)
    
    def on_done_spawn(self, future):
        try:
            response = future.result()
            self.get_logger().info("Spawned turtle successfully")
        except Exception as e:
            self.get_logger().info("Spawn service call failed")

    def kill_turtle(self):
        self.kill_client.wait_for_service()
        req = Kill.Request()
        req.name = self.turtle_name_
        future = self.kill_client.call_async(req)
        future.add_done_callback(self.on_done_kill)
    
    def on_done_kill(self, future):
        try:
            response = future.result()
            self.get_logger().info("Killed turtle successfully")
        except Exception as e:
            self.get_logger().info("Kill service call failed")

        
def main(args=None):
        rclpy.init(args=args)
        node = TurtleControllerServerNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

        
if __name__ == "__main__":
    main()


