import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill, Spawn
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("turtle_name", "turtle1")
        self.turtle_name_ = self.get_parameter("turtle_name").get_parameter_value().string_value
        self.cb_group = ReentrantCallbackGroup()
        self.spawn_client = self.create_client(Spawn, '/spawn', callback_group=self.cb_group)
        self.kill_client = self.create_client(Kill, '/kill', callback_group=self.cb_group)
        self.get_logger().info("Turtle Controller started running")
        self.spawn_turtle()
    
    def spawn_turtle(self):
        self.spawn_client.wait_for_service()
        req = Spawn.Request()
        req.x = 4.0
        req.y = 2.0
        req.theta = 0.0
        req.name = self.turtle_name_
        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.on_done_spawn)
        time.sleep(3)
        self.kill_turtle()
    
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
    node = TurtleController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()