import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("IN CONSTRUCTOR")
        self.number_ = 1
        self.publish_frequency_ = 1.0
        self.number_publisher_ = None
        self.number_timer_ = None
       
    # Create ROS2 communications
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on configure")
        # Cannot publish in inactive state, that is why we used create_lifecycle_publisher instead of create_publisher
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(1.0 / self.publish_frequency_, self.publish_number)
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on cleanup")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on activate")
        # Calling super is required
        return super().on_activate(previous_state)    
    
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on deactivate")
        # Calling super is required
        return super().on_deactivate(previous_state)
    
    def on_shutdown(self, previous_state):
        self.get_logger().info("IN on shutdown")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS
    
    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        # Publishes the msg to the topic /number
        self.number_publisher_.publish(msg)
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
