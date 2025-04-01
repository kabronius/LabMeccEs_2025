import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interface.srv import CustomSrv
from time import sleep

class PythonSubscriber(Node):
    def __init__(self, name='python_subscriber'):
        super().__init__(node_name=name)

        self.declare_parameter("topic_name","topic")
        self.declare_parameter("message", "This is a message")

        name_of_topic = self.get_parameter("topic_name").get_parameter_value().string_value
        self.message = self.get_parameter("message").get_parameter_value().string_value

        self.cli = self.create_client(CustomSrv, "/pub_start")
        self.pub = self.create_subscription(String, name_of_topic, self.msg_callback, 10)
    
    def msg_callback(self, msg: String):
        new_msg = msg.data + self.message
        self.get_logger().info(f'{new_msg}')

    def call_srv(self, data=True):
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for service...")
        self.get_logger().info("Service found...")
        self.req = CustomSrv.Request()
        self.req.start = data
        return self.cli.call_async(self.req)

def main(arg=None):
    rclpy.init(args=arg)
    node = PythonSubscriber()
    future = node.call_srv(False)
    rclpy.spin_until_future_complete(node, future) 
    response = future.result()
    if response is not None:
        node.get_logger().info(f"Service response: success:{response.success} message: {response.message}")
    else:   
        node.get_logger().error("Something was wrong...")
    node.destroy_node()
    rclpy.shutdown()

if '__name__'=='__main__':
    main()
