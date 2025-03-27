import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonSubscriber(Node):
    def __init__(self, name='python_subscriber'):
        super().__init__(node_name=name)

        self.declare_parameter("topic_name","topic")
        self.declare_parameter("message", "This is a message")

        name_of_topic = self.get_parameter("topic_name").get_parameter_value().string_value
        self.message = self.get_parameter("message").get_parameter_value().string_value

        self.pub = self.create_subscription(String, name_of_topic, self.msg_callback, 10)

    def msg_callback(self, msg: String):
        new_msg = msg.data + self.message
        self.get_logger().info(f'{new_msg}')

def main(arg=None):
    rclpy.init(args=arg)
    node = PythonSubscriber()
    rclpy.spin(node)    
    node.destroy_node()
    rclpy.shutdown()

if '__name__'=='__main__':
    main()
