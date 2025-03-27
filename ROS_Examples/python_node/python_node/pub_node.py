import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose

class PythonPublisher(Node):

    def __init__(self, name='python_publisher'):
        super().__init__(node_name=name)

        self.declare_parameter("topic_name","topic")
        self.declare_parameter("timer_period",0.5)
        self.declare_parameter("message", "This is a message")

        name_of_topic = self.get_parameter("topic_name").get_parameter_value().string_value
        period = self.get_parameter("timer_period").get_parameter_value().double_value 
        self.message = self.get_parameter("message").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, name_of_topic, 10)
        #timer_period = 0.5  # seconds
        self.timer = self.create_timer(period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        # msg.data = 'Hello World: %d' % self.i
        msg.data = self.message
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PythonPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()