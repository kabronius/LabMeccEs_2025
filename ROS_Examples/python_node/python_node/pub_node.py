import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from custom_interface.srv import CustomSrv

class PythonPublisher(Node):

    def __init__(self, name='python_publisher'):
        super().__init__(node_name=name)

        self.declare_parameter("topic_name","topic")
        self.declare_parameter("timer_period",0.5)
        self.declare_parameter("message", "This is a message")

        self.name_of_topic = self.get_parameter("topic_name").get_parameter_value().string_value
        self.period = self.get_parameter("timer_period").get_parameter_value().double_value 
        self.message = self.get_parameter("message").get_parameter_value().string_value

        self.srv = self.create_service(CustomSrv, "pub_start", self.srv_callback)
        self.start_flag = False

    def srv_callback(self, request: CustomSrv.Request, response: CustomSrv.Response): 
        if request.start==True:
            if self.start_flag==False:
                self.publisher = self.create_publisher(String, self.name_of_topic, 10)
                self.timer = self.create_timer(self.period, self.timer_callback)
                self.start_flag = True
                response.success = True
                response.message = "Publisher started" 
            else:
                self.start_flag = False
                response.success = False
                response.message = "Publisher already started"
        elif request.start==False:
            if self.start_flag:
                self.destroy_publisher(self.publisher)
                self.destroy_timer(self.timer)
                self.start_flag = False
                response.success = True
                response.message = "Publisher stopped"
            else:
                self.start_flag = False
                response.success = False
                response.message = "Publisher already stopped"
        # Si deve ritornare sempre la risposta del service server, per questo motivo scattava l'eccezione di TypeError() 🥲
        return response

    def timer_callback(self):
        msg = String()
        msg.data = self.message
        self.publisher.publish(msg)

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