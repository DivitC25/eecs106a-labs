import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):

    # Here, we define the constructor
    def __init__(self):
        # We call the Node class's constructor and call it "minimal_publisher"
        super().__init__('turtle_controller')
        turtle_name = sys.argv[1]
        
         # Here, we set that the node publishes message of type String (where did this type come from?), over a topic called "chatter_talk", and with queue size 10. The queue size limits the amount of queued messages if a subscriber doesn't receive them quickly enough.
        self.publisher_ = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
        
        # We create a timer with a callback (a function that runs automatically when something happens so you don't have to constantly check if something has happened) 
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    # Here we create a message with the counter value appended and publish it
    def timer_callback(self):
        msg = Twist()
        pressed_key = input()
        if pressed_key == 'w':
            msg.linear.x += 2.0
        elif pressed_key == 's':
            msg.linear.x -= 2.0
        elif pressed_key == 'a':
            msg.angular.z += 1.0
        elif pressed_key == 'd':
            msg.angular.z -= 1.0
        self.publisher_.publish(msg)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    turtle_controller = TurtleController()
    # Spin the node so its callbacks are called
    rclpy.spin(turtle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()