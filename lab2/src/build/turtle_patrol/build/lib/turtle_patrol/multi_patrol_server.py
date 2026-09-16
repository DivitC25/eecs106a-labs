import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class MultiPatrolServer(Node):
    def __init__(self):
        super().__init__('multi_patrol_server')

        # Publisher: actually drives turtle1
        self._turtles = {}
        self._srv = self.create_service(Patrol, '/turtle_patrol', self.patrol_callback)
        self._pub_timer = self.create_timer(0.1, self._publish_current_cmd)
        self.get_logger().info('MultiPatrolServer ready (continuous publish mode).')
    
    def _publish_current_cmd(self):
        for turtle_name in self._turtles.keys():
            msg = Twist()
            msg.linear.x = self._turtles[turtle_name]['lin']
            msg.angular.z = self._turtles[turtle_name]['ang']
            self._turtles[turtle_name]['publisher'].publish(msg)


    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        turtle = {
            'publisher': self.create_publisher(Twist, f'/{request.turtle_name}/cmd_vel', 10),
            'lin': 0.0,
            'ang': 0.0,
        }

        self._turtles[request.turtle_name] = turtle

        teleport_client = self.create_client(
            TeleportAbsolute,
            f'/{request.turtle_name}/teleport_absolute'
        )

        while not teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {request.turtle_name} teleport"
            )
        
        teleport.request = TeleportAbsolute.Request()
        teleport.request.x = float(request.x)
        teleport.request.y = float(request.y)
        teleport.request.theta = float(request.theta)

        future = teleport_client.call_async(teleport_request)
        rclpy.spin_until_future_complete(self, future)

        self._turtles[request.turtle_name]['lin'] = request.vel
        self._turtles[request.turtle_name]['ang'] = request.omega


        self.get_logger().info(
            f"Patrol request: vel={request.vel:.2f}, omega={request.omega:.2f}"
        )


        # Prepare response Twist reflecting current command
        cmd = Twist()
        cmd.linear.x = self._turtles[request.turtle_name]['lin']
        cmd.angular.z = self._turtles[request.turtle_name]['ang']
        response.cmd = cmd

        self.get_logger().info(
            f"Streaming cmd_vel: lin.x={self._turtles[request.turtle_name]['lin']:.2f}, ang.z={self._turtles[request.turtle_name]['ang']:.2f} (10 Hz)"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MultiPatrolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()