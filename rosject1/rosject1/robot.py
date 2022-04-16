import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

# Rosject2
from threading import Thread
from custom_interfaces.srv import FindWall


class ClientAsync(Node):

    def __init__(self):
        super().__init__('server_client')
        self.client = self.create_client(FindWall, 'findwall')
        # checks once per second if a service matching the type and name of the client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')

        # create a Empty request
        self.req = FindWall.Request()

    def send_request(self):
        # Not async
        self.future = self.client.call_async(self.req)


class Exercise31(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('exercise31')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.move_turtlebot, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        self.laser_right = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def move_turtlebot(self, msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[180]
        self.laser_right = msg.ranges[90]

    def motion(self):
        # print the data
        self.get_logger().info(
            f'Forward {self.laser_forward}\nRight{self.laser_right}\n')
        # Logic of move
        if self.laser_forward < 0.5:
            self.cmd.angular.z = 0.7
        else:
            if self.laser_right > 0.3:
                self.cmd.angular.z = -0.1
            elif self.laser_right < 0.2:
                self.cmd.angular.z = 0.1
            else:  # between 0.3 and 0.2
                self.cmd.angular.z = 0.0

        self.cmd.linear.x = 0.05

        # Publishing the cmd_vel values to topipc
        self.publisher_.publish(self.cmd)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # --- Rosject 2 begin ----
    client = ClientAsync()
    # run the send_request() method
    client.send_request()
    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                client.get_logger().info(
                    'Pretty message')
            break
    client.destroy_node()
    # --- End ---

    # --- Rosject 1 begin ----
    # declare the node constructor
    exercise31 = Exercise31()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(exercise31)
    # Explicity destroy the node
    exercise31.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
