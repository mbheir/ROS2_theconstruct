# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the MyCustomServiceMessage module from custom_interfaces_service interface
from custom_interfaces.srv import FindWall
# import the ROS2 python client libraries
import rclpy
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from threading import Thread
import numpy as np

# Globals
ranges = 0
laser_front = 0
laser_right = 0


class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('LaserSubscriber')
        # Create subscriber
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.update_scan, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber

    def update_scan(self, msg):
        # Save the frontal laser scan info at 0°
        #self.get_logger().info('Got new scan')
        global ranges
        global laser_front
        global laser_right

        ranges = msg.ranges
        laser_front = msg.ranges[0]
        laser_right = msg.ranges[270]


class Service(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('findwall_server')
        # create the service server object
        # defines the type, name and callback function
        self.srv = self.create_service(
            FindWall, 'findwall', self.FindWall_callback)
        # create the publisher object
        # in this case the publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def FindWall_callback(self, request, response):
        # The callback function recives the self class parameter,
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as response
        self.get_logger().info('Recieved FindWall Call')

        # create a Twist message
        direction = np.argmin(ranges)
        print(direction)

        # Turn robot to wall
        self.get_logger().info('Turning to right direction!!')
        msg = Twist()
        #msg.linear.x = -0.01
        # msg.angular.z = 0.4
        # self.publisher_.publish(msg)
        # keep turning until pointing towards wall
        while not (direction < 20 or (360 - direction) < 20):
            direction = np.argmin(ranges)
            self.get_logger().info(f"direction = {direction}")
            msg.angular.z = 0.4
            self.publisher_.publish(msg)

        # Send robot towards wall
        # print('yo')
        self.get_logger().info('Going forward!')

        msg.angular.z = 0.0
        msg.linear.x = -0.1
        self.get_logger().info('Before publish')
        self.publisher_.publish(msg)
        self.get_logger().info('Published msg!')

        # Wait until we are close to wall
        while laser_front > 0.30:
            self.get_logger().info(f"Distance to wall = {laser_front}")
            pass
        # Stop
        self.get_logger().info('Found Wall, rotating...')

        # msg.angular.z = -0.4
        # msg.linear.x = 0.0
        # self.publisher_.publish(msg)

        while not(direction > 80 and direction < 100):
            self.get_logger().info(f"direction= {direction},should be 270")
            direction = np.argmin(ranges)
            msg.angular.z = -0.4
            msg.linear.x = 0.0
            self.publisher_.publish(msg)

        self.get_logger().info('Done, return success')
        response.success = True

        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        # return the response parameter
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    laser_subscriber = LaserSubscriber()
    service = Service()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(laser_subscriber)
    executor.add_node(service)
    # Spin in a separate thread
    try:
        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()
    except Exception as e:
        print(e)
    # shutdown the ROS communication
    executor_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
