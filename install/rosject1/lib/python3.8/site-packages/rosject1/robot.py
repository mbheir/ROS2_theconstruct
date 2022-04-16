import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


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
