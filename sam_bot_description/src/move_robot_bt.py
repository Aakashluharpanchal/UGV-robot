import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
from simple_behavior_tree import BehaviorTree, NodeStatus, LeafNode

# Define a custom behavior tree node for moving to a waypoint
class MoveToWaypointNode(LeafNode):
    def __init__(self, name, x, y, robot_controller):
        super().__init__(name)
        self.x = x
        self.y = y
        self.robot_controller = robot_controller
        self.threshold = 0.1  # acceptable distance to the waypoint

    def tick(self):
        # Get current position from the robot controller
        current_position = self.robot_controller.get_current_position()
        distance = math.sqrt((self.x - current_position[0])**2 + (self.y - current_position[1])**2)

        if distance < self.threshold:
            return NodeStatus.SUCCESS

        # Move the robot towards the waypoint
        self.robot_controller.move_towards(self.x, self.y)
        return NodeStatus.RUNNING

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_position = (0.0, 0.0)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def get_current_position(self):
        return self.current_position

    def move_towards(self, target_x, target_y):
        twist = Twist()
        # Basic logic to move towards the target
        # You can improve this with more complex control logic
        twist.linear.x = 0.5  # Forward speed
        twist.angular.z = 0.0  # No rotation
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotController()
    
    # Create behavior tree
    bt = BehaviorTree()
    bt.root = MoveToWaypointNode("move_to_waypoint_1", 1.0, 1.0, robot_controller)  # Example waypoint

    rate = robot_controller.create_rate(10)  # 10 Hz
    while rclpy.ok():
        bt.tick()  # Run the behavior tree
        rclpy.spin_once(robot_controller)
        rate.sleep()

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

