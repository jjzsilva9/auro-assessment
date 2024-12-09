import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSPresetProfiles, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Pose, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CameraInfo
from shape_msgs.msg import Plane
from auro_interfaces.msg import StringWithPose, Item, ItemInfo
from std_msgs.msg import Header
from vision_msgs.msg import Point2D
from assessment_interfaces.msg import ItemList
from auro_interfaces.srv import ItemRequest
from builtin_interfaces.msg import Time

from ipm_library.ipm import IPM
from ipm_library.exceptions import NoIntersectionError

from tf_transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
import angles

from enum import Enum
import random
import math

CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640


LINEAR_VELOCITY  = 0.3
ANGULAR_VELOCITY = 0.5

TURN_LEFT = 1
TURN_RIGHT = -1

SCAN_THRESHOLD = 0.5
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

class State(Enum):
    EXPLORING = 0
    RECORDING = 1
    COLLECTING = 2
    RETURNING = 3

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.state = State.EXPLORING
        self.pose = Pose()
        self.yaw = 0.0
        self.previous_yaw = 0.0
        self.turn_angle = 0.0
        self.turn_direction = TURN_LEFT
        self.scan_triggered = [False] * 4
        self.items = ItemList() # TODO Look into this as a datastructure, we might want a quicker way to find the closest item

        self.camera_info = CameraInfo(
            header=Header(frame_id='camera_rgb_optical_frame'),
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
            k=[530.4669406576809, 0.0, 320.5, 0.0, 530.4669406576809, 240.5, 0.0, 0.0, 1.0],
            d=[0.0, 0.0, 0.0, 0.0, 0.0],
            distortion_model="plumb_bob"
        )
        
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
        self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)

        self.item_subscriber = self.create_subscription(
            ItemList,
            '/robot1/items',
            self.item_callback,
            10, callback_group=timer_callback_group
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10, callback_group=timer_callback_group)
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value, callback_group=timer_callback_group)

        self.item_info_subscriber = self.create_subscription(
            ItemInfo,
            "item_info",
            self.item_info_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=3
            ),
            callback_group=timer_callback_group
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.item_info_publisher = self.create_publisher(ItemInfo, 'item_info', 10)

        self.point_pub = self.create_publisher(PointStamped, "ipm_point", 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.ipm = IPM(self.tf_buffer, self.camera_info, distortion=True)

        self.plane = Plane()
        self.plane.coef = [0.0, 0.0, 1.0, -0.01]

    def item_callback(self, msg):
        self.items = msg
        time = Time()
        self.get_logger().info(f"{len(self.items.data)} items currently visible")
        for item in self.items.data:

            point = Point2D(
                x = CAMERA_WIDTH/2 - item.x,
                y = CAMERA_HEIGHT/2 - item.y
            )

            self.get_logger().info(f"Image coords - x: {point.x}, y: {point.y}")

            try:
                point = self.ipm.map_point(
                    self.plane,
                    point,
                    time,
                    plane_frame_id='camera_rgb_frame',
                    output_frame_id='base_footprint'
                )

                self.get_logger().info(f"World coords - x: {point.point.x}, y: {point.point.y}, z: {point.point.z}")
                self.point_pub.publish(point)
            except NoIntersectionError:
                self.get_logger().info("No intersection found")
            # estimated_distance = 32.4 * float(item.diameter) ** -0.75

            # relative_x = (estimated_distance * (item.x - CAMERA_WIDTH/2))/CAMERA_F
            # relative_y = (estimated_distance * (item.y - CAMERA_HEIGHT/2))/CAMERA_F
            
            # self.get_logger().info(f"x: {relative_x}, y: {relative_y}, z: {estimated_distance}")

    def item_info_callback(self, msg):
        self.get_logger().info(f"x: {msg.x}, y: {msg.y}")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw
        self.get_logger().info(f"Yaw: {self.yaw}")

    def scan_callback(self, msg):
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30]
        left_ranges  = msg.ranges[31:90]
        back_ranges  = msg.ranges[91:270]
        right_ranges = msg.ranges[271:330]

        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD

    def control_loop(self):
        match self.state:
            case State.EXPLORING:
                self.get_logger().info(f"Exploring")
            case State.RECORDING:
                self.get_logger().info(f"Recording")

            case State.COLLECTING:
                self.get_logger().info(f"Collecting")
                if len(self.items.data) == 0:
                    self.previous_pose = self.pose
                    self.state = State.FORWARD
                    return
                
                item = self.items.data[0]

                # Obtained by curve fitting from experimental runs.
                estimated_distance = 32.4 * float(item.diameter) ** -0.75 #69.0 * float(item.diameter) ** -0.89

                self.get_logger().info(f'Estimated distance {estimated_distance}')

                if estimated_distance <= 0.35:
                    rqt = ItemRequest.Request()
                    rqt.robot_id = self.robot_id
                    try:
                        future = self.pick_up_service.call_async(rqt)
                        self.executor.spin_until_future_complete(future)
                        response = future.result()
                        if response.success:
                            self.get_logger().info('Item picked up.')
                            self.state = State.FORWARD
                            self.items.data = []
                        else:
                            self.get_logger().info('Unable to pick up item: ' + response.message)
                    except Exception as e:
                        self.get_logger().info('Exception ' + e)   

                msg = Twist()
                msg.linear.x = 0.25 * estimated_distance
                msg.angular.z = item.x / 320.0
                self.cmd_vel_publisher.publish(msg)
            case State.RETURNING:
                self.get_logger().info(f"Returning")
    def destroy_node(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Stopping: {msg}")
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()