import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSPresetProfiles, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import Twist, Pose, PointStamped, Point, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CameraInfo
from shape_msgs.msg import Plane
from auro_interfaces.msg import StringWithPose, Item, ItemInfo
from std_msgs.msg import Header
from vision_msgs.msg import Point2D
from assessment_interfaces.msg import ItemList
from auro_interfaces.srv import ItemRequest
from builtin_interfaces.msg import Time

from tf_transformations import euler_from_quaternion, quaternion_matrix
import tf2_ros
import numpy as np
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
    NAVIGATING = 1
    BACKING_UP = 2
    COLLECTING = 3
    RETURNING = 4

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
        self.item_list = []
        self.tracking_item = None
        self.collecting_item = None
        self.carried_item = None
        self.robot_id = "robot1"
        self.zone_assignments = {"Purple" : None, "Cyan" : None, "Green" : None, "Pink" : None}
        zone_header = Header(frame_id="odom")
        purple_pose = PoseStamped()
        purple_pose.header = zone_header
        purple_pose.pose.position.x=0.0
        purple_pose.pose.position.y=-2.5
        cyan_pose = PoseStamped()
        cyan_pose.header = zone_header
        cyan_pose.pose.position.x=0.0 #
        cyan_pose.pose.position.y=2.5 #
        green_pose = PoseStamped()
        green_pose.header = zone_header
        green_pose.pose.position.x=6.0
        green_pose.pose.position.y=-2.5
        pink_pose = PoseStamped()
        pink_pose.header = zone_header
        pink_pose.pose.position.x=6.0
        pink_pose.pose.position.y=2.5
        self.zone_locations = {"Purple" : purple_pose, 
                                "Cyan" : cyan_pose, 
                                "Green" : green_pose, 
                                "Pink" : pink_pose}
        self.double_zone_assigned = False
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
            '/robot1/odom',
            self.odom_callback,
            10, callback_group=timer_callback_group)
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
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

        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/robot1/initialpose', 10) 
        initial_pose = PoseWithCovarianceStamped() 
        initial_pose.header.frame_id = 'map' 
        initial_pose.pose.pose.position.x = -3.5
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.orientation.w = 1.0

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose = initial_pose.pose.pose

        self.initial_pose_publisher.publish(initial_pose)
        self.navigator = BasicNavigator()
        self.navigator.setInitialPose(pose_stamped)

    def item_callback(self, msg):
        self.items = msg
        time = Time()
        #self.get_logger().info(f"{len(self.items.data)} items currently visible")
        for item in self.items.data:
            
            # We determine distance from the pixel size of the item
            # When the item is at an extreme of the image, this is warped
            # This can lead to an inaccuracte estimate of the point, so we should skip
            if abs(item.x) > 160:
                continue
            
            relativePoint = self.itemToWorldCoords(item)
            
            #self.get_logger().info(f"World coords - x: {x}, y: {y}, z: {estimated_distance}")
            estimated_distance = 32.4 * float(item.diameter) ** -0.75
            self.add_item(relativePoint.point, estimated_distance)
            #self.get_logger().info(f"Items in Item List: {len(self.item_list)}")
            # for item, observed_distance in self.item_list:
            #     publish_point = PointStamped()
            #     publish_point.header = Header(stamp=time, frame_id="odom")
            #     publish_point.point = item
            #     self.point_pub.publish(publish_point)

    def distance(self, point1, point2): 
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)

    def itemToWorldCoords(self, item):
        time = Time()
        point = Point2D(
                x = CAMERA_WIDTH/2 - item.x,
                y = CAMERA_HEIGHT/2 - item.y
            )

        estimated_distance = 32.4 * float(item.diameter) ** -0.75
        x = (point.x - self.camera_info.k[2]) * estimated_distance / self.camera_info.k[0]
        y = (point.y - self.camera_info.k[5]) * estimated_distance / self.camera_info.k[4]

        point_3D = Point()
        point_3D.x = estimated_distance
        point_3D.y = -x
        point_3D.z = y
        header = Header(stamp=time, frame_id="base_footprint")
        point_stamped = PointStamped(header=header, point=point_3D)
        relativePoint = self.tf_buffer.transform(point_stamped, "odom")
        return relativePoint
        
    def add_item(self, new_item, estimated_distance):
        distance_threshold = 1

        found = False
        for i in range(len(self.item_list)):
            item, observed_distance = self.item_list[i]
            if self.distance(item, new_item) < distance_threshold:
                found = True
                self.item_list[i][0].x = new_item.x
                self.item_list[i][0].y = new_item.y
                self.item_list[i][0].z = new_item.z
                self.item_list[i][1] = estimated_distance
        if not found:
            self.item_list.append([new_item, estimated_distance])

    def item_info_callback(self, msg):
        self.get_logger().info(f"x: {msg.x}, y: {msg.y}")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw

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
                # State for exploring to find items when none have been found
                #self.get_logger().info(f"Exploring")
                if len(self.item_list) > 0:
                    item, observed_distance = self.item_list[0]
                    self.tracking_item = item

                    # Obtained by curve fitting from experimental runs.
                    estimated_distance = self.distance(self.pose.position, item)
                    goalPose = PoseStamped()
                    goalPose.header = Header(frame_id="odom")
                    goalPose.pose.position.x = item.x
                    goalPose.pose.position.y = item.y
                    goalPose.pose.position.z = item.z

                    try:
                        result = self.navigator.goToPose(goalPose)
                        if not result:
                            raise Exception("Goal rejected")
                        self.state = State.NAVIGATING
                    except Exception as e:
                        self.item_list.pop(0)
            case State.NAVIGATING:
                # State for when navigating using nav2 to the approximate location of an item
                #self.get_logger().info(f"Navigating")

                if self.navigator.isTaskComplete():
                    self.navigator.backup(backup_dist=0.1)
                    self.state = State.BACKING_UP
            case State.BACKING_UP:
                #self.get_logger().info(f"Backing up")
                if self.navigator.isTaskComplete():
                    if len(self.items.data) == 0:
                        self.state = State.EXPLORING
                    else:
                        self.state = State.COLLECTING
            case State.COLLECTING:
                #self.get_logger().info(f"Collecting")
                
                i = self.items.data
                i.sort(reverse=True, key = lambda x: x.diameter)
                self.collecting_item = i[0]
                # Obtained by curve fitting from experimental runs.
                estimated_distance = 32.4 * float(self.collecting_item.diameter) ** -0.75 #69.0 * float(item.diameter) ** -0.89

                if estimated_distance <= 0.35:
                    rqt = ItemRequest.Request()
                    rqt.robot_id = self.robot_id
                    try:
                        future = self.pick_up_service.call_async(rqt)
                        self.executor.spin_until_future_complete(future)
                        response = future.result()
                        if response.success:
                            self.get_logger().info('Item picked up.')
                            self.carried_item = self.collecting_item
                            self.setZoneGoal()
                            self.state = State.RETURNING
                        else:
                            self.get_logger().info('Unable to pick up item: ' + response.message)
                    except Exception as e:
                        self.get_logger().info('Exception ' + str(e))   

                msg = Twist()
                msg.linear.x = 0.25 * estimated_distance
                msg.angular.z = self.collecting_item.x / 320.0
                self.cmd_vel_publisher.publish(msg)
            case State.RETURNING:
                #self.get_logger().info(f"Returning")
                #self.get_logger().info(f"Item colour is {self.carried_item.colour}")

                if self.navigator.isTaskComplete():
                    rqt = ItemRequest.Request()
                    rqt.robot_id = self.robot_id
                    try:
                        future = self.offload_service.call_async(rqt)
                        self.executor.spin_until_future_complete(future)
                        response = future.result()
                        if response.success:
                            self.get_logger().info('Item returned up.')
                            self.state = State.EXPLORING
                        else:
                            self.get_logger().info('Unable to pick up item: ' + response.message)
                    except Exception as e:
                        self.get_logger().info('Exception ' + str(e))


    def setZoneGoal(self):
        colour = self.carried_item.colour
        assigned_dest = None
        assigned_zone = None
        distances = [[zone, position, self.distance(self.pose.position, position.pose.position)] for zone, position in self.zone_locations.items()]
        distances.sort(key=lambda x:x[2])
        colour_count = 0

        for c in self.zone_assignments.values():
            if c == colour:
                colour_count += 1

        for zone, position, distance in distances:
            if self.zone_assignments[zone] == colour:
                assigned_dest = position
                assigned_zone = zone
                break
            elif self.zone_assignments[zone] == None:
                if colour_count == 0:
                    assigned_dest = position
                    assigned_zone = zone
                    break
                elif colour_count == 1 and not self.double_zone_assigned:
                    self.double_zone_assigned = True
                    assigned_dest = position
                    assigned_zone = zone
                    break
        self.get_logger().info(f"Assigned zone: {assigned_zone}")
        self.navigator.goToPose(assigned_dest)


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