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
        self.item_list = []
        self.tracking_item = None
        self.collecting_item = None
        self.carried_item_colour = None
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
        green_pose.pose.position.x=5.5
        green_pose.pose.position.y=-2.5
        pink_pose = PoseStamped()
        pink_pose.header = zone_header
        pink_pose.pose.position.x=5.0
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

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        #self.point_pub = self.create_publisher(PointStamped, "ipm_point", 10)

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
        """
        Processes data from the items topic.

        Converts each item to world coordinates and adds it to the item list.
        """
        self.items = msg
        
        for item in self.items.data:
            # We determine distance from the pixel size of the item
            # When the item is at an extreme of the image, this is warped
            # This can lead to an inaccuracte estimate of the point, so we should skip
            if abs(item.x) > 160:
                continue
            
            relativePoint = self.itemToWorldCoords(item)
            
            # If the estimated coordinates are out of map bounds we should skip
            if 0 <= relativePoint.point.x <= 6 and -2.5 <= relativePoint.point.y <= 2.5:
                estimated_distance = 32.4 * float(item.diameter) ** -0.75
                self.add_item(relativePoint.point, estimated_distance, item.colour)

    def distance(self, point1, point2): 
        """
        Calculates the euclidean distance between 2 points
        """
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)

    def itemToWorldCoords(self, item):
        """
        Converts from a 2D point on the camera image to a 3D world coordinate.
        
        This is achieved using the intrinsic properties of the camera and inverting
        the perspective projection.
        """
        time = Time()
        point = Point2D(
                x = CAMERA_WIDTH/2 - item.x,
                y = CAMERA_HEIGHT/2 - item.y
            )

        # We derive the distance from the diameter of the item
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
        
    def add_item(self, new_item, estimated_distance, colour):
        """
        Adds an item and it's colour to the item list.

        Also checks if it's location is close to an item already in the list,
        and if so, doesn't add it.
        """
        distance_threshold = 1

        found = False
        for i in range(len(self.item_list)):
            item, prev_colour = self.item_list[i]
            if self.distance(item, new_item) < distance_threshold and prev_colour == colour:
                found = True
                break
        if not found:
            self.item_list.append([new_item, colour])

    def odom_callback(self, msg):
        """
        Sets the pose from the odom topic
        """
        self.pose = msg.pose.pose

    def control_loop(self):
        """
        The main control loop of the robot.
        """
        match self.state:
            case State.EXPLORING:
                # State for exploring to find items when none have been found

                # If an item is found, set the closest one as the goal destination
                if len(self.item_list) > 0:
                    i = self.item_list
                    i.sort(key=lambda x: self.distance(self.pose.position, x[0]))
                    item, carried_item_colour = i[0]

                    self.tracking_item = item
                    self.carried_item_colour = carried_item_colour
                    self.item_list.remove(i[0])

                    # Set the destination
                    estimated_distance = self.distance(self.pose.position, item)
                    goalPose = PoseStamped()
                    goalPose.header = Header(frame_id="odom")
                    goalPose.pose.position.x = item.x
                    goalPose.pose.position.y = item.y
                    goalPose.pose.position.z = item.z

                    # If the goal is successfully set, switch to navigating state
                    result = self.navigator.goToPose(goalPose)
                    if result:  
                        self.state = State.NAVIGATING
            case State.NAVIGATING:
                # State for navigating using nav2 to the approximate location of an item

                if self.navigator.isTaskComplete():
                    # Try to pickup
                    rqt = ItemRequest.Request()
                    rqt.robot_id = self.robot_id
                    try:
                        future = self.pick_up_service.call_async(rqt)
                        self.executor.spin_until_future_complete(future)
                        response = future.result()
                        if response.success:
                            self.get_logger().info('Item picked up.')
                            self.setZoneGoal()
                            self.state = State.RETURNING
                        else:
                            # If we fail to pick up, we back up slightly and then see if an item is visible
                            self.get_logger().info('Unable to pick up item: ' + response.message)
                            self.navigator.backup(backup_dist=0.1)
                            self.state = State.BACKING_UP
                    except Exception as e:
                        self.get_logger().info('Exception ' + str(e))
            case State.BACKING_UP:
                # State for backing up

                if self.navigator.isTaskComplete():
                    if len(self.items.data) == 0:
                        self.state = State.EXPLORING
                    else:
                        self.state = State.COLLECTING
            case State.COLLECTING:
                # State for collecting if the pickup from nav2 fails
                
                # We pick the closest item to collect
                i = self.items.data
                i.sort(reverse=True, key = lambda x: x.diameter)
                self.collecting_item = i[0]
                estimated_distance = 32.4 * float(self.collecting_item.diameter) ** -0.75 #69.0 * float(item.diameter) ** -0.89

                # Attempt to pickup if we are close enough
                if estimated_distance <= 0.35:
                    rqt = ItemRequest.Request()
                    rqt.robot_id = self.robot_id
                    try:
                        future = self.pick_up_service.call_async(rqt)
                        self.executor.spin_until_future_complete(future)
                        response = future.result()
                        if response.success:
                            # If we succeed, set a zone as the goal
                            self.get_logger().info('Item picked up.')
                            self.carried_item_colour = self.collecting_item.colour
                            self.setZoneGoal()
                            self.state = State.RETURNING
                        else:
                            # If we fail, explore
                            self.get_logger().info('Unable to pick up item: ' + response.message)
                            self.state = State.EXPLORING
                    except Exception as e:
                        self.get_logger().info('Exception ' + str(e))   

                msg = Twist()
                msg.linear.x = 0.25 * estimated_distance
                msg.angular.z = self.collecting_item.x / 320.0
                self.cmd_vel_publisher.publish(msg)
            case State.RETURNING:
                # State for returning to a zone if we are carrying an item

                # When we arrive at the zone, drop the item
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
                            # TODO Add a retry here, or handle if it actually isnt carrying an item
                            self.get_logger().info('Unable to offload item: ' + response.message)
                    except Exception as e:
                        self.get_logger().info('Exception ' + str(e))


    def setZoneGoal(self):
        """
        Sets the goal zone when an item has been picked up.
        
        Finds the closest zone that accepts the colour whilst allowing
        all colours to have a zone.
        """
        colour = self.carried_item_colour
        assigned_dest = None
        assigned_zone = None

        # Sorts the zones on euclidean distance
        distances = [[zone, position, self.distance(self.pose.position, position.pose.position)] for zone, position in self.zone_locations.items()]
        distances.sort(key=lambda x:x[2])
        
        # Counts the zone assignments of the colour
        colour_count = 0
        for c in self.zone_assignments.values():
            if c == colour:
                colour_count += 1

        # Finds the best fit zone
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
        
        assigned_dest = self.zone_locations["Green"]
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