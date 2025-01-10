import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import Twist, Pose, PointStamped, Point, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header, Int32
from vision_msgs.msg import Point2D
from assessment_interfaces.msg import ItemList, ItemHolders
from auro_interfaces.srv import ItemRequest
from solution_interfaces.srv import SetZoneGoal, AddItem, GetItem
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory
import os

import tf2_ros
from tf2_geometry_msgs import PointStamped

from enum import Enum
import math

CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640

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
        self.tracked_items = 0

        self.tracking_item = None
        self.collecting_item = None
        self.carried_item_colour = None
        self.robot_id = self.get_namespace()[1:]

        package_share_directory = get_package_share_directory('solution')
        self.behaviour_tree = os.path.join(package_share_directory, 'config', 'dynamic_replanning.xml') 

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
        self.set_zone_service = self.create_client(SetZoneGoal, '/set_zone_goal', callback_group=client_callback_group)
        self.add_item_client = self.create_client(AddItem, '/add_item', callback_group=client_callback_group)
        self.get_item_client = self.create_client(GetItem, '/get_item', callback_group=client_callback_group)
        
        self.item_subscriber = self.create_subscription(
            ItemList,
            f'/{self.robot_id}/items',
            self.item_callback,
            10, callback_group=timer_callback_group
        )

        self.tracked_items_subscriber = self.create_subscription(
            Int32,
            '/tracked_items',
            self.tracked_items_callback,
            10, callback_group=timer_callback_group
        )

        self.carried_item_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.carried_item_callback,
            10, callback_group=timer_callback_group
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self.odom_callback,
            10, callback_group=timer_callback_group)

        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, f'/{self.robot_id}/initialpose', 10) 

        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0]) 
        initial_x, initial_y, initial_yaw= self.get_parameter('initial_pose').get_parameter_value().double_array_value
        
        initial_pose = PoseWithCovarianceStamped() 
        initial_pose.header.frame_id = 'map' 
        initial_pose.pose.pose.position.x = initial_x
        initial_pose.pose.pose.position.y = initial_y
        initial_pose.pose.pose.orientation.w = 1.0

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose = initial_pose.pose.pose
        self.initial_pose_publisher.publish(initial_pose)

        self.navigator = BasicNavigator(namespace=self.get_namespace())
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
                self.add_item(relativePoint.point, item.colour)
            
    def tracked_items_callback(self, msg):
        """
        Stores the amount of items tracked in ItemTracker
        """
        self.tracked_items = msg.data

    def carried_item_callback(self, msg):
        """
        Processes data from the item_holders topic.
        
        Used to identify the colour of a carried item.
        """
        self.carried_item_colour = msg.data[0].item_colour

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
                if self.tracked_items > 0:
                    
                    response = self.get_item()
                    
                    if response:
                        result = self.navigator.goToPose(response.item_pose_stamped, behavior_tree=self.behaviour_tree)
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
                            self.set_zone_goal()
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
                            self.set_zone_goal()
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


    def set_zone_goal(self):
        rqt = SetZoneGoal.Request()
        rqt.carried_item_colour = self.carried_item_colour
        rqt.robot_pose = PoseStamped()
        rqt.robot_pose.pose = self.pose
        rqt.robot_pose.header = Header(frame_id="odom")
        try:
            future = self.set_zone_service.call_async(rqt)
            self.executor.spin_until_future_complete(future)
            response = future.result()
            if response.success:
                # If we succeed, set a zone as the goal
                self.get_logger().info(f'Assigned zone: {response.assigned_dest}')
                self.navigator.goToPose(response.assigned_dest, behavior_tree=self.behaviour_tree)
                self.state = State.RETURNING
            else:
                # If we fail, explore
                self.get_logger().info('Unable to set zone goal')
                self.state = State.EXPLORING
        except Exception as e:
            self.get_logger().info('Exception ' + str(e))

    def add_item(self, item_position, item_colour):
        rqt = AddItem.Request()
        rqt.item_position = item_position
        rqt.item_colour = item_colour
        try:
            future = self.add_item_client.call_async(rqt)
            self.executor.spin_until_future_complete(future)
        except Exception as e:
            self.get_logger().info('Exception ' + str(e))

    def get_item(self):
        rqt = GetItem.Request()
        rqt.robot_position = self.pose.position
        try:
            future = self.get_item_client.call_async(rqt)
            self.executor.spin_until_future_complete(future)
            response = future.result()
            if response.success:
                self.get_logger().info(f'Item is of type: {type(response.item_pose_stamped)}')
                return response
            else:
                # If we fail, explore
                self.get_logger().info('Get Item failed, likely no tracked items')
                self.state = State.EXPLORING
        except Exception as e:
            self.get_logger().info('Exception ' + str(e))

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