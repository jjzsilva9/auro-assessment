import rclpy
from rclpy.node import Node
from solution_interfaces.srv import SetZoneGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf2_ros
from assessment_interfaces.msg import ItemHolders
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math

class ZoneGoalService(Node):
    """
    Manages the assigned zones when robots pick up an item.
    """
    def __init__(self):
        super().__init__('zone_goal_service')
        
        timer_callback_group = ReentrantCallbackGroup()
        service_callback_group = ReentrantCallbackGroup()
        self.carried_items_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.carried_item_callback,
            10, callback_group=timer_callback_group
        )
        self.srv = self.create_service(SetZoneGoal, '/set_zone_goal', self.set_zone_goal_callback, callback_group=service_callback_group)

        zone_header = Header(frame_id="map")
        purple_pose = PoseStamped()
        purple_pose.header = zone_header
        purple_pose.pose.position.x=-3.5
        purple_pose.pose.position.y=-2.5
        cyan_pose = PoseStamped()
        cyan_pose.header = zone_header
        cyan_pose.pose.position.x=-3.5
        cyan_pose.pose.position.y=2.5
        green_pose = PoseStamped()
        green_pose.header = zone_header
        green_pose.pose.position.x=2.5
        green_pose.pose.position.y=-2.5
        pink_pose = PoseStamped()
        pink_pose.header = zone_header
        pink_pose.pose.position.x=2.5
        pink_pose.pose.position.y=2.5
        
        self.zone_locations = {"Purple" : purple_pose, "Cyan" : cyan_pose, "Green" : green_pose, "Pink" : pink_pose}
        self.zone_assignments = {"Purple" : None, "Cyan" : None, "Green" : None, "Pink" : None}
        self.double_zone_assigned = False

        self.carried_items = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

    def carried_item_callback(self, msg):
        """
        Processes data from the /item_holders topic to see what items
        are being carried.
        """
        for robot in msg.data:
            self.carried_items[robot.robot_id] = robot.item_colour

    def wait_for_carried_item(self, robot_id):
        """
        Waits for colour data to be published when an item is picked up,
        called through the set_zone_goal service.
        """
        timeout = 5
        total_time = 0
        while self.carried_items[robot_id] == '' and total_time < timeout:
            time.sleep(0.1)
            total_time += 0.1
        if self.carried_items[robot_id] == '':
            raise Exception("No colour data published for carried item")

    def set_zone_goal_callback(self, request, response):
        """
        Provides an assigned zone when an item is picked up.
        
        Takes as input:
        
        - string robot_id
        - geometry_msgs/PoseStamped robot_pose
        
        returns:
        
        - bool success
        - geometry_msgs/PoseStamped assigned_dest
        """
        self.wait_for_carried_item(request.robot_id)
        colour = self.carried_items[request.robot_id]
        self.get_logger().info(f"Received request for item of colour: {colour}")
        
        robot_pose = request.robot_pose.pose
        
        assigned_dest = None
        assigned_zone = None

        # Sorts the zones on euclidean distance
        distances = [(zone, position, self.distance(robot_pose.position, position.pose.position)) for zone, position in self.zone_locations.items()]
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
                    self.zone_assignments[zone] = colour
                    break
                elif colour_count == 1 and not self.double_zone_assigned:
                    self.double_zone_assigned = True
                    assigned_dest = position
                    assigned_zone = zone
                    self.zone_assignments[zone] = colour
                    break
        
        self.get_logger().info(f"Assigned zone: {assigned_zone}")
        response.assigned_dest = assigned_dest
        response.success = True
        return response

    def distance(self, point1, point2): 
        """
        Calculates the euclidean distance between 2 points
        """
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)

def main(args=None): 
    rclpy.init(args=args)
    node = ZoneGoalService()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown() 
    
if __name__ == '__main__':
    main()