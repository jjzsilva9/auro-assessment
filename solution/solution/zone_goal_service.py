import sys

import rclpy
from rclpy.node import Node
from solution_interfaces.srv import SetZoneGoal
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from std_msgs.msg import Header
import tf2_ros
from tf2_geometry_msgs import PointStamped

import math

class ZoneGoalService(Node):
    def __init__(self):
        super().__init__('zone_goal_service')
        self.srv = self.create_service(SetZoneGoal, '/set_zone_goal', self.set_zone_goal_callback)

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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

    def set_zone_goal_callback(self, request, response):

        
        self.get_logger().info(f'Purple: {self.zone_assignments["Purple"]}, Cyan: {self.zone_assignments["Cyan"]}, Green: {self.zone_assignments["Green"]}, Pink: {self.zone_assignments["Pink"]}')
        colour = request.carried_item_colour
        
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
    rclpy.spin(node)
    rclpy.shutdown() 
    
if __name__ == '__main__':
    main()