import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from solution_interfaces.srv import AddItem, GetItem
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import math
import random

class ItemTracker(Node):
    """
    Manages access to the list of tracked items item_list.

    Each item is stored as 2 elements in an array:

    - Point position
    - string colour
    
    """
    def __init__(self):
        super().__init__('item_tracker')

        self.item_list = []
        self.add_item = self.create_service(AddItem, '/add_item', self.add_item_callback)
        self.get_item = self.create_service(GetItem, '/get_item', self.get_item_callback)
        self.count_publisher = self.create_publisher(Int32, '/tracked_items', 10)
        self.timer = self.create_timer(0.1, self.publish_tracked_items)

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

        self.declare_parameter('num_robots', 0) 
        self.robots = self.get_parameter('num_robots').get_parameter_value().integer_value

    def add_item_callback(self, request, response):
        """
        Adds a new item if it is not already tracked.
        
        This is checked based on distance to an existing point
        and colour.
        """

        new_item_position = request.item_position
        new_colour = request.item_colour

        distance_threshold = 1

        found = False
        for i in range(len(self.item_list)):
            item_position, colour = self.item_list[i]
            if self.distance(item_position, new_item_position) < distance_threshold and colour == new_colour:
                found = True
                break
        if not found:
            self.item_list.append([new_item_position, new_colour])

        response.success = not found
        return response
    
    def get_item_callback(self, request, response):
        """
        Returns the closest item to a robot.
        """

        if request.robot_id == "robot1":
            maxX = 2.75
            maxY = 2.75
            minX = -3.75   
            minY = -2.75
            if self.robots == 2 or self.robots == 3:
                minY = 0
            if self.robots == 3:
                minX = 0
        elif request.robot_id == "robot2":
            maxX = 2.75
            minX = -3.75
            maxY = 0
            minY = -2.75
            if self.robots == 3:
                minX = 0
        else:
            maxX = 0
            minX = -3.75
            maxY = 2.75
            minY = -2.75
            
        if len(self.item_list) == 0:
        
        robot_position = request.robot_position
        
        i = self.item_list
        i.sort(key=lambda x: self.distance(robot_position, x[0]))

        assigned_item = None
        for item in i:
            if minX <= item[0].x <= maxX and minY <= item[0].y <= maxY:
                assigned_item = item
                break

        if assigned_item is None:
            response.success = False

            potentialZones = ["Purple", "Cyan", "Green", "Pink"]
            if request.robot_id == "robot1":
                if self.robots == 2:
                    potentialZones = ["Cyan", "Pink"]
                elif self.robots == 3:
                    potentialZones = ["Pink"]
            elif request.robot_id == "robot2":
                if self.robots == 2:
                    potentialZones = ["Purple", "Green"]
                elif self.robots == 3:
                    potentialZones = ["Green"]
            elif request.robot_id == "robot3":
                potentialZones = ["Cyan", "Purple"]
            response.item_pose_stamped = self.zone_locations[random.choice(potentialZones)]
            return response

        self.item_list.remove(assigned_item)
        item = assigned_item[0]
        response.item_pose_stamped = PoseStamped()
        response.item_pose_stamped.header = Header(frame_id="map")
        response.item_pose_stamped.pose.position.x = item.x
        response.item_pose_stamped.pose.position.y = item.y
        response.item_pose_stamped.pose.position.z = item.z

        response.success = True
        return response
    
    def publish_tracked_items(self):
        """
        Returns how many items are currently tracked.
        """
        msg = Int32()
        msg.data = len(self.item_list)

        self.count_publisher.publish(msg)

    def distance(self, point1, point2): 
        """
        Calculates the euclidean distance between 2 points
        """
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)

def main(args=None): 
    rclpy.init(args=args)
    node = ItemTracker()
    rclpy.spin(node)
    rclpy.shutdown() 
    
if __name__ == '__main__':
    main()