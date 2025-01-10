import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from solution_interfaces.srv import AddItem, GetItem
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import math

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

    def add_item_callback(self, request, response):
        """
        Adds a new item if it is not already tracked.
        
        This is checked based on distance to an existing point
        and colour.
        """
        self.get_logger().info("Received request")
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
        self.get_logger().info("Received request")

        if len(self.item_list) == 0:
            response.success = False
            return response
        
        robot_position = request.robot_position
        
        i = self.item_list
        i.sort(key=lambda x: self.distance(robot_position, x[0]))
        item = i[0][0]

        self.item_list.remove(i[0])

        response.item_pose_stamped = PoseStamped()
        response.item_pose_stamped.header = Header(frame_id="odom")
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