<h1>Autonomous Retrieval and Collection Robots</h1>

This repository contains the code used for the coursework for Autonomous Robotics, a module I took in my final year of undergraduate studies at the University of York.
To read the full report you can click <a href="">here</a>.

This paper details the architecture and evaluation of an autonomous robotic system developed
to retrieve and deposit items efficiently. To achieve this, a deliberative architecture was used,
with reactive elements in navigation. The system developed is a decentralized homogenous
multi-robot system and includes explicit communication to aid in planning. Inspiration was
taken from microservice architecture, to maintain modularity and to facilitate efficient communication between robots.

<h2>Architecture</h2>

<img src=""></image>
The robot controller node implements the main control strategy using a finite state machine
(FSM). Nav2 also implements some of the control strategy which is implemented through a
behavior tree (BT). The robot controller node receives information about visible items through
the items topic from the item sensor node. It estimates the 3D coordinates of the item, and
uses the add item service provided by the item tracker node, which maintains a list of observed
items. The robot controller uses the get item service provided by the item tracker, to request
the position of an item to collect. This information is used to set a Nav2 goal, which plans
and follows a path to the item while avoiding obstacles.

This modular approach to item tracking and zone assignment works well, and enables intelligent coordination, preventing robots from trying to collect the same tracked item, and
allowing a robot to pursue an item that was added by another robot, reducing exploration
time. This system also enables easy implementation of other tasks while changing minimal
code, by changing logic in the item tracker or the zone goal service.

<h2>Control</h2>

The FSM consists of 3 states: exploring, navigating, and returning. The exploration state
takes place when there is not an item for a robot to collect, meaning the get item service
responds with a failure, but provides a location to navigate to (assigned dest), so that the
robot can hopefully see items on the way. The robot continually uses this service, and if it
responds with success, it cancels any current navigation, begins navigating to the assigned item,
and moves to the navigating state.

The navigating state does nothing until the navigation provided by Nav2 has finished, when
it attempts to pick up an item using the pick up item service. If this fails, it goes back to the
exploring state, but if it is successful, gets an assigned zone to drop the item at through the
set zone goal service and moves to the returning state.

The returning state, waits until navigation has finished, then attempts to offload the item
using the offload item service. Then it moves to the exploring state. The simplicity of this FSM
is a result of the microservice architecture, the main control logic is cleared due to offloading
other computation to services.

The state machine provided here describes the navigation subtree of the behavior tree used,
and omits the recovery subtree for clarity. This behavior tree is provided by Nav2 and is called
Navigate To Pose and Pause Near Goal-Obstacle.

This behavior tree implements replanning every second, and clears the global costmap in
the event of no available path. This dynamic replanning is helpful in a multi-robot scenario,
as it accounts for the other robots and prevents collisions. When near a goal, if the path is
significantly longer due to an obstacle, it waits. This works well if the obstacle is another robot,
which will move and clear the path.

<h2>Evaluation</h2>

<h3>Tracking Items</h3>
Adding an item to be tracked is an O(1) operation. Getting an item for a robot involves sorting
the whole list based on distance between the item and the robot. This results in a O(n log n)
operation, which might be performed very regularly when a robot is in the exploring state, as
the list needs to be resorted as the robot moves around. When multiple robots are spawned,
they can only collect items from different segments to spread them around the map, so this
could be used to prevent sorting irrelevant items by maintaining an item list for each segment.
Alternatively, a k-d tree could be used to more efficiently find the closest item to the robot,
which could be done in average O(log n).

<h3>Path Planning</h3>
The Nav2 behavior tree uses a grid based approach to planning, using a global costmap. The
Navfn planner is used, and uses the A* algorithm. With a larger map this can become inefficient,
and memory usage will increase exponentially with resolution. However in the case of this
scenario with a bounded map, it is suitable.

<h3>Localization</h3>
The system uses Adaptive Monte Carlo Localization (AMCL), which uses a particle filter to
keep track of the robot against a known map. The size of the particle set here trades off
accuracy with computational efficiency.
<br>
<br>
Overall, the system is highly efficient in item retrieval, and the effectiveness is not lost as the number of
robots increases. The location of items is determined by converting the 2D image coordinates
into 3D world coordinates using the intrinsic values of the camera and reversing the perspective
projection. This proves to be very powerful when combined with the item tracker, meaning
robots being in the Exploring state is extremely rare. This method does result in the tracking
of ghost items, but the system quickly recovers and is able to route the robot to a nearby item.
The benefit of being able to track items between the robots outweighs the impact of navigating
to a ghost item completely. This system also enables the usage of Nav2 for all navigation, which
enables more deliberative actions and collision avoidance. The system could however benefit
from a more low-level collision avoidance solution, such as implementing a collision monitor.
