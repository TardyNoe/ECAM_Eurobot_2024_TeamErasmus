#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

path_points = []  # Global variable to store path points
robot_position = (0, 0)  # Global variable to store the robot's position

def distance(point1, point2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def position_callback(pose_stamped):
    """Callback function for the PoseStamped topic."""
    global robot_position
    robot_position = (pose_stamped.pose.position.x, pose_stamped.pose.position.y)

def send_to_robot(pose_pub, point):
    """Send a point to the robot."""
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    goal.pose.position.x = point[0]
    goal.pose.position.y = point[1]
    goal.pose.orientation.w = 1.0  # Assuming the robot only needs x, y coordinate
    print(point[0],point[1])
    pose_pub.publish(goal)

def path_callback(msg):
    """Callback function for /path topic."""
    global path_points
    path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

def follow_path(pose_pub, parent_frame, threshold=0.05):
    """Make the robot follow the path from the nearest point."""
    if not path_points:
        rospy.loginfo("No path points received.")
        return

    robot_pos = robot_position
    if robot_pos is None:
        return
    print("calcul")
    nearest_point = path_points[0]
    min_distance = distance(robot_pos, nearest_point)
    for point in path_points:
        dist = distance(robot_pos, point)
        if dist < min_distance:
            min_distance = dist
            nearest_point = point

    start_index = path_points.index(nearest_point)
    booltest = False
    for point in path_points[start_index:]:
        send_to_robot(pose_pub, point)
        while True:
            current_pos = robot_position
            if current_pos is None or distance(current_pos, point) < threshold:
                if booltest == True:
                    return
                booltest = True
                break

def main():
    rospy.init_node('robot_path_follower')
    pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # Subscribe to the /path topic
    rospy.Subscriber('/path', Path, path_callback)
    # Subscribe to the PoseStamped topic
    rospy.Subscriber('/Pami/LocalPosition', PoseStamped, position_callback)

    parent_frame = 'map'  # Replace with your robot's base frame

    while not rospy.is_shutdown():
        follow_path(pose_pub, parent_frame)

if __name__ == '__main__':
    main()

