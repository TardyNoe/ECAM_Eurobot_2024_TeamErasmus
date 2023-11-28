#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time
from queue import PriorityQueue
import heapq

robot_position = (0, 0)
pose_pub = None

bridge = CvBridge()
cam_image = None
color_data = None


def a_star_search(grid, start, end):
    """
    A* search algorithm for pathfinding on a grid.

    :param grid: A 2D numpy array representing the grid (0 for open space, 255 for walls)
    :param start: A tuple (x, y) representing the start position
    :param end: A tuple (x, y) representing the end position
    :return: A list of tuples representing the path from start to end
    """

    def heuristic(a, b):
        """Calculate the Manhattan distance between two points."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def neighbors(pos):
        """Get the neighbors of a given position."""
        x, y = pos
        for nx, ny in [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]:
            if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and grid[nx][ny] != 255:
                yield (nx, ny)

    # Check if start or end is in a wall
    if grid[start] == 255 or grid[end] == 255:
        raise ValueError("Start or end point is in a wall")

    # Check if start or end is out of grid bounds
    if not (0 <= start[0] < grid.shape[0] and 0 <= start[1] < grid.shape[1]) or \
       not (0 <= end[0] < grid.shape[0] and 0 <= end[1] < grid.shape[1]):
        raise ValueError("Start or end point is out of grid bounds")

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for neighbor in neighbors(current):
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    raise ValueError("No path found from start to end")

def color_callback(data):
    global color_data
    color_data = data.data
def position_callback(pose_stamped):
    global robot_position
    robot_position = (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
def cam_callback(data):
    global cam_image
    cam_image = bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
def moving_average(points, window_size):
    # Separate the x and y coordinates
    x = points[:, 0]
    y = points[:, 1]
    x_smooth = np.convolve(x, np.ones(window_size), 'valid') / window_size
    y_smooth = np.convolve(y, np.ones(window_size), 'valid') / window_size

    return np.column_stack((x_smooth, y_smooth))
def send_to_robot(pose_pub):
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    goal.pose.position.x = robot_position[0]
    goal.pose.position.y = robot_position[1]
    goal.pose.orientation.w = 1.0  # Assuming the robot only needs x, y coordinate
    pose_pub.publish(goal)
    
def findEndPoint(mask, Team):
    img = mask
    mask1 = np.zeros((60,40),dtype=np.uint8)
    mask2 = np.zeros((60,40),dtype=np.uint8)
    mask3 = np.zeros((60,40),dtype=np.uint8)

    if Team:
        cv2.rectangle(mask1, (0, 0), (9, 9), 255, -1)
        cv2.rectangle(mask2, (40, 0), (30, 9), 255, -1)
        cv2.rectangle(mask3, (15, 60), (24, 50), 255, -1)
    else:
        cv2.rectangle(mask1, (0, 50), (9, 60), 255, -1)
        cv2.rectangle(mask2, (40, 60), (31, 51), 255, -1)
        cv2.rectangle(mask3, (15, 0), (24, 9), 255, -1)
    average_color1 = np.mean(img[mask1 < 250])
    average_color2 = np.mean(img[mask2 < 250])
    average_color3 = np.mean(img[mask3 < 250])
    max_average = min(average_color1, average_color2, average_color3)
    #print(average_color1,average_color2,average_color3)
    if Team:
        if max_average == average_color1:
            center = (46, 46)
        elif max_average == average_color2:
            center = (354, 46)
        else:
            center = (200, 554)
    else:
        if max_average == average_color1:
            center = (46, 554)
        elif max_average == average_color2:
            center = (354, 554)
        else:
            center = (200, 46)
    print(center)
    return center

        
oldpath = None
def main():
    global cam_image
    global pose_pub
    global oldpath
    # Initialize the ROS node
    rospy.init_node('image_subscriber_node', anonymous=True)
    rospy.Subscriber("mask", Image, cam_callback)
    rospy.Subscriber('/Pami', PoseStamped, position_callback)
    rospy.Subscriber('/color', Bool, color_callback)
    pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    path_publisher = rospy.Publisher('/path', Path, queue_size=10)

    # Main loop
    while not rospy.is_shutdown():
        try:
            kernel = np.ones((5,5), np.uint8)
            r = rospy.Rate(10)
            result = cam_image
            start_point = (39,30)
            end_point = (25,59)
            end_pointB = findEndPoint(cam_image,0)
            end_point = (int(end_pointB[0]/10),int(end_pointB[1]/10))
            cv2.rectangle(result, (0,0), (39,59), 255, 1) 
            result = cv2.circle(result,start_point,5, 0, -1) 
            result = cv2.circle(result,end_point,5, 0, -1) 
            pathc = a_star_search(result, (start_point[1],start_point[0]), (end_point[1],end_point[0]))
            smoothed_points = moving_average(np.array(pathc), 2)
            path = Path()
            path.header.frame_id = "map"  # Change as per your frame
            path.header.stamp = rospy.Time.now()
            for p in smoothed_points:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.y = (p[0] - 30) / 20
                pose.pose.position.x = -(p[1] - 40) / 20
                pose.pose.orientation.w = 1.0 # Assuming a 2D path, set z to 0
                path.poses.append(pose)
            #path_publisher.publish(path)
            print("p")
            if len(pathc)>3:
                while not rospy.is_shutdown():
                    path_publisher.publish(path)
                    print("publish")
            r.sleep()
        except Exception as e:
                print(e)
                #continue

if __name__ == '__main__':
    main()

