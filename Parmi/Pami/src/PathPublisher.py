#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def create_path_message(points):
    path = Path()
    path.header.frame_id = "map"  # Change as per your frame
    path.header.stamp = rospy.Time.now()

    for point in points:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.w = 1.0  # Assuming no orientation change
        path.poses.append(pose)

    return path

def path_publisher():
    rospy.init_node('path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/path', Path, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz, adjust as needed
    points = [[0, 0], [1.5,1.2],  [1.5,-1.2]]  # Example list of points

    while not rospy.is_shutdown():
        path_msg = create_path_message(points)
        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass

