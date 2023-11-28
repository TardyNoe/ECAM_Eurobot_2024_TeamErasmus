#!/usr/bin/env python
import rospy
import message_filters
from geometry_msgs.msg import PoseStamped

offset = (0, 0)

def callback(pami_msg, local_position_msg, pub):
    global offset
    local = pami_msg.pose.position
    camera = local_position_msg.pose.position
    offset = (local.x - camera.x, local.y - camera.y)
    orientation = local_position_msg.pose.orientation
    print("ok")
def callback_fusion(msg):
    global offset
    fusion_msg = PoseStamped()
    fusion_msg.header = msg.header
    camera = msg.pose.position
    fusion_msg.pose.position.x = camera.x + offset[0]
    fusion_msg.pose.position.y = camera.y + offset[1]
    fusion_msg.pose.position.z = 0
    fusion_msg.pose.orientation = msg.pose.orientation
    fusion_pub.publish(fusion_msg)
    #


if __name__ == '__main__':
    rospy.init_node('pose_sync_node', anonymous=True)

    # Subscribers
    pami_sub = message_filters.Subscriber('/Pami', PoseStamped)
    local_position_sub = message_filters.Subscriber('/Pami/LocalPosition', PoseStamped)

    # Publisher
    rospy.Subscriber('/Pami/LocalPosition', PoseStamped, callback_fusion)
    fusion_pub = rospy.Publisher('/Pami/fusion', PoseStamped, queue_size=10)

    # Synchronize the topics using ApproximateTimeSynchronizer
    ts = message_filters.ApproximateTimeSynchronizer([pami_sub, local_position_sub], 100, 0.1)
    ts.registerCallback(lambda pami, local_position: callback(pami, local_position, fusion_pub))

    rospy.spin()

