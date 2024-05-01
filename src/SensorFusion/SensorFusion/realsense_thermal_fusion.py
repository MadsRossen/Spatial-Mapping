#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

def callback(thermal_image, point_cloud):
    """
    Callback function to process synchronized messages.
    This function is called when synchronized thermal image and point cloud messages are received.
    """
    # Process synchronized messages
    rospy.loginfo("Received synchronized messages with timestamps: {} & {}".format(
        thermal_image.header.stamp, point_cloud.header.stamp))
    # Additional processing like transformation and projection can be added here

def main():
    """
    Main function to initialize the node and set up message filtering.
    """
    rospy.init_node('sensor_fusion_sync', anonymous=True)

    # Create subscribers using message_filters
    thermal_sub = message_filters.Subscriber('/thermal_camera/image_raw', Image)
    pc_sub = message_filters.Subscriber('/realsense/depth/points', PointCloud2)

    # Create a TimeSynchronizer
    ts = message_filters.TimeSynchronizer([thermal_sub, pc_sub], queue_size=10)
    ts.registerCallback(callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()