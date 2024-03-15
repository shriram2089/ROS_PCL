#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import pcl
import numpy as np

def process_point_cloud(msg):
    # Convert ROS PointCloud2 to PCL PointCloud
    cloud = ros_to_pcl(msg)
    # Do something with the point cloud (e.g., process, analyze, etc.)
    # For demonstration, print the size of the point cloud
    print("Received PointCloud with {} points".format(cloud.size))

def ros_to_pcl(ros_cloud):
    arr = ros_cloud.data
    pcl_data = np.fromstring(arr, np.float32)
    pcl_data = pcl_data.reshape((ros_cloud.height * ros_cloud.width, 3))
    cloud = pcl.PointCloud()
    cloud.from_array(pcl_data)
    return cloud

def point_cloud_subscriber():
    rospy.init_node('point_cloud_subscriber', anonymous=True)
    rospy.Subscriber('point_cloud', PointCloud2, process_point_cloud)
    rospy.spin()

if __name__ == '__main__':
    point_cloud_subscriber()
