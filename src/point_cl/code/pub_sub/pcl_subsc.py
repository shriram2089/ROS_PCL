#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np

class PointCloudSubscriberNode():
    def __init__(self):
        rospy.init_node('pointcloud_subscriber_node', anonymous=True)
        rospy.Subscriber('pointcloud_topic', PointCloud2, self.pointcloud_callback)
        rospy.loginfo("Subscribed to pointcloud_topic")

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 message to numpy array
        print(1)
        rospy.loginfo("Received message of type: %s", type(msg))

        pointcloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)  # Assuming XYZI format
        self.received_pointclouds.append(pointcloud)  # Store the received point cloud
        rospy.loginfo("Received PointCloud2 message")
    '''
    def print_all_pointclouds(self):
        rospy.loginfo("Printing all received PointCloud2 messages:")
        for i, pointcloud in enumerate(self.received_pointclouds):
            rospy.loginfo("PointCloud %d:\n%s", i, repr(pointcloud))
            '''
       # self.run_pointnet(pointcloud)

    def run_pointnet(self, pointcloud):
        # Implement your PointNet model here
        # Example:
        # - Load the pre-trained model
        # - Feed the point cloud data into the model
        # - Get predictions or process the output
        
        # Example:
        # result = pointnet_model(pointcloud)
        # print(result)
        return None

def main():
    PointCloudSubscriberNode()
    rospy.spin()

if __name__ == '__main__':
    main()
