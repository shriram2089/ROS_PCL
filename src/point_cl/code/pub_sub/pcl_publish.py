#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import rosbag

class PointCloudPublisherNode():
    def __init__(self):
        rospy.init_node('pointcloud_publisher_node', anonymous=True)
        self.publisher = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=10)

       
        bag = rosbag.Bag('/home/zeus_007/task2_Ws/extracted.bag', 'r')

        #print(bag.read_messages())
        for topic, msg, t in bag.read_messages():
            
            self.publisher.publish(msg)
            rospy.loginfo("Published a PointCloud2 message")  
        bag.close()

def main():
    PointCloudPublisherNode()
    rospy.spin()

if __name__ == '__main__':
    main()
