#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import pcl

def publish_point_cloud():
    rospy.init_node('point_cloud_publisher', anonymous=True)
    pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Create a PCL PointCloud
    cloud = pcl.PointCloud()
    # Fill in some data
    cloud.from_list([[1, 2, 3], [4, 5, 6], [7, 8, 9]])

    while not rospy.is_shutdown():
        # Convert PCL PointCloud to ROS PointCloud2
        cloud_msg = pcl_to_ros(cloud)
        pub.publish(cloud_msg)
        rate.sleep()

def pcl_to_ros(pcl_cloud):
    cloud_msg = PointCloud2()
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.header.frame_id = 'map'  # Change 'map' to your frame id
    cloud_msg.height = 1
    cloud_msg.width = pcl_cloud.size
    cloud_msg.fields.append(PointField(
        name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    cloud_msg.fields.append(PointField(
        name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    cloud_msg.fields.append(PointField(
        name="z", offset=8, datatype=PointField.FLOAT32, count=1))
    cloud_msg.point_step = 12
    cloud_msg.row_step = cloud_msg.point_step * pcl_cloud.size
    cloud_msg.is_bigendian = False
    cloud_msg.is_dense = True
    cloud_msg.data = pcl_cloud.to_array().tostring()
    return cloud_msg

if __name__ == '__main__':
    try:
        publish_point_cloud()
    except rospy.ROSInterruptException:
        pass
