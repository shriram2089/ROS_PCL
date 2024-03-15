import numpy as np
import sensor_msgs.point_cloud2 as pc2
import rospy

def preprocess_point_cloud(point_cloud):
    # Convert ROS PointCloud2 message to numpy array
    points = np.array([[p[0], p[1], p[2], p[3]] for p in pc2.read_points(point_cloud)])

    # Downsample the point cloud (example: keep every nth point)
    downsampled_points = points[::2]  # Keep every 2nd point, adjust as needed
    
    # Normalize the point cloud (example: scale all coordinates to range [0, 1])
    # Assuming the point cloud is in the form [x, y, z, intensity]
    min_vals = np.min(downsampled_points[:, :3], axis=0)
    max_vals = np.max(downsampled_points[:, :3], axis=0)
    normalized_points = (downsampled_points[:, :3] - min_vals) / (max_vals - min_vals)
    
    return normalized_points

def preprocess_bag_file(input_bag_file, output_bag_file, input_topic, output_topic):
    # Initialize ROS node
    rospy.init_node('point_cloud_preprocessor', anonymous=True)

    # Create publishers and subscribers
    input_pub = rospy.Publisher(input_topic, pc2, queue_size=10)
    output_pub = rospy.Publisher(output_topic, pc2, queue_size=10)

    # Open bag files
    with rosbag.Bag(input_bag_file, 'r') as in_bag, rosbag.Bag(output_bag_file, 'w') as out_bag:
        for topic, msg, t in in_bag.read_messages():
            if topic == input_topic:
                # Preprocess point cloud
                processed_points = preprocess_point_cloud(msg)

                # Convert processed points back to PointCloud2 message
                processed_pc2_msg = pc2.create_cloud_xyz32(msg.header, processed_points)

                # Publish processed point cloud
                output_pub.publish(processed_pc2_msg)

            # Republish other topics without modification
            else:
                input_pub.publish(msg)
            
            # Write messages to output bag
            out_bag.write(topic, msg, t)

if __name__ == '__main__':
    input_bag_file = 'extracted.bag'
    output_bag_file = 'output.bag'
    input_topic = '/input_point_cloud_topic'
    output_topic = '/output_point_cloud_topic'

    preprocess_bag_file(input_bag_file, output_bag_file, input_topic, output_topic)
