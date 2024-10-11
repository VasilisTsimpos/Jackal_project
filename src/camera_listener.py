#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def pointcloud_callback(pointcloud):
    min_z = float('inf')  # Initialize min_z with a large value
    closest_point = None

    # Iterate over points in the point cloud
    for point in pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point
        
        # Find the point with the smallest z value
        if z < min_z:
            min_z = z
            closest_point = (x, y, z)
    
    if closest_point:
        print(closest_point)

def listener():
    rospy.init_node('pointcloud_listener', anonymous=True)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
