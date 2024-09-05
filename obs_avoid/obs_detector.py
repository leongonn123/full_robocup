#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import numpy as np

class ObstacleDetector:
    def __init__(self):
        # Initialize publishers for markers and obstacle detection
        self.marker_pub = rospy.Publisher('/obstacle_marker', Marker, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/barrier_detected', Bool, queue_size=10)
        
        # Subscribe to the RealSense point cloud output
        rospy.Subscriber('/camera/depth/points', PointCloud2, self.callback)
        
        # Track the last state of obstacle detection
        self.last_obstacle_state = None  

    def callback(self, point_cloud):
        # Test with a smaller detection range, e.g., 0.2 meters
        detection_range = 0.5

        # Setup the marker for RViz visualization
        marker = Marker()
        marker.header.frame_id = point_cloud.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.2  # Diameter of 20 cm for each sphere
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Fully opaque
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(1)  # Each marker lasts for 1 second

        # Extract points from the point cloud
        gen = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)

        obstacle_detected = False
        for x, y, z in gen:
            # rospy.loginfo(f"Point detected: x={x}, y={y}, z={z}")
            if z <= detection_range:
                #rospy.loginfo(f"Detected close point within 0.2 meters: z={z}")
                p = Point(x, y, z)
                marker.points.append(p)
                obstacle_detected = True

        # Log and publish marker if obstacles are detected
        if marker.points:
            #rospy.loginfo("Obstacle detected within 0.2 meters!")
            self.marker_pub.publish(marker)
            self.obstacle_pub.publish(True)
        else:
            #rospy.loginfo("No obstacles within 0.2 meters.")
            self.obstacle_pub.publish(False)



def main():
    # Initialize the node
    rospy.init_node('obstacle_detector', anonymous=True)
    detector = ObstacleDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
