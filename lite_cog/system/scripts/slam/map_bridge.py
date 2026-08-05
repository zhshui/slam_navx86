#!/usr/bin/env python3
"""Bridge SLAM point cloud to 2D occupancy grid map for web display."""
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import struct

class MapBridge:
    def __init__(self):
        self.resolution = rospy.get_param("~resolution", 0.05)
        self.width = rospy.get_param("~width", 800)
        self.height = rospy.get_param("~height", 800)
        self.min_z = rospy.get_param("~min_z", -0.3)
        self.max_z = rospy.get_param("~max_z", 0.5)
        self.origin_x = rospy.get_param("~origin_x", -20.0)
        self.origin_y = rospy.get_param("~origin_y", -20.0)

        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.sub = rospy.Subscriber("/cloud_registered", PointCloud2, self.callback)
        rospy.loginfo("MapBridge started, publishing to /map")

    def callback(self, msg):
        # Parse PointCloud2
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        field_names = [f.name for f in msg.fields]
        offsets = {f.name: f.offset for f in msg.fields}
        try:
            xi = field_names.index('x')
            yi = field_names.index('y')
            zi = field_names.index('z')
        except ValueError:
            return

        point_step = msg.point_step
        data = msg.data
        count = 0
        for i in range(0, len(data), point_step):
            if i + point_step > len(data):
                break
            x = struct.unpack_from('f', data, i + offsets['x'])[0]
            y = struct.unpack_from('f', data, i + offsets['y'])[0]
            z = struct.unpack_from('f', data, i + offsets['z'])[0]

            if z < self.min_z or z > self.max_z:
                continue

            cx = int((x - self.origin_x) / self.resolution)
            cy = int((y - self.origin_y) / self.resolution)
            if 0 <= cx < self.width and 0 <= cy < self.height:
                grid[cy, cx] = 100
                count += 1

        # Build OccupancyGrid
        og = OccupancyGrid()
        og.header = msg.header
        og.header.frame_id = "map"
        og.info = MapMetaData()
        og.info.resolution = self.resolution
        og.info.width = self.width
        og.info.height = self.height
        og.info.origin = Pose()
        og.info.origin.position.x = self.origin_x
        og.info.origin.position.y = self.origin_y
        og.info.origin.position.z = 0
        og.info.origin.orientation.w = 1.0
        og.data = grid.flatten().tolist()
        self.map_pub.publish(og)
        rospy.loginfo_throttle(5, "MapBridge: {} points → /map ({}x{})".format(count, self.width, self.height))

if __name__ == "__main__":
    rospy.init_node("map_bridge")
    MapBridge()
    rospy.spin()
