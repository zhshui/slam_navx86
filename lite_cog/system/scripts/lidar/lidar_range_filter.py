#!/usr/bin/env python3
"""
点云距离过滤器 —— 滤除雷达支架/机体自反射点云
订阅 /livox/lidar，滤除距离传感器原点太近的点，发布到 /livox/lidar_filtered
"""
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np

class LidarRangeFilter:
    def __init__(self):
        self.min_range = rospy.get_param("~min_range", 0.35)  # 过滤半径 (米)
        self.sub = rospy.Subscriber(
            rospy.get_param("~input_topic", "/livox/lidar"),
            PointCloud2,
            self.callback,
            queue_size=1,
        )
        self.pub = rospy.Publisher(
            rospy.get_param("~output_topic", "/livox/lidar_filtered"),
            PointCloud2,
            queue_size=1,
        )
        self.dropped_count = 0
        self.total_count = 0
        rospy.loginfo(f"[lidar_filter] min_range={self.min_range}m, input={self.sub.name}, output={self.pub.name}")

    def callback(self, msg: PointCloud2):
        # 获取所有点（返回完整 field tuple）
        all_points = list(pc2.read_points(msg, field_names=None, skip_nans=True))
        if len(all_points) == 0:
            return

        # 定位 x, y 字段的索引
        field_names = [f.name for f in msg.fields]
        try:
            ix = field_names.index("x")
            iy = field_names.index("y")
        except ValueError:
            rospy.logerr("[lidar_filter] point cloud has no x/y fields")
            return

        r2 = self.min_range ** 2
        filtered = []
        dropped = 0

        for p in all_points:
            # 只判断水平距离（支架反射通常水平上很近）
            if p[ix] ** 2 + p[iy] ** 2 < r2:
                dropped += 1
                continue
            filtered.append(p)

        self.dropped_count += dropped
        self.total_count += dropped + len(filtered)

        if len(filtered) == 0:
            return

        # 重新打包 PointCloud2（保持原有 fields 和 header）
        new_msg = pc2.create_cloud(msg.header, msg.fields, filtered)

        # 统计日志（每 5 秒输出一次）
        if self.total_count > 0 and self.dropped_count % 1000 < dropped + 100:
            pct = 100.0 * self.dropped_count / max(self.total_count, 1)
            rospy.loginfo_throttle(5, f"[lidar_filter] dropped {self.dropped_count}/{self.total_count} pts ({pct:.1f}%) near < {self.min_range}m")

        self.pub.publish(new_msg)


if __name__ == "__main__":
    rospy.init_node("lidar_range_filter")
    LidarRangeFilter()
    rospy.spin()
