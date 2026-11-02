#!/usr/bin/env python3
"""Filter near-field reflections and points below the robot base frame."""

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2


FLOAT32 = 7


def pointcloud_xyz(data, point_count, point_step, offsets, little_endian):
    """Read PointCloud2 float32 x/y/z fields without unpacking other fields."""
    endian = "<" if little_endian else ">"
    columns = [
        np.ndarray(
            shape=(point_count,),
            dtype=endian + "f4",
            buffer=data,
            offset=offset,
            strides=(point_step,),
        )
        for offset in offsets
    ]
    return np.column_stack(columns)


def filter_pointcloud_records(data, point_count, point_step, keep):
    """Copy selected binary point records while preserving every field."""
    records = np.frombuffer(data, dtype=np.uint8, count=point_count * point_step)
    return records.reshape(point_count, point_step)[keep].tobytes()


def base_transform(transform):
    """Return the rotation matrix and translation from source to base."""
    rotation = transform.rotation
    norm = np.sqrt(
        rotation.x * rotation.x
        + rotation.y * rotation.y
        + rotation.z * rotation.z
        + rotation.w * rotation.w
    )
    if norm == 0.0:
        raise ValueError("zero-length transform quaternion")
    x = rotation.x / norm
    y = rotation.y / norm
    z = rotation.z / norm
    w = rotation.w / norm
    rotation_matrix = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )
    translation = np.array(
        [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ],
        dtype=np.float32,
    )
    return rotation_matrix, translation


class LidarRangeFilter:
    def __init__(self):
        self.min_range = rospy.get_param("~min_range", 0.35)
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.min_base_z = rospy.get_param("~min_base_z", -0.15)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.base_transforms = {}

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
        rospy.loginfo(
            f"[lidar_filter] min_range={self.min_range}m, "
            f"base_frame={self.base_frame}, min_base_z={self.min_base_z}m, "
            f"input={self.sub.name}, output={self.pub.name}"
        )

    def get_base_transform(self, source_frame):
        if source_frame == self.base_frame:
            return np.eye(3, dtype=np.float32), np.zeros(3, dtype=np.float32)
        if source_frame not in self.base_transforms:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
            self.base_transforms[source_frame] = base_transform(transform.transform)
        return self.base_transforms[source_frame]

    def callback(self, msg):
        point_count = msg.width * msg.height
        if point_count == 0:
            return

        fields = {field.name: field for field in msg.fields}
        try:
            xyz_fields = tuple(fields[name] for name in ("x", "y", "z"))
        except KeyError:
            rospy.logerr_throttle(5, "[lidar_filter] point cloud has no x/y/z fields")
            return
        if any(field.datatype != FLOAT32 or field.count != 1 for field in xyz_fields):
            rospy.logerr_throttle(5, "[lidar_filter] x/y/z fields must be scalar float32")
            return

        xyz = pointcloud_xyz(
            msg.data,
            point_count,
            msg.point_step,
            tuple(field.offset for field in xyz_fields),
            little_endian=not msg.is_bigendian,
        )
        horizontal_range = np.hypot(xyz[:, 0], xyz[:, 1])
        keep = np.all(np.isfinite(xyz), axis=1) & (horizontal_range >= self.min_range)

        try:
            rotation, translation = self.get_base_transform(msg.header.frame_id)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, ValueError) as exc:
            rospy.logwarn_throttle(5, f"[lidar_filter] base transform unavailable: {exc}")
        else:
            base_xyz = xyz.dot(rotation.T) + translation
            keep &= base_xyz[:, 2] >= self.min_base_z

        kept_count = int(np.count_nonzero(keep))
        dropped = point_count - kept_count
        self.dropped_count += dropped
        self.total_count += point_count
        if kept_count == 0:
            return

        filtered_data = filter_pointcloud_records(
            msg.data, point_count, msg.point_step, keep
        )
        self.pub.publish(
            PointCloud2(
                header=msg.header,
                height=1,
                width=kept_count,
                fields=msg.fields,
                is_bigendian=msg.is_bigendian,
                point_step=msg.point_step,
                row_step=kept_count * msg.point_step,
                data=filtered_data,
                is_dense=msg.is_dense,
            )
        )

        pct = 100.0 * self.dropped_count / max(self.total_count, 1)
        rospy.loginfo_throttle(
            5,
            f"[lidar_filter] dropped={self.dropped_count}/{self.total_count} "
            f"({pct:.1f}%), base_z_min={self.min_base_z}m",
        )


if __name__ == "__main__":
    rospy.init_node("lidar_range_filter")
    LidarRangeFilter()
    rospy.spin()
