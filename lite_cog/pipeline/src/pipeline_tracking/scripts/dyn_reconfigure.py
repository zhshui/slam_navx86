#!/usr/bin/python

import rospy
import dynamic_reconfigure.client

rospy.init_node("myconfig_py", anonymous=True)

# --- Wait for TEB params to be loaded on parameter server ---
# move_base loads YAML at startup; wait until params are available
teb_ns = "/move_base/TebLocalPlannerROS"
timeout = rospy.Duration(30.0)
start = rospy.Time.now()
while not rospy.has_param(teb_ns + "/xy_goal_tolerance") and not rospy.is_shutdown():
    if rospy.Time.now() - start > timeout:
        break
    rospy.sleep(0.5)

# Read goal tolerances from rosparam (loaded from YAML), not hardcoded
xy_tol = rospy.get_param(teb_ns + "/xy_goal_tolerance", 0.1)
yaw_tol = rospy.get_param(teb_ns + "/yaw_goal_tolerance", 0.1)
rospy.loginfo("[myconfig] TEB goal tolerance from YAML: xy=%.2f, yaw=%.2f" % (xy_tol, yaw_tol))

# Apply slow-velocity mode + goal tolerances to TEB via dynamic_reconfigure
client_teb = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")

slow_vel_params = {
    "max_vel_x": 0.1,
    "max_vel_x_backwards": 0.02,
    "max_vel_y": 0.08,
    "max_vel_theta": 0.3,
    "acc_lim_x": 0.08,
    "acc_lim_y": 0.03,
    "acc_lim_theta": 0.3,
    "yaw_goal_tolerance": yaw_tol,
    "xy_goal_tolerance": xy_tol,
}
client_teb.update_configuration(slow_vel_params)
rospy.loginfo("[myconfig] TEB slow-vel + goal tolerance applied")

# Disable unused costmap obstacle layers (lidar2scan / stereo2scan not in use)
for layer in [
    "/move_base/global_costmap/obstacle_layer_lidar2scan",
    "/move_base/global_costmap/obstacle_layer_stereo2scan",
    "/move_base/local_costmap/obstacle_layer_lidar2scan",
    "/move_base/local_costmap/obstacle_layer_stereo2scan",
]:
    try:
        client = dynamic_reconfigure.client.Client(layer)
        client.update_configuration({"enabled": False})
    except Exception:
        pass  # layer may not exist, skip
