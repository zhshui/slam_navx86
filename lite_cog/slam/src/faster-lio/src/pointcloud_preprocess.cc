#include "pointcloud_preprocess.h"

#include <glog/logging.h>
#include <execution>

namespace faster_lio {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudType::Ptr &pcl_out) {
    AviaHandler(msg);
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out) {
    switch (lidar_type_) {
        case LidarType::AVIA:
            AviaHandlerPC2(msg);
            break;

        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::C16:
            C16Handler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type";
            break;
    }
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    int plsize = msg->point_num;

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<bool> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if ((msg->points[i].line < num_scans_) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = msg->points[i].x;
                cloud_full_[i].y = msg->points[i].y;
                cloud_full_[i].z = msg->points[i].z;
                cloud_full_[i].intensity = msg->points[i].reflectivity;
                cloud_full_[i].curvature =
                    msg->points[i].offset_time /
                    float(1000000);  // use curvature as time of each laser points, curvature unit: ms

                if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                    (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                    (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) &&
                        (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                             cloud_full_[i].z * cloud_full_[i].z >
                         (blind_ * blind_))) {
                    is_valid_pt[i] = true;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }
}

void PointCloudPreprocess::AviaHandlerPC2(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    // Livox PointCloud2 layout: x(f), y(f), z(f), intensity(f), tag(u8), line(u8), timestamp(f64)
    // point_step = 26, offsets: 0,4,8,12,16,17,18
    int plsize = msg->width * msg->height;
    cloud_out_.reserve(plsize / point_filter_num_ + 1);
    cloud_full_.resize(plsize);

    const uint8_t* data = msg->data.data();
    int point_step = msg->point_step;

    std::vector<bool> is_valid_pt(plsize, false);

    int tag_filtered = 0, line_filtered = 0, range_filtered = 0, pass_count = 0;
    for (int i = 0; i < plsize; i++) {
        const uint8_t* ptr = data + i * point_step;

        float px = *reinterpret_cast<const float*>(ptr);
        float py = *reinterpret_cast<const float*>(ptr + 4);
        float pz = *reinterpret_cast<const float*>(ptr + 8);
        float intensity = *reinterpret_cast<const float*>(ptr + 12);
        uint8_t tag = *(ptr + 16);
        uint8_t line = *(ptr + 17);
        double timestamp = *reinterpret_cast<const double*>(ptr + 18);

        // Filter: line < num_scans_, and tag bits 5-4 check
        if (line >= num_scans_) { line_filtered++; continue; }
        uint8_t tag_type = tag & 0x30;
        if (tag_type != 0x10 && tag_type != 0x00) { tag_filtered++; continue; }

        if (i % point_filter_num_ != 0) continue;

        double range = px * px + py * py + pz * pz;
        if (range < (blind_ * blind_)) { range_filtered++; continue; }

        cloud_full_[i].x = px;
        cloud_full_[i].y = py;
        cloud_full_[i].z = pz;
        cloud_full_[i].intensity = intensity;
        cloud_full_[i].curvature = 0.0;  // temp: match C16Handler behavior
        is_valid_pt[i] = true;
        pass_count++;
    }
    LOG_FIRST_N(INFO, 5) << "AviaHandlerPC2: plsize=" << plsize
                         << " line_filt=" << line_filtered
                         << " tag_filt=" << tag_filtered
                         << " range_filt=" << range_filtered
                         << " pass=" << pass_count
                         << " first_tag=" << (int)(plsize > 0 ? msg->data[16] : 0)
                         << " first_line=" << (int)(plsize > 0 ? msg->data[17] : 0);

    for (int i = 0; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) continue;

        Eigen::Vector3d pt_vec;
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = pl_orig.points[i].t / 1e6;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
        }

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }
}

void PointCloudPreprocess::C16Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    int plsize = msg->width * msg->height;
    cloud_out_.reserve(plsize / point_filter_num_ + 1);

    const uint8_t* data = msg->data.data();
    int point_step = msg->point_step;

    // Parse Livox PointCloud2 directly: x(f,0), y(f,4), z(f,8), intensity(f,12), tag(u8,16), line(u8,17), timestamp(f64,18)
    for (int i = 0; i < plsize; i++) {
        if (i % point_filter_num_ != 0) continue;

        const uint8_t* ptr = data + i * point_step;
        float px = *reinterpret_cast<const float*>(ptr);
        float py = *reinterpret_cast<const float*>(ptr + 4);
        float pz = *reinterpret_cast<const float*>(ptr + 8);
        float intensity = *reinterpret_cast<const float*>(ptr + 12);
        double timestamp = *reinterpret_cast<const double*>(ptr + 18);

        double range = px * px + py * py + pz * pz;
        if (range < (blind_ * blind_)) continue;

        PointType added_pt;
        added_pt.x = px;
        added_pt.y = py;
        added_pt.z = pz;
        added_pt.intensity = intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = 0.0;  // timestamp handled by separate mechanism

        cloud_out_.points.push_back(added_pt);
    }
}

}  // namespace faster_lio
