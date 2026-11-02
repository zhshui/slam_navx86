/**
 * level_pcd — Z-axis leveling for 3D point cloud maps
 *
 * Detects the dominant ground plane via RANSAC, then rotates the entire
 * point cloud so the ground normal aligns with true vertical (0,0,1).
 *
 * Build:
 *   g++ -std=c++17 -O2 -o level_pcd level_pcd.cpp \
 *       $(pkg-config --cflags --libs pcl_common-1.10 pcl_io-1.10 \
 *                      pcl_filters-1.10 pcl_segmentation-1.10 pcl_sample_consensus-1.10)
 *
 * Usage:
 *   ./level_pcd <input.pcd> [output.pcd] [options]
 *
 * Options:
 *   --max-iterations N      RANSAC iterations (default: 500)
 *   --distance-threshold D  Inlier distance threshold in meters (default: 0.05)
 *   --max-tilt-degrees D    Max allowed ground tilt from Z (default: 45)
 *   --ground-z-min Z        Min Z for ground inlier points (default: -2.0)
 *   --voxel-leaf L          Voxel downsampling leaf size (default: 0.1)
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>

static constexpr double PI = 3.14159265358979323846;

struct Options {
    std::string input;
    std::string output;
    int max_iterations = 500;
    double distance_threshold = 0.05;   // RANSAC inlier threshold (m)
    double max_tilt_deg = 45.0;         // max angle deviation from Z for ground normal
    double ground_z_min = -2.0;         // min Z for ground candidate points
    double voxel_leaf = 0.1;            // voxel downsample leaf (m)
};

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " <input.pcd> [output.pcd] [options]\n"
              << "Options:\n"
              << "  --max-iterations N      RANSAC iterations (default: 500)\n"
              << "  --distance-threshold D  Inlier threshold in meters (default: 0.05)\n"
              << "  --max-tilt-degrees D    Max allowed ground tilt from Z (default: 45)\n"
              << "  --ground-z-min Z        Min Z for inlier points (default: -2.0)\n"
              << "  --voxel-leaf L          Voxel leaf size for downsampling (default: 0.1)\n"
              << "\nExample:\n"
              << "  " << prog << " scans_1.pcd scans_1_leveled.pcd\n"
              << "  " << prog << " scans_1.pcd --max-tilt-degrees 30\n";
}

Eigen::Matrix4f compute_leveling_transform(const Eigen::Vector3f& ground_normal,
                                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_inliers) {
    // Desired normal: straight up (0, 0, 1)
    const Eigen::Vector3f target(0.0f, 0.0f, 1.0f);

    // Ensure ground normal points generally upward
    Eigen::Vector3f n = ground_normal;
    if (n.dot(target) < 0) n = -n;

    // Compute rotation axis and angle
    // axis = target × n: rotating target around this axis by +angle maps target → n
    // Then R^T maps n → target, leveling the ground
    Eigen::Vector3f axis = target.cross(n);
    float axis_norm = axis.norm();

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    if (axis_norm < 1e-9f) {
        // Already level
        std::cout << "[level_pcd] Ground plane is already level (normal=Z)\n";
        return T;
    }

    axis.normalize();
    float cos_angle = n.dot(target);
    float angle = std::acos(std::max(-1.0f, std::min(1.0f, cos_angle)));

    std::cout << "[level_pcd] Ground normal: [" << n.x() << ", " << n.y() << ", " << n.z() << "]\n";
    std::cout << "[level_pcd] Tilt angle: " << angle * 180.0 / PI << " degrees\n";

    // Rotation from current ground normal to (0,0,1)
    Eigen::AngleAxisf rot(angle, axis);
    Eigen::Matrix3f R = rot.toRotationMatrix();

    // We want to rotate the cloud so ground becomes horizontal
    // R maps target→n, so R^T = R^{-1} maps n→target
    T.block<3, 3>(0, 0) = R.transpose();

    // Compute translation to keep the ground inliers at roughly the same height
    // by keeping their centroid Z at approximately 0 or its original average
    // We apply the rotation at origin, no translation shift needed unless desired
    // (rotate around origin to preserve absolute heights)

    return T;
}

int main(int argc, char** argv) {
    Options opts;

    // Parse CLI
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--max-iterations" && i + 1 < argc) {
            opts.max_iterations = std::atoi(argv[++i]);
        } else if (arg == "--distance-threshold" && i + 1 < argc) {
            opts.distance_threshold = std::atof(argv[++i]);
        } else if (arg == "--max-tilt-degrees" && i + 1 < argc) {
            opts.max_tilt_deg = std::atof(argv[++i]);
        } else if (arg == "--ground-z-min" && i + 1 < argc) {
            opts.ground_z_min = std::atof(argv[++i]);
        } else if (arg == "--voxel-leaf" && i + 1 < argc) {
            opts.voxel_leaf = std::atof(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (arg[0] == '-') {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        } else {
            positional.push_back(arg);
        }
    }

    if (positional.empty()) {
        print_usage(argv[0]);
        return 1;
    }

    opts.input = positional[0];
    opts.output = (positional.size() > 1) ? positional[1]
                  : opts.input.substr(0, opts.input.find_last_of('.')) + "_leveled.pcd";

    std::cout << "[level_pcd] Input:  " << opts.input << "\n";
    std::cout << "[level_pcd] Output: " << opts.output << "\n";
    std::cout << "[level_pcd] Parameters:\n"
              << "  max_iterations=" << opts.max_iterations << "\n"
              << "  distance_threshold=" << opts.distance_threshold << " m\n"
              << "  max_tilt_deg=" << opts.max_tilt_deg << "°\n"
              << "  ground_z_min=" << opts.ground_z_min << " m\n"
              << "  voxel_leaf=" << opts.voxel_leaf << " m\n";

    // --- Load PCD ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(opts.input, *cloud) == -1) {
        std::cerr << "[level_pcd] ERROR: Failed to load " << opts.input << "\n";
        return 1;
    }
    std::cout << "[level_pcd] Loaded " << cloud->size() << " points\n";

    if (cloud->empty()) {
        std::cerr << "[level_pcd] ERROR: empty cloud\n";
        return 1;
    }

    // --- Voxel downsample (for plane detection speed) ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (opts.voxel_leaf > 0.0f && cloud->size() > 100000) {
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(opts.voxel_leaf, opts.voxel_leaf, opts.voxel_leaf);
        vg.filter(*cloud_filtered);
        std::cout << "[level_pcd] After voxel downsample: " << cloud_filtered->size() << " points\n";
    } else {
        cloud_filtered = cloud;
    }

    // --- Robust Z percentile for ground candidate selection ---
    // Use percentile-based range to avoid outlier contamination
    // (e.g. a few stray points at Z=-5 would skew min/max entirely)
    std::vector<float> z_values;
    z_values.reserve(cloud_filtered->size());
    for (const auto& pt : cloud_filtered->points)
        z_values.push_back(pt.z);
    std::sort(z_values.begin(), z_values.end());

    double z_p01 = z_values[std::max(size_t(0), std::min(z_values.size() - 1,
                              size_t(z_values.size() * 0.01)))];   // 1st percentile
    double z_p30 = z_values[std::max(size_t(0), std::min(z_values.size() - 1,
                              size_t(z_values.size() * 0.30)))];   // 30th percentile
    double z_p99 = z_values[std::max(size_t(0), std::min(z_values.size() - 1,
                              size_t(z_values.size() * 0.99)))];   // 99th percentile

    // Ground candidates: bottom 30% of the *robust* Z span (P1 → P30)
    double ground_z_min_eff = z_p01;
    double ground_z_max = z_p30;

    std::cout << "[level_pcd] Z percentiles: P1=" << z_p01 << " P30=" << z_p30
              << " P99=" << z_p99 << " (robust min/max)\n";

    // Fallback if percentiles are degenerate (flat cloud)
    if (ground_z_max - ground_z_min_eff < 0.01) {
        ground_z_min_eff = z_values.front();
        ground_z_max = z_values.back();
        std::cout << "[level_pcd] Percentile range too narrow, using full Z range\n";
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_candidates(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(ground_z_min_eff, ground_z_max);
        pass.filter(*ground_candidates);
    }
    std::cout << "[level_pcd] Ground candidates (Z in [" << ground_z_min_eff
              << ", " << ground_z_max << "]): " << ground_candidates->size() << " points\n";

    if (ground_candidates->size() < 100) {
        std::cerr << "[level_pcd] ERROR: Too few ground candidate points. "
                  << "Try adjusting --ground-z-min or check the data.\n";
        return 1;
    }

    // --- RANSAC plane detection ---
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(opts.max_iterations);
    seg.setDistanceThreshold(opts.distance_threshold);
    seg.setInputCloud(ground_candidates);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "[level_pcd] ERROR: RANSAC could not find a plane.\n";
        return 1;
    }

    // Plane model: ax + by + cz + d = 0
    Eigen::Vector3f normal(coefficients->values[0],
                           coefficients->values[1],
                           coefficients->values[2]);

    // Check if plane normal is within allowed tilt from Z axis
    float tilt_angle = std::acos(std::abs(normal.dot(Eigen::Vector3f(0, 0, 1))));
    float tilt_deg = tilt_angle * 180.0f / PI;

    std::cout << "[level_pcd] Detected plane: "
              << coefficients->values[0] << "x + "
              << coefficients->values[1] << "y + "
              << coefficients->values[2] << "z + "
              << coefficients->values[3] << " = 0\n";
    std::cout << "[level_pcd] Plane normal: [" << normal.x() << ", " << normal.y()
              << ", " << normal.z() << "], tilt=" << tilt_deg << "°\n";
    std::cout << "[level_pcd] Inliers: " << inliers->indices.size()
              << " / " << ground_candidates->size() << "\n";

    if (tilt_deg > opts.max_tilt_deg) {
        std::cerr << "[level_pcd] ERROR: Detected plane tilt (" << tilt_deg
                  << "°) exceeds --max-tilt-degrees (" << opts.max_tilt_deg
                  << "°). Likely a wall, not ground. Aborting to keep original PCD.\n";
        return 1;
    }

    // --- Compute leveling transform ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(ground_candidates);
    extract.setIndices(inliers);
    extract.filter(*inlier_cloud);

    Eigen::Matrix4f T = compute_leveling_transform(normal, inlier_cloud);

    // --- Apply transform to FULL cloud ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr leveled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *leveled, T);
    std::cout << "[level_pcd] Transformed " << leveled->size() << " points\n";

    // --- Save leveled PCD ---
    if (pcl::io::savePCDFileBinary(opts.output, *leveled) == -1) {
        std::cerr << "[level_pcd] ERROR: Failed to save " << opts.output << "\n";
        return 1;
    }
    std::cout << "[level_pcd] Leveled PCD saved to: " << opts.output << "\n";

    // Report Z range change
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    pcl::PointXYZ lvl_min, lvl_max;
    pcl::getMinMax3D(*leveled, lvl_min, lvl_max);
    std::cout << "[level_pcd] Z range before: [" << min_pt.z << ", " << max_pt.z << "]\n";
    std::cout << "[level_pcd] Z range after:  [" << lvl_min.z << ", " << lvl_max.z << "]\n";
    std::cout << "[level_pcd] Done.\n";

    return 0;
}
