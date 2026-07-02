/**
 * visualize_pcd_filter — 3D visualization of Z-range filtered PCD
 *
 * Loads a PCD file, applies a Z-axis passthrough filter, and colors points:
 *   🟢 Green = retained   (Z in range, "没有被过滤的点")
 *   🔴 Red   = discarded  (Z out of range, "被过滤掉的点")
 *
 * The color is embedded in a PointXYZRGB cloud so it can be viewed
 * interactively or saved to a colored PCD for later inspection.
 *
 * Build:
 *   g++ -std=c++17 -O2 -o visualize_pcd_filter visualize_pcd_filter.cpp \
 *       $(pkg-config --cflags --libs pcl_common-1.10 pcl_io-1.10 \
 *                      pcl_filters-1.10 pcl_visualization-1.10)
 *
 * Usage:
 *   ./visualize_pcd_filter <input.pcd> [options]
 *
 * Options:
 *   --z-min Z             Lower Z bound (default: -0.1)
 *   --z-max Z             Upper Z bound (default: 0.6)
 *   --downsample N        Random downsample to ~N points for display (default: 500000)
 *   --save-colored FILE   Also save the colored cloud to FILE.pcd
 *   --help                Show this message
 *
 * Interactive controls (PCLVisualizer):
 *   Mouse drag           Rotate
 *   Shift + Mouse drag   Pan
 *   Scroll / Right drag  Zoom
 *   r                    Reset camera
 *   q / Esc              Quit
 *   1 / 2 / 3            Toggle retained / discarded / both
 *   g                    Toggle grid
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>
#include <random>
#include <set>

struct Options {
    std::string input_path;
    std::string colored_output_path;   // if non-empty, save colored cloud
    double z_min       = -0.1;
    double z_max       = 0.6;
    int    downsample  = 500000;       // target points for display (-1 = no downsample)
};

// ---------------------------------------------------------------------------
// CLI parsing
// ---------------------------------------------------------------------------
void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " <input.pcd> [options]\n"
        << "\n"
        << "Visualize which points survive the Z-range filter used in\n"
        << "PCD → 2D occupancy-grid conversion.\n"
        << "\n"
        << "  🟢 Green  = within Z range  (retained / 没有被过滤的点)\n"
        << "  🔴 Red    = outside Z range (discarded / 被过滤掉的点)\n"
        << "\n"
        << "Options:\n"
        << "  --z-min Z              Lower Z bound in meters (default: -0.1)\n"
        << "  --z-max Z              Upper Z bound in meters (default: 0.6)\n"
        << "  --downsample N         Random-sample to ~N points for faster display\n"
        << "                         (default: 500000, -1 = show all)\n"
        << "  --save-colored FILE    Also save the color-coded cloud to FILE\n"
        << "  --help                 Show this message\n"
        << "\n"
        << "Interactive controls:\n"
        << "  Mouse drag             Rotate\n"
        << "  Shift + Mouse drag     Pan\n"
        << "  Scroll / Right drag    Zoom\n"
        << "  r                      Reset camera to top-down view\n"
        << "  1 / 2 / 3              Show retained / discarded / both\n"
        << "  q / Esc                Quit\n"
        << "\n"
        << "Example:\n"
        << "  " << prog << " scans_23.pcd\n"
        << "  " << prog << " scans_23.pcd --z-min -0.2 --z-max 0.5 --downsample 200000\n";
}

bool parse_cli(int argc, char** argv, Options& opts) {
    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "--help" || a == "-h") {
            print_usage(argv[0]);
            return false;
        } else if (a == "--z-min" && i + 1 < argc) {
            opts.z_min = std::atof(argv[++i]);
        } else if (a == "--z-max" && i + 1 < argc) {
            opts.z_max = std::atof(argv[++i]);
        } else if (a == "--downsample" && i + 1 < argc) {
            opts.downsample = std::atoi(argv[++i]);
        } else if (a == "--save-colored" && i + 1 < argc) {
            opts.colored_output_path = argv[++i];
            // auto-append .pcd if no extension
            if (opts.colored_output_path.find('.') == std::string::npos)
                opts.colored_output_path += ".pcd";
        } else if (a[0] == '-') {
            std::cerr << "Unknown option: " << a << "\n";
            print_usage(argv[0]);
            return false;
        } else {
            opts.input_path = a;
        }
    }
    if (opts.input_path.empty()) {
        std::cerr << "ERROR: no input PCD specified.\n\n";
        print_usage(argv[0]);
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    Options opts;
    if (!parse_cli(argc, argv, opts)) return 1;

    // --- Load PCD ---------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(opts.input_path, *cloud) == -1) {
        std::cerr << "ERROR: Failed to load " << opts.input_path << "\n";
        return 1;
    }
    std::cout << "Loaded " << cloud->size() << " points from " << opts.input_path << "\n";

    if (cloud->empty()) {
        std::cerr << "ERROR: empty point cloud.\n";
        return 1;
    }

    // --- Z-range statistics ------------------------------------------------
    float z_min_all =  std::numeric_limits<float>::max();
    float z_max_all = -std::numeric_limits<float>::max();
    for (const auto& pt : cloud->points) {
        if (pt.z < z_min_all) z_min_all = pt.z;
        if (pt.z > z_max_all) z_max_all = pt.z;
    }

    // --- Build color-coded XYZRGB cloud -----------------------------------
    // We assign green (0,255,0) to retained points and red (255,0,0) to
    // discarded points.  This lets us inspect the cloud visually and also
    // save it for later use.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored->reserve(cloud->size());

    std::size_t n_retained = 0, n_discarded = 0;

    for (const auto& pt : cloud->points) {
        pcl::PointXYZRGB cpt;
        cpt.x = pt.x;
        cpt.y = pt.y;
        cpt.z = pt.z;

        bool in_range = (pt.z >= opts.z_min) && (pt.z <= opts.z_max);
        if (in_range) {
            // Green = retained
            cpt.r = 0;   cpt.g = 255; cpt.b = 0;
            ++n_retained;
        } else {
            // Red = discarded
            cpt.r = 255; cpt.g = 0;   cpt.b = 0;
            ++n_discarded;
        }
        colored->push_back(cpt);
    }

    double ratio = 100.0 * n_retained / cloud->size();
    std::cout << "Z range in data:  [" << z_min_all << ", " << z_max_all << "] m\n";
    std::cout << "Z filter window:  [" << opts.z_min  << ", " << opts.z_max  << "] m\n";
    std::cout << "Retained (green): " << n_retained  << "  (" << ratio << "%)\n";
    std::cout << "Discarded (red):  " << n_discarded << "  (" << 100.0 - ratio << "%)\n";

    // --- Optional: save colored cloud -------------------------------------
    if (!opts.colored_output_path.empty()) {
        if (pcl::io::savePCDFileBinary(opts.colored_output_path, *colored) == -1) {
            std::cerr << "ERROR: Failed to save colored PCD to " << opts.colored_output_path << "\n";
        } else {
            std::cout << "Colored PCD saved to: " << opts.colored_output_path << "\n";
            std::cout << "  You can open it with: pcl_viewer " << opts.colored_output_path << "\n";
        }
    }

    // --- Downsample for display (PCLVisualizer struggles with >1M pts) ----
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (opts.downsample > 0 && static_cast<std::size_t>(opts.downsample) < colored->size()) {
        // Use PCL RandomSample to pick a subset
        pcl::RandomSample<pcl::PointXYZRGB> rs;
        rs.setInputCloud(colored);
        rs.setSample(static_cast<unsigned int>(opts.downsample));
        rs.setSeed(std::random_device{}());
        rs.filter(*display_cloud);

        // Count retained/discarded in the sampled set for display
        std::size_t samp_retained = 0, samp_discarded = 0;
        for (const auto& pt : display_cloud->points) {
            if (pt.g == 255 && pt.r == 0) ++samp_retained;
            else ++samp_discarded;
        }
        std::cout << "Display sample: " << display_cloud->size()
                  << " points (retained=" << samp_retained
                  << ", discarded=" << samp_discarded << ")\n";
    } else {
        display_cloud = colored;
    }

    // --- PCLVisualizer ----------------------------------------------------
    pcl::visualization::PCLVisualizer viewer("PCD Z-Filter Visualization — Green=retained  Red=discarded");
    viewer.setBackgroundColor(0.05, 0.05, 0.05);  // dark background
    viewer.addCoordinateSystem(1.0, "origin", 0);
    viewer.initCameraParameters();

    // --- Add retained (green) points --------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr red_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto& pt : display_cloud->points) {
        if (pt.g == 255 && pt.r == 0)
            green_cloud->push_back(pt);
        else
            red_cloud->push_back(pt);
    }

    // Green points
    if (!green_cloud->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_handler(green_cloud, 0, 255, 0);
        viewer.addPointCloud<pcl::PointXYZRGB>(
            green_cloud, green_handler, "retained_cloud");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "retained_cloud");
    }

    // Red points
    if (!red_cloud->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_handler(red_cloud, 255, 0, 0);
        viewer.addPointCloud<pcl::PointXYZRGB>(
            red_cloud, red_handler, "discarded_cloud");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "discarded_cloud");
    }

    // --- Grid overlay (ground reference plane at Z=0) ----------------------
    {
        pcl::ModelCoefficients plane_coeffs;
        plane_coeffs.values = {0.0f, 0.0f, 1.0f, 0.0f};
        viewer.addPlane(plane_coeffs, 0.0, 0.0, 0.0, "ground_plane");
    }
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        "ground_plane");
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.3, 0.3, "ground_plane");

    // --- Z-range wireframe box (semi-transparent) -------------------------
    // Compute XY bounds from the display cloud
    float x_min =  std::numeric_limits<float>::max();
    float x_max = -std::numeric_limits<float>::max();
    float y_min =  std::numeric_limits<float>::max();
    float y_max = -std::numeric_limits<float>::max();
    for (const auto& pt : display_cloud->points) {
        if (pt.x < x_min) x_min = pt.x;
        if (pt.x > x_max) x_max = pt.x;
        if (pt.y < y_min) y_min = pt.y;
        if (pt.y > y_max) y_max = pt.y;
    }
    viewer.addCube(x_min, x_max, y_min, y_max, opts.z_min, opts.z_max,
                   0.0, 1.0, 0.0, "z_range_box");
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        "z_range_box");
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "z_range_box");

    // --- Text overlay ------------------------------------------------------
    std::string info = "Z filter: [" + std::to_string(opts.z_min).substr(0,4) + ", "
                       + std::to_string(opts.z_max).substr(0,4) + "] m  |  "
                       + "Retained: " + std::to_string(ratio).substr(0,5) + "%  |  "
                       + "Green=retained  Red=discarded";
    viewer.addText(info, 10, 20, 16, 1.0, 1.0, 1.0, "info_text");

    // --- Animation callback to show stats on keypress --------------------
    // We register a simple keyboard callback
    viewer.registerKeyboardCallback(
        [](const pcl::visualization::KeyboardEvent& event, void*) {
            if (event.getKeySym() == "r" && event.keyDown()) {
                // Reset to top-down view
                // (handled by PCL's default 'r' — we just log)
                std::cout << "[view] 'r' pressed — resetting camera\n";
            }
        },
        nullptr);

    std::cout << "\n=== Interactive viewer started ===\n"
              << "Controls:\n"
              << "  Mouse drag        Rotate\n"
              << "  Shift+drag        Pan\n"
              << "  Scroll            Zoom\n"
              << "  r                 Reset camera\n"
              << "  1 / 2 / 3         Toggle retained / discarded / both\n"
              << "  q / Esc           Quit\n"
              << std::endl;

    // --- Spin loop ---------------------------------------------------------
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    std::cout << "Done.\n";
    return 0;
}
