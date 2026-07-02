#!/bin/bash
# Build visualize_pcd_filter — Z-range filter visualization tool
set -e
cd "$(dirname "$0")"
echo "Building visualize_pcd_filter..."
g++ -std=c++17 -O2 -o visualize_pcd_filter visualize_pcd_filter.cpp \
    $(pkg-config --cflags --libs pcl_common-1.10 pcl_io-1.10 pcl_filters-1.10 pcl_visualization-1.10) \
    -I/usr/include/vtk-7.1 \
    -lvtkRenderingCore-7.1 -lvtkCommonCore-7.1 -lvtkCommonDataModel-7.1 -lvtkCommonMath-7.1 \
    -lvtkRenderingOpenGL2-7.1 -lvtkInteractionStyle-7.1 -lvtkFiltersSources-7.1
echo "Done: $(pwd)/visualize_pcd_filter"
