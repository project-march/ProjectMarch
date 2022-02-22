/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_REALSENSE_TO_PCL
#define MARCH_REALSENSE_TO_PCL

#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/**
 * Transforms pointclouds from the realsense cameras to pointcloud from the PCL
 * library
 *
 * @param points realsense pointcloud
 * @return PointCloud::Ptr pcl pointcloud
 */
PointCloud::Ptr points_to_pcl(const rs2::points& points)
{
    PointCloud::Ptr cloud = boost::make_shared<PointCloud>();

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto x_min = -0.5;
    auto x_max = 0.5;
    auto y_min = -1;
    auto y_max = 1;
    auto z_min = -0.10;
    auto z_max = 2;

    int point_count = 0;
    auto ptr = points.get_vertices();
    for (std::size_t i = 0; i < points.size(); i++) {
        // if (ptr->y < y_min || ptr->y > y_max || ptr->x < x_min || ptr->x >
        // x_max
        //     || ptr->z < z_min || ptr->z > z_max) {
        // } else {
        (*cloud)[point_count].x = ptr->x;
        (*cloud)[point_count].y = ptr->y;
        (*cloud)[point_count].z = ptr->z;
        point_count++;
        // }
        ptr++;
    }
    cloud->points.resize(point_count);

    return cloud;
}

#endif // MARCH_REALSENSE_TO_PCL
