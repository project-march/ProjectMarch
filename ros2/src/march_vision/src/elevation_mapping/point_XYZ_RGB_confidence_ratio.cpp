/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */

#define PCL_NO_PRECOMPILE
#include "march_vision/elevation_mapping/point_XYZ_RGB_confidence_ratio.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

template class pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>;
template class pcl::PCLBase<pcl::PointXYZRGBConfidenceRatio>;    // NOLINT(cppcoreguidelines-special-member-functions)
template class pcl::VoxelGrid<pcl::PointXYZRGBConfidenceRatio>;  // NOLINT(cppcoreguidelines-special-member-functions)
template void pcl::removeNaNFromPointCloud<pcl::PointXYZRGBConfidenceRatio>(
    const pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>& cloud_in, pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>& cloud_out,
    std::vector<int, std::allocator<int> >& index);
template class pcl::ExtractIndices<pcl::PointXYZRGBConfidenceRatio>;
template class pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio>;
template class pcl::CropBox<pcl::PointXYZRGBConfidenceRatio>;

std::ostream& operator<<(std::ostream& os, const pcl::PointXYZRGBConfidenceRatio& p) {
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << static_cast<int>(p.r) << ","  // NOLINT(cppcoreguidelines-pro-type-union-access)
     << static_cast<int>(p.g)                                                            // NOLINT(cppcoreguidelines-pro-type-union-access)
     << ","                                                                              // NOLINT(cppcoreguidelines-pro-type-union-access)
     << static_cast<int>(p.b) << " - " << p.confidence_ratio << ")";                     // NOLINT(cppcoreguidelines-pro-type-union-access)
  return (os);
}
