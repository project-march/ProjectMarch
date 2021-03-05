#ifndef MARCH_PARAMETER_DETERMINER_H
#define MARCH_PARAMETER_DETERMINER_H
#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "utilities/realsense_gait_utilities.h"
#include "march_shared_msgs/GetGaitParameters.h"

using PlaneParameterVector = std::vector<pcl::ModelCoefficients::Ptr>;
using HullVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
using GaitParameters = march_shared_msgs::GaitParameters;
using PolygonVector = std::vector<std::vector<pcl::Vertices>>;

class ParameterDeterminer
{
public:
    ParameterDeterminer(YAML::Node config_tree, bool debugging);
    // This function is required to be implemented by any plane finder
    virtual bool determine_parameters(
        boost::shared_ptr<PlaneParameterVector> plane_parameter_vector,
        boost::shared_ptr<HullVector> hull_vector,
        boost::shared_ptr<PolygonVector> polygon_vector,
        SelectedGait selected_obstacle,
        boost::shared_ptr<GaitParameters> gait_parameters)=0;

    virtual ~ParameterDeterminer() {};

protected:
    boost::shared_ptr<PlaneParameterVector> plane_parameter_vector_;
    boost::shared_ptr<HullVector> hull_vector_;
    boost::shared_ptr<PolygonVector> polygon_vector_;
    SelectedGait selected_obstacle_;
    boost::shared_ptr<GaitParameters> gait_parameters_;
    YAML::Node config_tree_;
    bool debugging_;
};

/** The simple parameter determiner
 *
 */
class SimpleParameterDeterminer : ParameterDeterminer
{
public:
    //Use the constructors defined in the super class
    using ParameterDeterminer::ParameterDeterminer;
    /** This function should take in a pointcloud with matching normals and
     * regions, and turn this into chulls where the foot can be located. **/
    bool determine_parameters(
        boost::shared_ptr<PlaneParameterVector> plane_parameter_vector,
        boost::shared_ptr<HullVector> hull_vector,
        boost::shared_ptr<PolygonVector> polygon_vector,
        SelectedGait selected_obstacle,
        boost::shared_ptr<GaitParameters> gait_parameters) override;
};

#endif //MARCH_PARAMETER_DETERMINER_H
