#ifndef MARCH_OCTREE_NODE_HPP
#define MARCH_OCTREE_NODE_HPP

#include <octomap/OcTreeNode.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeIterator.hxx>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace octomap {

class MarchOctreeNode : public OcTreeNode
{
private:
    float normal_x;
    float normal_y;
    float normal_z;
    float normal_average_deviation;
    int normal_consensus_size;
    float node_location_x;
    float node_location_y;
    float node_location_z;
    long last_hit_timestamp;
    long number_of_hits;

public:
    
    MarchOctreeNode() : OcTreeNode(), normal_x(0.0f), normal_y(0.0f), normal_z(0.0f), normal_average_deviation(0.0f), 
                                      normal_consensus_size(0), node_location_x(0.0f), node_location_y(0.0f), 
                                      node_location_z(0.0f), last_hit_timestamp(NAN), number_of_hits(0) {}
    ~MarchOctreeNode() = default;

    void copyData(MarchOctreeNode& other);
    void clear();
    void resetNormal();
    void resetNodeLocation();
    void updateNormalChildren();
    MarchOctreeNode& getChildNode(unsigned int index);
    void updateNodeLocationChildren();

    //TODO: Change so that this returns a vector for normal
    Eigen::Vector3f getNormal();

    //TODO: Change so that this sets a vector for normal
    void setNormal(Eigen::Vector3f& normal);
    void negateNormal();
    void setNormalQuality(float average_deviation, int consensus_size);
    float getNormalAverageDeviation();
    int getNormalConsensusSize();
    bool isNormalSet();
    bool isNodeLocationSet();

    // TODO: Change to PCL::Point
    void getNodeLocation(pcl::PointXYZ& hit_location_to_pack);

    //Point3D getHitLocationCopy();
    void updateNodeLocation(pcl::PointXYZ& center_update);
    void updateNodeLocation(pcl::PointXYZ& center_update, long update_weight);
    void updateNodeLocation(pcl::PointXYZ& center_update, long update_weight, long maximum_number_of_hits);
    void updateNodeLocation(pcl::PointXYZ& center_update, long update_weight, long maximum_number_of_hits, long current_timestamp);
    void updateNodeLocation(float x_update, float y_update, float z_update);
    long getNumberOfHits();
    float getNodeLocationX();
    float getNodeLocationY();
    float getNodeLocationZ();
    float getNormalX();
    float getNormalY();
    float getNormalZ();
    bool isLastHitTimestampDefined();
    long getLastHitTimestamp();
    // void updateHitTimestamp(long timestamp);

protected:

    bool epsilonEqualsInternal(MarchOctreeNode& other, double epsilon);
};

}
#endif // MARCH_OCTREE_NODE_HPP
