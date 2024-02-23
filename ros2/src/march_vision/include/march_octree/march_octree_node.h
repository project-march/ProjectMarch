#ifndef MARCH_OCTREE_NODE_HPP
#define MARCH_OCTREE_NODE_HPP

#include <octomap/OcTreeNode.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeIterator.hxx>
#include <vector>
#include <Eigen/Core>


namespace octomap {

class MarchOctreeNode : public OcTreeNode
{
private:
    float m_normal_x;
    float m_normal_y;
    float m_normal_z;
    float m_normal_average_deviation;
    int m_normal_consensus_size;
    float m_node_location_x;
    float m_node_location_y;
    float m_node_location_z;
    long m_last_hit_timestamp;
    long m_number_of_hits;

public:
    
    MarchOctreeNode() : OcTreeNode(), m_normal_x(0.0f), m_normal_y(0.0f), m_normal_z(0.0f), m_normal_average_deviation(0.0f), 
                                      m_normal_consensus_size(0), m_node_location_x(0.0f), m_node_location_y(0.0f), 
                                      m_node_location_z(0.0f), m_last_hit_timestamp(NAN), m_number_of_hits(0) {}
    ~MarchOctreeNode() = default;

    void copyData(MarchOctreeNode& other);
    void clear();
    void resetNormal();
    void resetNodeLocation();
    void updateNormalChildren();
    MarchOctreeNode& getChildNode(unsigned int index) const;
    void updateNodeLocationChildren();

    //TODO: Change so that this returns a vector for normal
    Eigen::Vector3f getNormal();

    //TODO: Change so that this sets a vector for normal
    void setNormal(Eigen::Vector3f& normal);
    void negateNormal();
    void setNormalQuality(float average_deviation, int consensus_size);
    float getNormalAverageDeviation() const;
    int getNormalConsensusSize() const;
    bool isNormalSet() const;
    bool isNodeLocationSet() const;

    // TODO: Change to PCL::Point
    octomap::point3d getNodeLocation();

    //Point3D getHitLocationCopy();
    void updateNodeLocation(octomap::point3d& center_update);
    void updateNodeLocation(octomap::point3d& center_update, long update_weight, long maximum_number_of_hits, long current_timestamp);
    void updateNodeLocation(float x_update, float y_update, float z_update);
    void updateNodeLocation(float x_update, float y_update, float z_update, long update_weight, long maximum_number_of_hits, long current_timestamp);
    long getNumberOfHits() const;
    float getNodeLocationX() const;
    float getNodeLocationY() const;
    float getNodeLocationZ() const;
    float getNormalX() const;
    float getNormalY() const;
    float getNormalZ() const;
    bool isLastHitTimestampDefined() const;
    long getLastHitTimestamp() const;
    // void updateHitTimestamp(long timestamp);

protected:

    bool epsilonEqualsInternal(MarchOctreeNode& other, double epsilon);
};

}
#endif // MARCH_OCTREE_NODE_HPP
