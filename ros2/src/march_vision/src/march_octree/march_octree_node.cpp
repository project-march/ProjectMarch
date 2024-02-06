#include <march_octree/march_octree_node.h>
#include <cmath>


namespace octomap {

MarchOctreeNode::MarchOctreeNode()
    : normal_x{NAN},
      normal_y{NAN},
      normal_z{NAN},
      normal_average_deviation{NAN},
      normal_consensus_size{0},
      node_location_x{NAN},
      node_location_y{NAN},
      node_location_z{NAN},
      //TODO: Check if PCL has a functionality for this
      last_hit_timestamp{NAN},
      number_of_hits{0}
{
}

void MarchOctreeNode::copyData(MarchOctreeNode& other){
    OcTreeNode::copyData(other);
    normal_x = other.getNormalX();
    normal_y = other.getNormalY();
    normal_z = other.getNormalZ();
    normal_average_deviation = other.getNormalAverageDeviation();
    normal_consensus_size = other.getNormalConsensusSize();
    node_location_x = other.getNodeLocationX();
    node_location_y = other.getNodeLocationY();
    node_location_z = other.getNodeLocationZ();
    last_hit_timestamp = other.getLastHitTimestamp();
    number_of_hits = other.getNumberOfHits();
}

void MarchOctreeNode::clear()
{
    this->setLogOdds(0);
    resetNormal();
    resetNodeLocation();
}

void MarchOctreeNode::resetNormal()
{
    normal_x = NAN;
    normal_y = NAN;
    normal_y = NAN;
    normal_average_deviation = NAN;
    normal_consensus_size = 0;
}   

void MarchOctreeNode::resetNodeLocation()
{
    node_location_x = NAN;
    node_location_y = NAN;
    node_location_z = NAN;
    number_of_hits = 0;
    last_hit_timestamp = 0;
}

void MarchOctreeNode::updateNormalChildren()
{}

Eigen::Vector3f MarchOctreeNode::getNormal()
{
    Eigen::Vector3f res_vector;
    res_vector << normal_x, normal_y, normal_x;
    return res_vector;
}

// //TODO: Change this method
// Eigen::Vector3f MarchOctreeNode::getNormalCopy()
// {
//     return Eigen::Vector3f(normal_x, normal_y, normal_z);
// }

void MarchOctreeNode::setNormal(Eigen::Vector3f& normal)
{
    normal_x = static_cast<float>(normal[0]);
    normal_y = static_cast<float>(normal[1]);
    normal_z = static_cast<float>(normal[2]);
}

void MarchOctreeNode::negateNormal()
{
    normal_x = -normal_x;
    normal_y = -normal_y;
    normal_z = -normal_z;
}

void MarchOctreeNode::setNormalQuality(float average_deviation, int consensus_size)
{
    normal_average_deviation = average_deviation;
    normal_consensus_size = consensus_size;
}

float MarchOctreeNode::getNormalAverageDeviation()
{
    return normal_average_deviation;
}

int MarchOctreeNode::getNormalConsensusSize()
{
    return normal_consensus_size;
}

bool MarchOctreeNode::isNormalSet()
{
    return !std::isnan(normal_x) && !std::isnan(normal_y) && !std::isnan(normal_z);
}

bool MarchOctreeNode::isNodeLocationSet()
{
    return !std::isnan(node_location_x) && !std::isnan(node_location_y) && !std::isnan(node_location_z);
}
//TODO: Change this method
void MarchOctreeNode::getNodeLocation(pcl::PointXYZ& hit_location_to_pack)
{
    hit_location_to_pack.x = this->getNodeLocationX();
    hit_location_to_pack.y = this->getNodeLocationY();
    hit_location_to_pack.z = this->getNodeLocationZ();
}

// //TODO: Decide if this is needed.
// pcl::PointXYZ MarchOctreeNode::getNodeLocationCopy()
// {
//     return pcl::PointXYZ(node_location_x, node_location_y, node_location_z);
// }

//TODO: Remove unneeded methods from these three.
void MarchOctreeNode::updateNodeLocation(pcl::PointXYZ& center_update)
{
    updateNodeLocation(center_update, 1L);
}

void MarchOctreeNode::updateNodeLocation(pcl::PointXYZ& center_update, long update_weight)
{
    updateNodeLocation(center_update, update_weight, LONG_MAX);
}

void MarchOctreeNode::updateNodeLocation(pcl::PointXYZ& centerUpdate, long updateWeight, long maximum_number_of_hits)
{
    updateNodeLocation(centerUpdate, updateWeight, maximum_number_of_hits, NAN);
}

void MarchOctreeNode::updateNodeLocation(pcl::PointXYZ& center_update, long update_weight, long maximum_number_of_hits, long current_timestamp)
{
    updateNodeLocation(x_update, y_update, z_update, update_weight, maximum_number_of_hits, current_timestamp);
}

void MarchOctreeNode::updateNodeLocation(float x_update, float y_update, float z_update)
{
    updateNodeLocation(x_update, y_update, z_update, 1L, NAN);
}

void MarchOctreeNode::updateNodeLocation(float x_update, float y_update, float z_update, long update_weight, long maximum_number_of_hits, long current_timestamp)
{
    if (number_of_hits == 0)
    {
        node_location_x = 0.0f;
        node_location_y = 0.0f;
        node_location_z = 0.0f;
    }

    number_of_hits += update_weight;
    number_of_hits = std::min(number_of_hits, maximum_number_of_hits);
    double n_inv = static_cast<double>(update_weight) / static_cast<double>(number_of_hits);
    node_location_x += (x_update - node_location_x) * n_inv;
    node_location_y += (y_update - node_location_y) * n_inv;
    node_location_z += (z_update - node_location_z) * n_inv;
    if (current_timestamp != NAN && current_timestamp > last_hit_timestamp)
        last_hit_timestamp = current_timestamp;
}


void MarchOctreeNode::updateNodeLocationChildren()
{
    if (children == nullptr)
    {
        resetNodeLocation();
        return;
    }

    node_location_x = 0.0f;
    node_location_y = 0.0f;
    node_location_z = 0.0f;
    int count = 0;

    for (int i = 0; i < 8; i++)
    {
        MarchOctreeNode& child = this->getChildNode(i);

        if (child.isNodeLocationSet())
        {
            node_location_x += child.getNodeLocationX();
            node_location_y += child.getNodeLocationY();
            node_location_z += child.getNodeLocationZ();
            count++;
        }
    }

    if (count == 0)
    {
        resetNodeLocation();
        return;
    }

    double inv_count = 1.0 / count;
    node_location_x *= inv_count;
    node_location_y *= inv_count;
    node_location_z *= inv_count;
}

float MarchOctreeNode::getNodeLocationX()
{
    return node_location_x;
}

float MarchOctreeNode::getNodeLocationY()
{
    return node_location_y;
}

float MarchOctreeNode::getNodeLocationZ()
{
    return node_location_z;
}

long MarchOctreeNode::getNumberOfHits()
{
    return number_of_hits;
}

float MarchOctreeNode::getNormalX()
{
    return normal_x;
}

float MarchOctreeNode::getNormalY()
{
    return normal_y;
}

float MarchOctreeNode::getNormalZ()
{
    return normal_z;
}

//TODO: Change this method
bool MarchOctreeNode::isLastHitTimestampDefined()
{
    return last_hit_timestamp != NAN;
}

long MarchOctreeNode::getLastHitTimestamp()
{
    return last_hit_timestamp;
}

// void updateHitTimestamp(long timestamp){

//     last_hit_timestamp = timestamp;
// }

bool MarchOctreeNode::epsilonEqualsInternal(MarchOctreeNode& other, double epsilon)
{
    if (!std::isnan(normal_x) && !std::isnan(other.getNormalX()) && !std::isnan(normal_y) && !std::isnan(other.getNormalY()) &&
        !std::isnan(normal_z) && !std::isnan(other.getNormalZ()))
    {
        if (fabs(normal_x - other.getNormalX()) > epsilon)
            return false;
        if (fabs(normal_y - other.getNormalY()) > epsilon)
            return false;
        if (fabs(normal_z - other.getNormalZ()) > epsilon)
            return false;
    }

    if (!std::isnan(node_location_x) && !std::isnan(other.getNodeLocationX()) && !std::isnan(node_location_y) && !std::isnan(other.getNodeLocationY()) &&
        !std::isnan(node_location_z) && !std::isnan(other.getNodeLocationZ()))
    {
        if (fabs(node_location_x - other.getNodeLocationX()) > epsilon)
            return false;
        if (fabs(node_location_y - other.getNodeLocationY()) > epsilon)
            return false;
        if (fabs(node_location_z - other.getNodeLocationZ()) > epsilon)
            return false;
    }

    return std::abs(this->getLogOdds() - other.getLogOdds()) <= epsilon;
}
}
