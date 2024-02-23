#include <march_octree/march_octree_node.h>
#include <cmath>


namespace octomap {

MarchOctreeNode::MarchOctreeNode()
    : m_normal_x{NAN},
      m_normal_y{NAN},
      m_normal_z{NAN},
      m_normal_average_deviation{NAN},
      m_normal_consensus_size{0},
      m_node_location_x{NAN},
      m_node_location_y{NAN},
      m_node_location_z{NAN},
      //TODO: Check if PCL has a functionality for this
      m_last_hit_timestamp{NAN},
      m_number_of_hits{0}
{
}

void MarchOctreeNode::copyData(MarchOctreeNode& other){
    OcTreeNode::copyData(other);
    m_normal_x = other.getNormalX();
    m_normal_y = other.getNormalY();
    m_normal_z = other.getNormalZ();
    m_normal_average_deviation = other.getNormalAverageDeviation();
    m_normal_consensus_size = other.getNormalConsensusSize();
    m_node_location_x = other.getNodeLocationX();
    m_node_location_y = other.getNodeLocationY();
    m_node_location_z = other.getNodeLocationZ();
    m_last_hit_timestamp = other.getLastHitTimestamp();
    m_number_of_hits = other.getNumberOfHits();
}

void MarchOctreeNode::clear()
{
    this->setLogOdds(0);
    resetNormal();
    resetNodeLocation();
}

void MarchOctreeNode::resetNormal()
{
    m_normal_x = NAN;
    m_normal_y = NAN;
    m_normal_y = NAN;
    m_normal_average_deviation = NAN;
    m_normal_consensus_size = 0;
}   

void MarchOctreeNode::resetNodeLocation()
{
    m_node_location_x = NAN;
    m_node_location_y = NAN;
    m_node_location_z = NAN;
    m_number_of_hits = 0;
    m_last_hit_timestamp = 0;
}

void MarchOctreeNode::updateNormalChildren()
{}

Eigen::Vector3f MarchOctreeNode::getNormal()
{
    Eigen::Vector3f res_vector;
    res_vector << m_normal_x, m_normal_y, m_normal_x;
    return res_vector;
}

// //TODO: Change this method
// Eigen::Vector3f MarchOctreeNode::getNormalCopy()
// {
//     return Eigen::Vector3f(normal_x, normal_y, normal_z);
// }

void MarchOctreeNode::setNormal(Eigen::Vector3f& normal)
{
    m_normal_x = static_cast<float>(normal[0]);
    m_normal_y = static_cast<float>(normal[1]);
    m_normal_z = static_cast<float>(normal[2]);
}

void MarchOctreeNode::negateNormal()
{
    m_normal_x = -m_normal_x;
    m_normal_y = -m_normal_y;
    m_normal_z = -m_normal_z;
}

void MarchOctreeNode::setNormalQuality(float average_deviation, int consensus_size)
{
    m_normal_average_deviation = average_deviation;
    m_normal_consensus_size = consensus_size;
}

float MarchOctreeNode::getNormalAverageDeviation() const
{
    return m_normal_average_deviation;
}

int MarchOctreeNode::getNormalConsensusSize() const
{
    return m_normal_consensus_size;
}

bool MarchOctreeNode::isNormalSet() const
{
    return !std::isnan(m_normal_x) && !std::isnan(m_normal_y) && !std::isnan(m_normal_z);
}

bool MarchOctreeNode::isNodeLocationSet() const
{
    return !std::isnan(m_node_location_x) && !std::isnan(m_node_location_y) && !std::isnan(m_node_location_z);
}
//TODO: Change this method
point3d MarchOctreeNode::getNodeLocation()
{
    return point3d(m_node_location_x, m_node_location_y, m_node_location_z);
}

// //TODO: Decide if this is needed.
// pcl::PointXYZ MarchOctreeNode::getNodeLocationCopy()
// {
//     return pcl::PointXYZ(node_location_x, node_location_y, node_location_z);
// }

//TODO: Remove unneeded methods from these three.
void MarchOctreeNode::updateNodeLocation(octomap::point3d& center_update)
{
    //TODO: Change these to the actual values to be used
    updateNodeLocation(center_update, 1L, LONG_MAX, NAN);
}

void MarchOctreeNode::updateNodeLocation(octomap::point3d& center_update, long update_weight, long maximum_number_of_hits, long current_timestamp)
{
    updateNodeLocation(center_update.x(), center_update.y(), center_update.z(), update_weight, maximum_number_of_hits, current_timestamp);
}

void MarchOctreeNode::updateNodeLocation(float x_update, float y_update, float z_update)
{
    updateNodeLocation(x_update, y_update, z_update, LONG_MAX, NAN, m_last_hit_timestamp);
}

void MarchOctreeNode::updateNodeLocation(float x_update, float y_update, float z_update, long update_weight, long maximum_number_of_hits, long current_timestamp)
{
    if (m_number_of_hits == 0)
    {
        m_node_location_x = 0.0f;
        m_node_location_y = 0.0f;
        m_node_location_z = 0.0f;
    }

    m_number_of_hits += update_weight;
    m_number_of_hits = std::min(m_number_of_hits, maximum_number_of_hits);
    double n_inv = static_cast<double>(update_weight) / static_cast<double>(m_number_of_hits);
    m_node_location_x += (x_update - m_node_location_x) * n_inv;
    m_node_location_y += (y_update - m_node_location_y) * n_inv;
    m_node_location_z += (z_update - m_node_location_z) * n_inv;
    if (current_timestamp != NAN && current_timestamp > m_last_hit_timestamp)
        m_last_hit_timestamp = current_timestamp;
}


void MarchOctreeNode::updateNodeLocationChildren()
{
    if (children == nullptr)
    {
        resetNodeLocation();
        return;
    }

    m_node_location_x = 0.0f;
    m_node_location_y = 0.0f;
    m_node_location_z = 0.0f;
    int count = 0;

    for (int i = 0; i < 8; i++)
    {
        MarchOctreeNode& child = this->getChildNode(i);

        if (child.isNodeLocationSet())
        {
            m_node_location_x += child.getNodeLocationX();
            m_node_location_y += child.getNodeLocationY();
            m_node_location_z += child.getNodeLocationZ();
            count++;
        }
    }

    if (count == 0)
    {
        resetNodeLocation();
        return;
    }

    double inv_count = 1.0 / count;
    m_node_location_x *= inv_count;
    m_node_location_y *= inv_count;
    m_node_location_z *= inv_count;
}

float MarchOctreeNode::getNodeLocationX() const
{
    return m_node_location_x;
}

float MarchOctreeNode::getNodeLocationY() const
{
    return m_node_location_y;
}

float MarchOctreeNode::getNodeLocationZ() const
{
    return m_node_location_z;
}

long MarchOctreeNode::getNumberOfHits() const
{
    return m_number_of_hits;
}

float MarchOctreeNode::getNormalX() const
{
    return m_normal_x;
}

float MarchOctreeNode::getNormalY() const
{
    return m_normal_y;
}

float MarchOctreeNode::getNormalZ() const
{
    return m_normal_z;
}

//TODO: Change this method
bool MarchOctreeNode::isLastHitTimestampDefined() const
{
    return m_last_hit_timestamp != NAN;
}

long MarchOctreeNode::getLastHitTimestamp() const
{
    return m_last_hit_timestamp;
}

// void updateHitTimestamp(long timestamp){

//     last_hit_timestamp = timestamp;
// }

bool MarchOctreeNode::epsilonEqualsInternal(MarchOctreeNode& other, double epsilon)
{
    if (!std::isnan(m_normal_x) && !std::isnan(other.getNormalX()) && !std::isnan(m_normal_y) && !std::isnan(other.getNormalY()) &&
        !std::isnan(m_normal_z) && !std::isnan(other.getNormalZ()))
    {
        if (fabs(m_normal_x - other.getNormalX()) > epsilon ||
            fabs(m_normal_y - other.getNormalY()) > epsilon ||
            fabs(m_normal_z - other.getNormalZ()) > epsilon)  {
            return false;
        }
    }

    if (!std::isnan(m_node_location_x) && !std::isnan(other.getNodeLocationX()) && !std::isnan(m_node_location_y) && !std::isnan(other.getNodeLocationY()) &&
        !std::isnan(m_node_location_z) && !std::isnan(other.getNodeLocationZ()))
    {
        if (fabs(m_node_location_x - other.getNodeLocationX()) > epsilon || 
            fabs(m_node_location_y - other.getNodeLocationY()) > epsilon ||
            fabs(m_node_location_z - other.getNodeLocationZ()) > epsilon)  {
            return false;
    }

    return std::abs(this->getLogOdds() - other.getLogOdds()) <= epsilon;
}
}
