#include <march_octree/march_octree_utils.h>
#include <march_octree/march_octree.h>
#include <march_octree/march_octree_node.h>
#include <octomap/OcTreeKey.h>

namespace octomap {

const double DEFAULT_SEARCH_RADIUS               = 0.08;
const double DEFAULT_MAX_DISTANCE_FROM_PLANE     = 0.02;
const double DEFAULT_MIN_CONSENSUS_RATIO         = 0.5;
const double DEFAULT_MAX_AVERAGE_DEVIATION_RATIO = 0.75;
const int    DEFAULT_NUMBER_OF_ITERATIONS        = 1;
const bool   DEFAULT_LEAST_SQUARES_ESTIMATION    = true;
const bool   DEFAULT_WEIGHT_BY_NUMBER_OF_HITS    = true;

// Returns if search ball S(query,r) contains node
template <typename Distance>
bool MarchOctreeUtils::contains(const MarchOctreeNode* node, const point3d& query, float squared_radius) {

    float x = query.x() - node->getNodeLocationX();
    float y = query.y() - node->getNodeLocationY();
    float z = query.z() - node->getNodeLocationZ();
    x = std::abs(x);
    y = std::abs(y);
    z = std::abs(z);
    x += node->extent;
    y += node->extent;
    z += node->extent;

    return (Distance::norm(x, y, z) < squared_radius);
}

void MarchOctreeUtils::findRadiusNeighbors(const MarchOctreeNode* node, const point3d query_point, double search_radius, std::vector<uint32_t>& neighbors_indices) {

    double radius_squared = search_radius * search_radius;
    float node_x = node->getNodeLocationX();
    float node_y = node->getNodeLocationY();
    float node_z = node->getNodeLocationZ();

    if (contains<L1Distance>(node, query_point, radius_squared)){

        // recursiveAction(node, recursiveAction)
        return;
    }

    if (!node->hasChildren()){

        float dx = query_point.x() - node_x;
        float dy = query_point.y() - node_y;
        float dz = query_point.z() - node_z;

        float distanceSquared = dx * dx + dy * dy + dz * dz;
        //TODO: Fix this actionRule 
        // if (distanceSquared < radius_squared) actionRule.doActionOnNeighbor(node);
        // return;
    }
    // check whether child nodes are in range.
    for (int childIndex = 0; childIndex < 8; childIndex++){
        //TODO: Check if this makes sense
        MarchOctreeNode child = node->getChildNode(childIndex);
        if (child == NULL) continue;
        //TODO: Fix the way this is called by only getting one radius
        // if (!overlaps(child, x, y, z, radius_squared)) continue;
        // //TODO: Fix how this is called and if it accepts pointer to the child
        // findRadiusNeighbors(child, x, y, z, radius, radius_squared, recursiveAction);
        // if (actionRule.earlyAbort()) return;
    }
}

void MarchOctreeUtils::findRadiusNeighbors(const MarchOctreeNode* node, const point3d query_point, double search_radius, std::function<void(MarchOctreeNode*)> recursiveAction) {

    double radius_squared = search_radius * search_radius;
    float node_x = node->getNodeLocationX();
    float node_y = node->getNodeLocationY();
    float node_z = node->getNodeLocationZ();

    if (contains<L1Distance>(node, query_point, radius_squared)){

        //recursiveAction(node, recursiveAction)
        return;
    }
    if (!node->hasChildren()){

        float dx = query_point.x() - node_x;
        float dy = query_point.y() - node_y;
        float dz = query_point.z() - node_z;

        float distanceSquared = dx * dx + dy * dy + dz * dz;
        //TODO: Fix this actionRule 
        //if (distanceSquared < radius_squared) actionRule.doActionOnNeighbor(node);
        return;
    }
    // check whether child nodes are in range.
    for (int childIndex = 0; childIndex < 8; childIndex++){
        //TODO: Check if this makes sense
        MarchOctreeNode child = node->getChildNode(childIndex);
        if (child == NULL)
        continue;
        //TODO: Fix the way this is called by only getting one radius
        //if (!overlaps(child, x, y, z, radius_squared)) continue;
        //TODO: Fix how this is called and if it accepts pointer to the child
        findRadiusNeighbors(&child, query_point, search_radius, recursiveAction);
        //if (actionRule.earlyAbort()) return;
    }
}


std::vector<MarchOctreeNode*> MarchOctreeUtils::searchNeighbors(MarchOctree* root, MarchOctreeNode* current_node) {

    std::vector<MarchOctreeNode*> neighbors;
    // Implement the search for neighbors logic here
    return neighbors;
}

void MarchOctreeUtils::computeNodeNormalRANSAC(MarchOctree* root, const octomap::OcTreeKey& key, int tree_depth) {

    MarchOctreeNode* current_node = root->search(key, tree_depth);
    computeNodeNormalRANSAC(root, current_node);
}

void MarchOctreeUtils::computeNodeNormalRANSAC(MarchOctree* root, MarchOctreeNode* current_node) {

    if (!current_node->isNodeLocationSet() || !current_node->isNormalSet()) {

        current_node->resetNormal();
        return;
    }
    std::vector<MarchOctreeNode*> neighbors = searchNeighbors(root, current_node);

    if (neighbors.size() < 2) return;

    Eigen::Vector3f current_normal = current_node->getNormal();
    // Need to be recomputed as the neighbors may have changed
    int number_of_hits_at_current_point = static_cast<int>(std::floor(current_node->getNumberOfHits()));
    // MutableInt and MutableDouble are not directly available, so we'll manage them differently
    int current_consensus = 0;
    double current_variance = 0.0;
    computeNormalConsensusAndVariance(current_node, current_normal, number_of_hits_at_current_point, neighbors, current_variance, current_consensus);

    for (int iteration = 0; iteration < DEFAULT_NUMBER_OF_ITERATIONS; iteration++) {

        Eigen::Vector3f candidate_normal = computeNormalFromTwoRandomNeighbors(neighbors, current_node->getNodeLocation());

        // TODO: Why and where is the max distance from plane used in the method
        if (DEFAULT_LEAST_SQUARES_ESTIMATION) candidate_normal = refineNormalWithLeastSquares(current_node, candidate_normal, neighbors);

        if (candidate_normal.isZero())
            continue;

        int candidate_consensus = 0;
        double candidate_variance = 0.0;

        computeNormalConsensusAndVariance(current_node, candidate_normal, number_of_hits_at_current_point, neighbors, candidate_variance, candidate_consensus);
        peekBestNormal(current_node, current_normal, current_variance, current_consensus, candidate_normal, candidate_variance, candidate_consensus);
    }
}

void MarchOctreeUtils::peekBestNormal(MarchOctreeNode* node, const Eigen::Vector3f& current_normal, double& current_variance, int& current_consensus,
                           Eigen::Vector3f& candidate_normal, double candidate_variance, int candidate_consensus) {

    if (isCandidateNormalBetter(current_variance, current_consensus, candidate_variance, candidate_consensus)) {

        //TODO: Might have to change the "const"s because of reassignment
        if (current_normal.dot(candidate_normal) < 0.0) candidate_normal = -candidate_normal;
        node->setNormal(candidate_normal);
        node->setNormalQuality(candidate_variance, candidate_consensus);
        current_consensus = candidate_consensus;
        current_variance = candidate_variance;
    }
}

bool MarchOctreeUtils::isCandidateNormalBetter(double current_variance, int current_consensus, double candidate_variance, int candidate_consensus) {

    if (candidate_consensus >= current_consensus && candidate_variance <= current_variance) return true;
    //TODO: change the name of this?
    bool has_smaller_consensus_but_is_much_better = candidate_consensus >= (int)(DEFAULT_MIN_CONSENSUS_RATIO * current_consensus)
        && candidate_variance <= DEFAULT_MAX_AVERAGE_DEVIATION_RATIO * current_variance;
    return has_smaller_consensus_but_is_much_better;
}

Eigen::Vector3f MarchOctreeUtils::computeNormalFromTwoRandomNeighbors(std::vector<MarchOctreeNode*>& neighbors, const point3d& current_node_hit_location) {

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, neighbors.size() - 1);
    std::vector<int> indices;

    while (indices.size() < 2) {

        int index = distribution(generator);
        if (std::find(indices.begin(), indices.end(), index) == indices.end())
            indices.push_back(index);
    }

    Eigen::Vector3f normal_candidate;
    // TODO: Check if the logic here is correct
    point3d loc1 = neighbors[indices[0]]->getNodeLocation();
    point3d loc2 = neighbors[indices[1]]->getNodeLocation();
    Eigen::Vector3f vec1(loc1.x() - current_node_hit_location.x(), loc1.y() - current_node_hit_location.y(), loc1.z() - current_node_hit_location.z());
    Eigen::Vector3f vec2(loc2.x() - current_node_hit_location.x(), loc2.y() - current_node_hit_location.y(), loc2.z() - current_node_hit_location.z());
    normal_candidate = vec1.cross(vec2);
    normal_candidate.normalize();

    return normal_candidate;
}

// TODO: Where is max distance from plane used and for what?
Eigen::Vector3f MarchOctreeUtils::refineNormalWithLeastSquares(MarchOctreeNode* current_node, const Eigen::Vector3f& ransac_normal,
                                                               const std::vector<MarchOctreeNode*>& neighbors) {

    // Compute the centroid of neighbors
    Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);

    for (const auto& neighbor : neighbors) {

        const point3d& neighbor_loc = neighbor->getNodeLocation();
        centroid[0] += neighbor_loc.x();
        centroid[1] += neighbor_loc.y();
        centroid[2] += neighbor_loc.z();
    }
    centroid /= static_cast<float>(neighbors.size());
    Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();

    for (const auto& neighbor : neighbors) {

        const point3d& neighbor_loc = neighbor->getNodeLocation();
        Eigen::Vector3f vec(neighbor_loc.x() - centroid[0], neighbor_loc.y() - centroid[1], neighbor_loc.z() - centroid[2]);
        covariance_matrix += vec * vec.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f refined_normal = svd.matrixV().col(2);
    refined_normal.normalize();
    return refined_normal;
}

void MarchOctreeUtils::computeNormalConsensusAndVariance(MarchOctreeNode* current_node, const Eigen::Vector3f& plane_normal,
                                              int hits_at_current_point, const std::vector<MarchOctreeNode*>& neighbors,
                                              double& variance_to_pack, int& consensus_to_pack) {

    variance_to_pack = 0.0;
    consensus_to_pack = 0;

    Eigen::Vector3f to_neighbor_hit_location;

    if (DEFAULT_WEIGHT_BY_NUMBER_OF_HITS) {

        // Weighted computation of consensus and variance
        for (const auto& neighbor : neighbors) {

            const octomap::point3d& neighbor_loc = neighbor->getNodeLocation();
            to_neighbor_hit_location << neighbor_loc.x() - current_node->getNodeLocationX(),
                                        neighbor_loc.y() - current_node->getNodeLocationY(),
                                        neighbor_loc.z() - current_node->getNodeLocationZ();
            double distance_from_plane = std::abs(plane_normal.dot(to_neighbor_hit_location));

            if (distance_from_plane <= DEFAULT_MAX_DISTANCE_FROM_PLANE) {

                double weight = neighbor->getNumberOfHits();
                variance_to_pack += weight * distance_from_plane * distance_from_plane;
                consensus_to_pack += weight;
            }
        }
    } else {
        // Non-weighted computation of consensus and variance
        for (const auto& neighbor : neighbors) {

            const octomap::point3d& neighbor_loc = neighbor->getNodeLocation();
            to_neighbor_hit_location << neighbor_loc.x() - current_node->getNodeLocationX(),
                                        neighbor_loc.y() - current_node->getNodeLocationY(),
                                        neighbor_loc.z() - current_node->getNodeLocationZ();
            double distance_from_plane = std::abs(plane_normal.dot(to_neighbor_hit_location));

            if (distance_from_plane <= DEFAULT_MAX_DISTANCE_FROM_PLANE) {

                variance_to_pack += distance_from_plane * distance_from_plane;
                consensus_to_pack++;
            }
        }
    }
}

} // namespace octomap

