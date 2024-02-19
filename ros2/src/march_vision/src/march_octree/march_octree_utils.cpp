#include <march_octree/march_octree_utils.h>
#include <march_octree/march_octree.h>
#include <march_octree/march_octree_node.h>
#include <octomap/OcTreeKey.h>

namespace octomap {

// // Returns if search ball S(query,r) contains node
// template <typename Distance>
// bool MarchOctreeUtils::contains(const MarchOctreeNode* node, const point3d& query, float squared_radius){

//     float x = query.x() - node->getNodeLocationX();
//     float y = query.y() - node->getNodeLocationY();
//     float z = query.z() - node->getNodeLocationZ();
//     x = std::abs(x);
//     y = std::abs(y);
//     z = std::abs(z);
//     x += node->extent;
//     y += node->extent;
//     z += node->extent;

//     return (Distance::norm(x, y, z) < squared_radius);

// }

// void findRadiusNeighbors(const MarchOctreeNode* node, const point3d query_point, double search_radius, std::vector<uint32_t>& neighborsIndices){

//     double radius_squared = search_radius * search_radius;
//     double node_x = node->getNodeLocationX();
//     double node_y = node->getNodeLocationY();
//     double node_z = node->getNodeLocationZ();

//     if (contains<L1Distance>(node, query_point, radius_squared)){

//         recursiveAction(node, recursiveAction)
//         return;
//     }
//     if (!node->hasChildren()){

//         double dx = query_point.x() - node_x;
//         double dy = query_point.y() - node_y;
//         double dz = query_point.z() - node_z;

//         double distanceSquared = dx * dx + dy * dy + dz * dz;
//         //TODO: Fix this actionRule 
//         if (distanceSquared < radius_squared) actionRule.doActionOnNeighbor(node);
//         return;
//     }
//     // check whether child nodes are in range.
//     for (int childIndex = 0; childIndex < 8; childIndex++){
//         //TODO: Check if this makes sense
//         MarchOctreeNode child = node->getChildNode(childIndex);
//         if (child == null)
//         continue;
//         //TODO: Fix the way this is called by only getting one radius
//         if (!overlaps(child, x, y, z, radius, radiusSquared)) continue;
//         //TODO: Fix how this is called and if it accepts pointer to the child
//         findRadiusNeighbors(child, x, y, z, radius, radiusSquared, recursiveAction);
//         if (actionRule.earlyAbort()) return;
//     }
// }

// void findRadiusNeighbors(const MarchOctreeNode* node, const point3d query_point, double search_radius, std::function<void(MarchOctreeNode*)> recursiveAction) {

//     double radius_squared = search_radius * search_radius;
//     double node_x = node->getNodeLocationX();
//     double node_y = node->getNodeLocationY();
//     double node_z = node->getNodeLocationZ();

//     if (contains<L1Distance>(node, query_point, radius_squared)){

//         recursiveAction(node, recursiveAction)
//         return;
//     }
//     if (!node->hasChildren()){

//         double dx = query_point.x() - node_x;
//         double dy = query_point.y() - node_y;
//         double dz = query_point.z() - node_z;

//         double distanceSquared = dx * dx + dy * dy + dz * dz;
//         //TODO: Fix this actionRule 
//         if (distanceSquared < radius_squared) actionRule.doActionOnNeighbor(node);
//         return;
//     }
//     // check whether child nodes are in range.
//     for (int childIndex = 0; childIndex < 8; childIndex++){
//         //TODO: Check if this makes sense
//         MarchOctreeNode child = node->getChildNode(childIndex);
//         if (child == null)
//         continue;
//         //TODO: Fix the way this is called by only getting one radius
//         if (!overlaps(child, x, y, z, radius, radiusSquared)) continue;
//         //TODO: Fix how this is called and if it accepts pointer to the child
//         findRadiusNeighbors(child, x, y, z, radius, radiusSquared, recursiveAction);
//         if (actionRule.earlyAbort()) return;
//     }
// }

#include <cmath>
#include <random>
#include <algorithm>
#include <vector>
#include <Eigen/Core>


static std::vector<MarchOctreeNode*> searchNeighbors(MarchOctree* root, MarchOctreeNode* current_node) {

    std::vector<MarchOctreeNode*> neighbors;
    // Implement the search for neighbors logic here
    return neighbors;
}

static void computeNodeNormalRANSAC(MarchOctree* root, const octomap::OcTreeKey& key, int tree_depth) {

    MarchOctreeNode* current_node = root->search(key, tree_depth);
    computeNodeNormalRANSAC(root, current_node);
}

static void computeNodeNormalRANSAC(MarchOctree* root, MarchOctreeNode* current_node) {

    if (!current_node->isHitLocationSet() || !current_node->isNormalSet()) {

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

    for (int iteration = 0; iteration < DEFAULT_NUMBER_OF_ITERATOINS; iteration++) {

        Eigen::Vector3f candidate_normal = computeNormalFromTwoRandomNeighbors(neighbors, current_node->getNodeLocation());

        if (DEFAULT_LEAST_SQUARES_ESTIMATION) candidate_normal = refineNormalWithLeastSquares(current_node, candidate_normal, neighbors);

        if (candidate_normal.isZero())
            continue;

        int candidate_consensus = 0;
        double candidate_variance = 0.0;

        computeNormalConsensusAndVariance(current_node, candidate_normal, number_of_hits_at_current_point, neighbors, candidate_variance, candidate_consensus);

        peekBestNormal(current_node, current_normal, current_variance, current_consensus, candidate_normal, candidate_variance, candidate_consensus);
    }
}


static void peekBestNormal(MarchOctreeNode* node, const Eigen::Vector3f& current_normal, double& current_variance, int& current_consensus,
                           Eigen::Vector3f& candidate_normal, double candidate_variance, int candidate_consensus) {

    if (isCandidateNormalBetter(current_variance, current_consensus, candidate_variance, candidate_consensus))
    {
        //TODO: Might have to change the "const"s because of reassignment
        if (current_normal.dot(candidate_normal) < 0.0) candidate_normal = -candidate_normal;
        node->setNormal(candidate_normal);
        node->setNormalQuality(candidate_variance, candidate_consensus);
        current_consensus = candidate_consensus;
        current_variance = candidate_variance;
    }
}

static bool isCandidateNormalBetter(double current_variance, int current_consensus, double candidate_variance, int candidate_consensus) {

    if (candidate_consensus >= current_consensus && candidate_variance <= current_variance) return true;
    //TODO: change the name of this?
    bool has_smaller_consensus_but_is_much_better = candidate_consensus >= (int)(DEFAULT_MIN_CONSENSUS_RATIO * current_consensus)
        && candidate_variance <= DEFAULT_MAX_AVERAGE_DEVIATION_RATIO * current_variance;
    return has_smaller_consensus_but_is_much_better;
}

static Eigen::Vector3f computeNormalFromTwoRandomNeighbors(std::vector<MarchOctreeNode*>& neighbors, const octomap::point3d& currentNodeHitLocation) {

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, neighbors.size() - 1);
    std::vector<int> indices;

    while (indices.size() < 2) {

        int index = distribution(generator);
        if (std::find(indices.begin(), indices.end(), index) == indices.end())
            indices.push_back(index);
    }

    Eigen::Vector3f normal_candidate;
    octomap::point3d& loc1 = neighbors->getChild(indices[0])->getNodeLocation();
    octomap::point3d& loc2 = neighbors[indices[1]]->getNodeLocation();
    Eigen::Vector3f vec1(loc1.x() - currentNodeHitLocation.x(), loc1.y() - currentNodeHitLocation.y(), loc1.z() - currentNodeHitLocation.z());
    Eigen::Vector3f vec2(loc2.x() - currentNodeHitLocation.x(), loc2.y() - currentNodeHitLocation.y(), loc2.z() - currentNodeHitLocation.z());
    normal_candidate = vec1.cross(vec2);
    normal_candidate.normalize();

    return normal_candidate;
}


static Eigen::Vector3f refineNormalWithLeastSquares(MarchOctreeNode* currentNode, const Eigen::Vector3f& ransacNormal,
                                                    double maxDistanceFromPlane, const std::vector<MarchOctreeNode*>& neighbors) {

    // Compute the centroid of neighbors
    Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);

    for (const auto& neighbor : neighbors) {

        const octomap::point3d& neighborLoc = neighbor->getNodeLocation();
        centroid[0] += neighborLoc.x();
        centroid[1] += neighborLoc.y();
        centroid[2] += neighborLoc.z();
    }
    centroid /= static_cast<float>(neighbors.size());
    Eigen::Matrix3f covarianceMatrix = Eigen::Matrix3f::Zero();

    for (const auto& neighbor : neighbors) {

        const octomap::point3d& neighborLoc = neighbor->getNodeLocation();
        Eigen::Vector3f vec(neighborLoc.x() - centroid[0], neighborLoc.y() - centroid[1], neighborLoc.z() - centroid[2]);
        covarianceMatrix += vec * vec.transpose();
    }
    // Compute SVD of the covariance matrix
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covarianceMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f refinedNormal = svd.matrixV().col(2);
    // Normalize the refined normal vector
    refinedNormal.normalize();
    return refinedNormal;
}

static void computeNormalConsensusAndVariance(MarchOctreeNode* currentNode, const Eigen::Vector3f& planeNormal,
                                              int hitsAtCurrentPoint, const std::vector<MarchOctreeNode*>& neighbors,
                                              double& varianceToPack, int& consensusToPack)
{
    varianceToPack = 0.0;
    consensusToPack = 0;
    Eigen::Vector3f toNeighborHitLocation;

    if (DEFAULT_WEIGHT_BY_NUMBER_OF_HITS) {

        // Weighted computation of consensus and variance
        for (const auto& neighbor : neighbors) {

            const octomap::point3d& neighborLoc = neighbor->getNodeLocation();
            toNeighborHitLocation << neighborLoc.x() - currentNode->getNodeLocationX(),
                                     neighborLoc.y() - currentNode->getNodeLocationY(),
                                     neighborLoc.z() - currentNode->getNodeLocationZ();
            double distanceFromPlane = std::abs(planeNormal.dot(toNeighborHitLocation));

            if (distanceFromPlane <= DEFAULT_MAX_DISTANCE_FROM_PLANE) {

                double weight = neighbor->getNumberOfHits();
                varianceToPack += weight * distanceFromPlane * distanceFromPlane;
                consensusToPack += weight;
            }
        }
    } else {
        // Non-weighted computation of consensus and variance
        for (const auto& neighbor : neighbors) {

            const octomap::point3d& neighborLoc = neighbor->getNodeLocation();
            toNeighborHitLocation << neighborLoc.x() - currentNode->getNodeLocationX(),
                                     neighborLoc.y() - currentNode->getNodeLocationY(),
                                     neighborLoc.z() - currentNode->getNodeLocationZ();
            double distanceFromPlane = std::abs(planeNormal.dot(toNeighborHitLocation));

            if (distanceFromPlane <= DEFAULT_MAX_DISTANCE_FROM_PLANE) {

                varianceToPack += distanceFromPlane * distanceFromPlane;
                consensusToPack++;
            }
        }
    }
}

} // namespace octomap

