#ifndef MARCH_OCTREE_UTILS_HPP
#define MARCH_OCTREE_UTILS_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <iostream>
#include <eigen3/Eigen/Core>

namespace octomap {

class MarchOctreeUtils {

    // Manhattan distance
    struct L1Distance { 

        static inline float compute(const point3d& p, const point3d& q){
            float diff1 = p.x() - q.x();
            float diff2 = p.y() - q.y();
            float diff3 = p.z() - q.z();
            return std::abs(diff1) + std::abs(diff2) + std::abs(diff3);
        }
        static inline float norm(float x, float y, float z){
            return std::abs(x) + std::abs(y) + std::abs(z);
        }
        static inline float sqr(float r) { return r;}
        static inline float sqrt(float r){ return r;}
    };

    // Euclidean distance
    struct L2Distance { 

        static inline float compute(const point3d& p, const point3d& q){
            float diff1 = p.x() - q.x();
            float diff2 = p.y() - q.y();
            float diff3 = p.z() - q.z();
            return std::pow(diff1, 2) + std::pow(diff2, 2) + std::pow(diff3, 2);
        }
        static inline float norm(float x, float y, float z){
            return std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2);
        }
        static inline float sqr(float r){ return r * r;}
        static inline float sqrt(float r) { return std::sqrt(r);}
    };

    // Chebyshev distance or L(infinity)
    struct MaxDistance {

        static inline float compute(const point3d& p, const point3d& q){
            float diff1 = p.x() - q.x();
            float diff2 = p.y() - q.y();
            float diff3 = p.z() - q.z();
            float maximum = diff1;
            if (diff2 > maximum) maximum = diff2;
            if (diff3 > maximum) maximum = diff3;
            return maximum;
        }
        static inline float norm(float x, float y, float z){
            float maximum = x;
            if (y > maximum) maximum = y;
            if (z > maximum) maximum = z;
            return maximum;
        }
        static inline float sqr(float r){ return r;}
        static inline float sqrt(float r){ return r;}
    };


private:

    // Normal Estimation parameters (primarily RANSAC + least squares)
    //TODO: Add parameter to choose which method?
    static const double DEFAULT_SEARCH_RADIUS;            
    static const double DEFAULT_MAX_DISTANCE_FROM_PLANE;
    static const double DEFAULT_MIN_CONSENSUS_RATIO;
    static const double DEFAULT_MAX_AVERAGE_DEVIATION_RATIO;
    static const int    DEFAULT_NUMBER_OF_ITERATIONS;
    static const bool   DEFAULT_LEAST_SQUARES_ESTIMATION;
    static const bool   DEFAULT_WEIGHT_BY_NUMBER_OF_HITS;

    static void findRadiusNeighbors(const octomap::MarchOctreeNode *node, octomap::point3d query_point, double search_radius,
                                    std::vector<std::seed_seq::result_type> &neighborsIndices);
    static void findRadiusNeighbors(const MarchOctreeNode* node, const point3d query_point, double search_radius, 
                                    std::function<void(MarchOctreeNode*)> recursiveAction);
    static std::vector<MarchOctreeNode*> searchNeighbors(const MarchOctree* root, MarchOctreeNode* current_node);
    static void computeNodeNormalRANSAC(const MarchOctree* root, const octomap::OcTreeKey& key, int tree_depth);
    static void computeNodeNormalRANSAC(const MarchOctree* root, MarchOctreeNode* current_node);
    static void peekBestNormal(const MarchOctreeNode* node, const Eigen::Vector3f& current_normal, double& current_variance, int& current_consensus,
                           Eigen::Vector3f& candidate_normal, double candidate_variance, int candidate_consensus);
    static void peekBestNormal(MarchOctreeNode* node, const Eigen::Vector3f& current_normal, double& current_variance, int& current_consensus,
                           Eigen::Vector3f& candidate_normal, double candidate_variance, int candidate_consensus);
    static bool isCandidateNormalBetter(double current_variance, int current_consensus, double candidate_variance, int candidate_consensus);
    static Eigen::Vector3f computeNormalFromTwoRandomNeighbors(std::vector<MarchOctreeNode*>& neighbors, const octomap::point3d& current_node_location);
    static Eigen::Vector3f refineNormalWithLeastSquares(const MarchOctreeNode* current_node, const Eigen::Vector3f& ransac_normal,
                                                        const std::vector<MarchOctreeNode*>& neighbors);
    static void computeNormalConsensusAndVariance(const MarchOctreeNode* current_node, const Eigen::Vector3f& plane_normal,
                                              int hits_at_current_point, const std::vector<MarchOctreeNode*>& neighbors,
                                              double& variance_to_pack, int& consensus_to_pack);

    template <typename Distance>
    static bool contains(const MarchOctreeNode* node, const point3d& query, float squared_radius);

};  

}
#endif // MARCH_OCTREE_UTILS_HPP
