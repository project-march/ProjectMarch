#ifndef MARCH_OCTREE_UTILS_HPP
#define MARCH_OCTREE_UTILS_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <iostream>
#include <march_octree/march_octree.h>
#include <Eigen/Core>


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
public: 

    // Normal Estimation parameters (primarily RANSAC + least squares)
    //TODO: Add parameter to choose which method?
    static constexpr double DEFAULT_SEARCH_RADIUS               = 0.08;
    static constexpr double DEFAULT_MAX_DISTANCE_FROM_PLANE     = 0.02;
    static constexpr double DEFAULT_MIN_CONSENSUS_RATIO         = 0.5;
    static constexpr double DEFAULT_MAX_AVERAGE_DEVIATION_RATIO = 0.75;
    static constexpr int    DEFAULT_NUMBER_OF_ITERATIONS        = 1;
    static constexpr bool   DEFAULT_LEAST_SQUARES_ESTIMATION    = true;
    static constexpr bool   DEFAULT_WEIGHT_BY_NUMBER_OF_HITS    = true;

private:
    // static void computeNormalRANSAC();
    // static void computeNormalFromTwoRandomNeighbors();
    // static std::vector<float> refineNormalWithLeastSquares();
    // static bool peekBestNormal();
    // static bool isCandidateNormalBetter();
    // static void computeNormalConsensusAndVariance();
    // static void computeNormalPCA();
    // static std::vector<MarchOctree> searchNeighbors(MarchOctreeNode* root, MarchOctreeNode* current_node, double search_radius);
    // static bool overlaps(MarchOctreeNode& node, double x, double y, double z, double radius, double squareRadius);
    // static bool contains(MarchOctreeNode& node, double x, double y, double z, double squareRadius);
    // static bool inside(MarchOctreeNode& node, double x, double y, double z, double radius);
    // static void findRadiusNeighbors(const MarchOctreeNode& rootNode, double x, double y, double z, double radius,
    //                                 std::shared_ptr<NeighborActionRule> actionRule);
    // static void findNearestNeighbor(const MarchOctreeNode& rootNode, double x, double y, double z, double minDistance,
    //                                 double maxDistance, OcTreeKey& nearestNeighborKeyToPack);
    // static void findNearestNeighborHelper(const MarchOctreeNode& node, double x, double y, double z, double& nearestDistanceSquared,
    //                                       double squareMinDistance, double squareMaxDistance, OcTreeKey& nearestNeighborKeyToPack);

    template <typename Distance>
    static bool contains(const point3d& query, float sqRadius, const MarchOctreeNode* node);

};  

}
#endif // MARCH_OCTREE_UTILS_HPP
