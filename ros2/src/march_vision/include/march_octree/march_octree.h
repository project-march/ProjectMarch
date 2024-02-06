#ifndef MARCH_OCTREE_H
#define MARCH_OCTREE_H

#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeIterator.hxx>
#include <march_octree/march_octree_node.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <ctime>
#include <set>
#include <eigen3/Eigen/Core>
#include <pcl/point_cloud.h>

namespace octomap {
class MarchOctree : public OccupancyOcTreeBase<MarchOctreeNode> {
  
public:

    //TODO: Add min/max insert range?

    // Occupancy parameters as constants
    static constexpr double DEFAULT_OCCUPANCY_THRESHOLD = 0.5;   // = 0.0 in logodds
    static constexpr double DEFAULT_HIT_UPDATE          = 0.7;   // = 0.85 in logodds
    static constexpr double DEFAULT_MISS_UPDATE         = 0.4;   // = -0.4 in logodds
    static constexpr double DEFAULT_MIN_PROBABILITY     = 0.1192;// = -2 in log odds
    static constexpr double DEFAULT_MAX_PROBABILITY     = 0.971; // = 3.5 in log odds

    // Normal Estimation parameters (primarily RANSAC + least squares)
    //TODO: Add parameter to choose which method?
    static constexpr double DEFAULT_SEARCH_RADIUS               = 0.08;
    static constexpr double DEFAULT_MAX_DISTANCE_FROM_PLANE     = 0.02;
    static constexpr double DEFAULT_MIN_CONSENSUS_RATIO         = 0.5;
    static constexpr double DEFAULT_MAX_AVERAGE_DEVIATION_RATIO = 0.75;
    static constexpr int    DEFAULT_NUMBER_OF_ITERATIONS        = 1;
    static constexpr bool   DEFAULT_LEAST_SQUARES_ESTIMATION    = true;
    static constexpr bool   DEFAULT_WEIGHT_BY_NUMBER_OF_HITS    = true;

    //TODO: Change this constructor to the one that inherits 
    MarchOctree(double in_resolution);
    ~MarchOctree() = default;

    /// virtual constructor: creates a new object of same type
    MarchOctree* create() const {return new MarchOctree(resolution); }

    //TODO: Should these be "const"?
    unsigned int getLastUpdateTime();
    void degradeOutdatedNodes(unsigned int time_thres);
    void updateNodeLogOdds(MarchOctreeNode* node, const float& update);
    void integrateMissNoTime(MarchOctreeNode* node);

    void update(octomap::Pointcloud& point_cloud);
    void insertNewScan(octomap::Pointcloud& point_cloud, bool insert_miss, std::set<MarchOctreeNode*>& updated_leaves_to_pack, 
                       std::set<OcTreeKey>& deleted_leaves_to_pack);
    
    void updateNormals();
    void updateNodesNormals(const std::vector<MarchOctreeNode*>& nodes_to_update);
    void clearNormals();
    void updateInnerNormalsRecursive(MarchOctreeNode* node, int depth);

    void enableReportTime(bool enable);
    void setNodeMaximumNumberOfHits(unsigned long maximum_number_of_hits);
    void enableParallelComputationForNormals(bool enable);
    void enableParallelInsertionOfMisses(bool enable);

    std::array<double, 5> getOccupancyParameters();
    std::array<double, 7> getNormalEstimationParameters();
   
    //TODO: fix this 
    //void updateHitTimestamp(long timestamp);

private:

    //TODO: Implement the parallel computations and test them
    bool compute_normals_in_parallel;
    bool insert_misses_in_parallel;
    bool report_time;
    
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        MarchOctree* tree = new MarchOctree(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
      }
      void ensureLinking() {}
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer marchOctreeMemberInit;

  };
} // namespace octomap 
#endif // MARCH_OCTREE_H
