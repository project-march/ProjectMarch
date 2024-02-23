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

namespace octomap {
class MarchOctree : public OccupancyOcTreeBase<MarchOctreeNode> {
  
public:

    //TODO: Add min/max insert range?

    // Occupancy parameters as constants
    static const double DEFAULT_OCCUPANCY_THRESHOLD;
    static const double DEFAULT_HIT_UPDATE;
    static const double DEFAULT_MISS_UPDATE;
    static const double DEFAULT_MIN_PROBABILITY;
    static const double DEFAULT_MAX_PROBABILITY;

    // Normal Estimation parameters (primarily RANSAC + least squares)
    //TODO: Add parameter to choose which method?
    static const double DEFAULT_SEARCH_RADIUS;
    static const double DEFAULT_MAX_DISTANCE_FROM_PLANE;
    static const double DEFAULT_MIN_CONSENSUS_RATIO;
    static const double DEFAULT_MAX_AVERAGE_DEVIATION_RATIO;
    static const int    DEFAULT_NUMBER_OF_ITERATIONS;
    static const bool   DEFAULT_LEAST_SQUARES_ESTIMATION;
    static const bool   DEFAULT_WEIGHT_BY_NUMBER_OF_HITS;

    //TODO: Change this constructor to the one that inherits 
    MarchOctree(double in_resolution);
    ~MarchOctree() = default;

    /// virtual constructor: creates a new object of same type
    MarchOctree* create() const {return new MarchOctree(resolution); }

    //TODO: Should these be "const"?
    unsigned int getLastUpdateTime() const;
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

    //TODO: fix this 
    //void updateHitTimestamp(long timestamp);

private:

    //TODO: Implement the parallel computations and test them
    bool m_compute_normals_in_parallel;
    bool m_insert_misses_in_parallel;
    bool m_report_time;
    
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
