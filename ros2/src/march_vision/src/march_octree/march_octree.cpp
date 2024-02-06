    #include <march_octree/march_octree.h>

namespace octomap {

MarchOctree::MarchOctree(double in_resolution)
: OccupancyOcTreeBase<MarchOctreeNode>(in_resolution) {
    marchOctreeMemberInit.ensureLinking();
}

unsigned int MarchOctree::getLastUpdateTime() {
// this value is updated whenever inner nodes are
// updated using updateOccupancyChildren()
return root->getLastHitTimestamp();
}

void MarchOctree::degradeOutdatedNodes(unsigned int time_thres) {
unsigned int query_time = (unsigned int) time(NULL);

for(leaf_iterator it = this->begin_leafs(), end=this->end_leafs();
    it!= end; ++it) {
    if ( this->isNodeOccupied(*it)
        && ((query_time - it->getLastHitTimestamp()) > time_thres) ) {
            integrateMissNoTime(&*it);
        }
    }
}

void MarchOctree::updateNodeLogOdds(MarchOctreeNode* node, const float& update) const {
    OccupancyOcTreeBase<MarchOctreeNode>::updateNodeLogOdds(node, update);
    //TODO: Fix this 
    //node->updateLastHitTimestamp();
}

void MarchOctree::integrateMissNoTime(MarchOctreeNode* node) const{
    OccupancyOcTreeBase<MarchOctreeNode>::updateNodeLogOdds(node, prob_miss_log);
}

// TODO: Have to see how this will work in the general update loop;
// Updates map with next instance of the point cloud
void MarchOctree::update(octomap::Pointcloud& point_cloud) {
    //TODO: Change the inputs to this method and remove the smaller versions
    insertPointCloud(point_cloud, true);
    updateNormals();
}

void MarchOctree::insertPointCloud(pcl::PointCloud<pcl::PointXYZ>& point_cloud, bool insert_miss, std::set<MarchOctreeNode*>& updated_leaves_to_pack,
                                   std::set<OcTreeKey>& deleted_leaves_to_pack) {
    if (point_cloud.empty())
        return;

    unsigned int query_time = (unsigned int)time(NULL);

    for (const auto& point : point_cloud.points) {

        pcl::PointXYZ point3D = point;

        OcTreeKey key;
        if (!coordToKeyChecked(point3D.x, point3D.y, point3D.z, key))
            continue;

        MarchOctreeNode* node = search(key);
        if (!node) node = updateNode(key, false); 

        if (node) {
            updateNodeLogOdds(node, prob_hit_log); // Update the node with hit probability
            if (updated_leaves_to_pack) updated_leaves_to_pack.insert(node);
            // Remove the key from the deleted set if present
            if (deleted_leaves_to_pack) deletedLeavesToPack.erase(key);
        }
    }

    if (insert_miss) {
        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it) {
            if (this->isNodeOccupied(*it) && ((query_time - it->getLastHitTimestamp()) > time_thres)) {
                integrateMissNoTime(&*it);
                // Insert the key to the deleted set
                if (deleted_leaves_to_pack) deleted_leaves_to_pack.insert(it.getKey());
            }
        }
    }
}

void MarchOctree::updateNormals() {
    std::vector<MarchOctreeNode*> allNodes;
    for (auto it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it) {
        allNodes.push_back(&*it);
    }

    updateNodesNormals(allNodes);
}

void MarchOctree::updateNodesNormals(const std::vector<MarchOctreeNode*>& nodesToUpdate) {
    for (auto node : nodesToUpdate) {
        // TODO: Choose which function and implement it.
        //computeNodeNormals(node);
    }

    if (this->root) {
        updateInnerNormalsRecursive(root, 0);
    }
}

void MarchOctree::clearNormals() {
    for (auto it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it) {
        it->resetNormal();
    }
}

void MarchOctree::updateInnerNormalsRecursive(MarchOctreeNode* node, int depth) {
    if (!node || !node->hasChildren())
        return;

    if (depth < this->getTreeDepth() - 1) {
        for (size_t i = 0; i < 8; ++i) {
            MarchOctreeNode* childNode = node->getChildNode(i);
            if (childNode)
                updateInnerNormalsRecursive(childNode, depth + 1);
        }
    }

    node->updateNormalChildren();
}

void MarchOctree::enableReportTime(bool enable) {
    // Implement enabling or disabling time reporting
}

void MarchOctree::enableParallelComputationForNormals(bool enable) {
    // Implement enabling or disabling parallel computation for normals
}


void MarchOctree::setNodeMaximumNumberOfHits(unsigned long maximumNumberOfHits) {
    // Implement setting the maximum number of hits for nodes
}


void MarchOctree::enableParallelInsertionOfMisses(bool enable) {
    
}

void MarchOctree::setMinimumInsertRange(double minRange) {
    // Implement setting the minimum insert range
}

void MarchOctree::setMaximumInsertRange(double maxRange) {
    // Implement setting the maximum insert range
}

void MarchOctree::setBoundsInsertRange(double minRange, double maxRange) {
    // Implement setting the bounds insert range
}

void MarchOctree::removeMinimumInsertRange() {
    // Implement removing the minimum insert range
}

void MarchOctree::removeMaximumInsertRange() {
    // Implement removing the maximum insert range
}

void MarchOctree::removeBoundsInsertRange() {
    // Implement removing the bounds insert range
}

std::array<double, 5> MarchOctree::getOccupancyParameters(){

    return { DEFAULT_OCCUPANCY_THRESHOLD, DEFAULT_HIT_UPDATE, 
             DEFAULT_MISS_UPDATE, DEFAULT_MIN_PROBABILITY, DEFAULT_MAX_PROBABILITY };
}

std::array<double, 7> MarchOctree::getNormalEstimationParameters(){

    return { DEFAULT_SEARCH_RADIUS, DEFAULT_MAX_DISTANCE_FROM_PLANE, DEFAULT_MIN_CONSENSUS_RATIO,
             DEFAULT_MAX_AVERAGE_DEVIATION_RATIO, DEFAULT_NUMBER_OF_ITERATIONS,
             DEFAULT_LEAST_SQUARES_ESTIMATION, DEFAULT_WEIGHT_BY_NUMBER_OF_HITS };
}

MarchOctree::StaticMemberInitializer MarchOctree::marchOctreeMemberInit;

} // namespace octomap
