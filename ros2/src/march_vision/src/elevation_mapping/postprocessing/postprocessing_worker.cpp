/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */

#include "elevation_mapping/postprocessing/PostprocessingWorker.hpp"

namespace elevation_mapping {

PostprocessingWorker::PostprocessingWorker(std::shared_ptr<rclcpp::Node> nodeHandle)
    : functor_(nodeHandle), work_(ioService_), thread_(boost::bind(&boost::asio::io_service::run, &ioService_)) {}

PostprocessingWorker::GridMap PostprocessingWorker::processBuffer() {
  return functor_(dataBuffer_);
}

void PostprocessingWorker::publish(const GridMap& gridMap) const {
  functor_.publish(gridMap);
}

bool PostprocessingWorker::hasSubscribers() const {
  return functor_.hasSubscribers();
}

}  // namespace elevation_mapping
