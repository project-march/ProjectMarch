#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

using TransformStamped = geometry_msgs::msg::TransformStamped;

class AlignedFramePublisherNode : public rclcpp::Node {
public:
    AlignedFramePublisherNode()
        : Node("march_aligned_frame_publisher")
    {

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_
            = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            
        publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), [this]() -> void {
            publishAlignedFrames();
        });
    }



private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_ { nullptr };
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr publish_timer_;

    const double FOOT_LENGTH = 0.1825;

    TransformStamped trans_left_;
    TransformStamped trans_right_;
    


    void publishAlignedFrames() {
        
        using namespace geometry_msgs::msg;

        try {
            trans_left_ = tf_buffer_->lookupTransform(
                "foot_left", "world", tf2::TimePointZero);
            trans_right_ = tf_buffer_->lookupTransform(
                "foot_right", "world", tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not compute aligned frames: %s", ex.what());
            return;
        }
        
        // Transformation from left foot to left toes
        TransformStamped tr1;
        tr1.header.stamp = this->get_clock()->now();
        tr1.header.frame_id = "foot_left";
        tr1.child_frame_id = "toes_left";
        tr1.transform.translation.x = -FOOT_LENGTH;
        tr1.transform.translation.y = 0.0;
        tr1.transform.translation.z = -0.025;
        tr1.transform.rotation = Quaternion();
        tf_broadcaster_->sendTransform(tr1);

        // Transformation from left toes to left aligned
        TransformStamped tr2;
        tr2.header.stamp = this->get_clock()->now();
        tr2.header.frame_id = "toes_left";
        tr2.child_frame_id = "toes_left_aligned";
        tr2.transform.rotation = trans_left_.transform.rotation;
        tf_broadcaster_->sendTransform(tr2);

        // Transformation from right foot to right toes
        TransformStamped tr3;
        tr3.header.stamp = this->get_clock()->now();
        tr3.header.frame_id = "foot_right";
        tr3.child_frame_id = "toes_right";
        tr3.transform.translation.x = -FOOT_LENGTH;
        tr3.transform.translation.y = 0.0;
        tr3.transform.translation.z = -0.025;
        tr3.transform.rotation = Quaternion();
        tf_broadcaster_->sendTransform(tr3);

        // Transformation from right toes to right aligned
        TransformStamped tr4;
        tr4.header.stamp = this->get_clock()->now();
        tr4.header.frame_id = "toes_right";
        tr4.child_frame_id = "toes_right_aligned";
        tr4.transform.rotation = trans_right_.transform.rotation;
        tf_broadcaster_->sendTransform(tr4);
    }

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<AlignedFramePublisherNode>();
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
