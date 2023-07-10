#include "weight_shift_buffer/weight_shift_buffer_node.hpp"

WeightShiftBufferNode::WeightShiftBufferNode()
    : Node("weight_shift_buffer")
{
    
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeightShiftBufferNode>());
    rclcpp::shutdown();
    return 0;
}
