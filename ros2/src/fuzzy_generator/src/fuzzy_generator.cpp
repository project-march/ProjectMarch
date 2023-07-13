//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"
#include "rclcpp/rclcpp.hpp"

FuzzyGenerator::FuzzyGenerator() = default;

FuzzyGenerator::FuzzyGenerator(std::string config_path)
{
    config_ = YAML::LoadFile(config_path);

    lower_bound = config_["bounds"]["lower_bound"].as<double>();
    upper_bound = config_["bounds"]["upper_bound"].as<double>();
};

std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::calculateWeights(
    std::vector<double> both_foot_heights)
{

    // get the joints, with their torque ranges from the yaml
    std::vector<std::tuple<std::string, float, float>> torque_ranges = getTorqueRanges();
    std::vector<std::tuple<std::string, float, float>> joints;

    // get stance leg

    both_foot_heights[1] -= both_foot_heights[0];
    both_foot_heights[0] = 0;
    float min = std::min(both_foot_heights[0], both_foot_heights[1]);
    both_foot_heights[0] -= min;
    both_foot_heights[1] -= min;

    // if not, then the feet height are just as they are

    // for each joint in the leg, calculate the torque weight and position weight
    for (auto t : torque_ranges) {
        std::string joint_name = std::get<0>(t);
        float minimum_torque_percentage = std::get<1>(t);
        float maximum_torque_percentage = std::get<2>(t);

        float torque_range = maximum_torque_percentage - minimum_torque_percentage;

        // getting the correct foot height and leg
        float foot_height;
        std::string current_leg;

        if (joint_name.find(/*__s=*/"left") != std::string::npos) {
            foot_height = both_foot_heights[0];
            current_leg = "left";
        } else if (joint_name.find(/*__s=*/"right") != std::string::npos) {
            foot_height = both_foot_heights[1];
            current_leg = "right";
        } else {
            foot_height = both_foot_heights[0];
            current_leg = "left";
            RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"),
                "We could not determine if joint "
                    << joint_name << " was left or right. Assuming test joint and using left foot height");
        }

        // getting the correct bounds
        // if the current leg has ascending desired positions,  then we use the ascending bounds
        // otherwise we use the descending bounds
        float lower_bound;
        float upper_bound;

        if (isAscending(current_leg)) {
            lower_bound = (float)ascending_lower_bound;
            upper_bound = (float)ascending_upper_bound;
        } else {
            lower_bound = (float)descending_lower_bound;
            upper_bound = (float)descending_upper_bound;
        }

        // calculate far the foot is in the 'fuzzy shifting range'
        float height_percentage = (upper_bound - foot_height) / (upper_bound - lower_bound);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "height percentage: " << height_percentage);

        // calculating the weights for both position and torque
        float torque_weight = torque_range * height_percentage;

        // clamp the torque weight to its min and max percentage
        torque_weight = std::max(minimum_torque_percentage, std::min(torque_weight, maximum_torque_percentage));
        float position_weight = 1 - torque_weight;

        joints.emplace_back(std::make_tuple(joint_name, position_weight, torque_weight));
    }
    return joints;
}

std::string FuzzyGenerator::getStanceLeg(std::vector<double> foot_heights)
{
    // if the left foot significantly lower than the right foot, then left is the stance leg
    return (foot_heights[1] - foot_heights[0]) > std::numeric_limits<double>::epsilon() ? "left" : "right";
}

void FuzzyGenerator::updateVelocities()
{

    std::vector<float> last_iterations_l = log[0];
    std::vector<float> last_iterations_r = log[1];

    if (last_iterations_l.size() <= 1 || last_iterations_r.size() <= 1) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "not enough data: " << last_iterations_l.size());
        return;
    }

    float deltaZ = last_iterations_l.back() - last_iterations_l.end()[-2];
    delta_avg_l = (delta_avg_l * alpha) + (deltaZ * (1 - alpha));

    deltaZ = last_iterations_r.back() - last_iterations_r.end()[-2];
    delta_avg_r = (delta_avg_r * alpha) + (deltaZ * (1 - alpha));
}

bool FuzzyGenerator::isAscending(std::string leg)
{
    // if the desired positions of the leg are strictly increasing over the z-axis over x iterations, we say the leg is
    // ascending if the foot is staying at the same height, it is safest to assume it is on the ground, and we will be
    // ascending soon

    if (leg.compare(/*__s=*/"left") == 0) {
        return delta_avg_l >= 0;
    } else {
        return delta_avg_r >= 0;
    }
}

std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::getTorqueRanges()
{
    std::vector<std::tuple<std::string, /*position_weight=*/float, /*torque_weight=*/float>> joints;

    YAML::Node joints_config = config_["joints"];

    for (YAML::const_iterator it = joints_config.begin(); it != joints_config.end(); ++it) {
        std::string joint_name = it->first.as<std::string>();

        auto min_torque = joints_config[joint_name]["minimum_torque"].as<float>();
        auto max_torque = joints_config[joint_name]["maximum_torque"].as<float>();

        joints.emplace_back(std::make_tuple(joint_name, min_torque, max_torque));
    }
    return joints;
}