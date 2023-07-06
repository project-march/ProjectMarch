//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"
#include "rclcpp/rclcpp.hpp"


FuzzyGenerator::FuzzyGenerator()
{

    std::string config_file_path = ament_index_cpp::get_package_share_directory("fuzzy_generator")
    + PATH_SEPARATOR + "config" + PATH_SEPARATOR + "joints.yaml";

    config_ = YAML::LoadFile(config_file_path);
    
    descending_lower_bound = config_["bounds"]["descending_lower_bound"].as<double>();
    descending_upper_bound = config_["bounds"]["descending_upper_bound"].as<double>();
    ascending_lower_bound = config_["bounds"]["ascending_lower_bound"].as<double>();
    ascending_upper_bound = config_["bounds"]["ascending_upper_bound"].as<double>();
};


std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::calculateWeights(std::vector<double> both_foot_heights){
    
    // get the joints, with their torque ranges from the yaml
    std::vector<std::tuple<std::string, float, float>> torque_ranges = getTorqueRanges();
    std::vector<std::tuple<std::string, float, float>> joints;

    // get stance leg
    std::string stance_leg = getStanceLeg(both_foot_heights);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "stance leg: " << stance_leg);

    if(stance_leg.compare("left") == 0){
        // if left is the stance leg, the heights are converted regarding the right foot
        both_foot_heights[1] += std::abs(both_foot_heights[0]);
        both_foot_heights[0] += std::abs(both_foot_heights[0]);
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "left height: " << both_foot_heights[0] << " right height: " << both_foot_heights[1]);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "pushing back heights ");
    log[0].push_back(both_foot_heights[0]);
    log[1].push_back(both_foot_heights[1]);
    // if not, then the feet height are just as they are

    // for each joint in the leg, calculate the torque weight and position weight
    for(auto t: torque_ranges){
        std::string joint_name = std::get<0>(t);
        float minimum_torque_percentage = std::get<1>(t);
        float maximum_torque_percentage = std::get<2>(t);

        float torque_range = maximum_torque_percentage - minimum_torque_percentage;

        // getting the correct foot height and leg
        float foot_height;
        std::string current_leg;

        if(joint_name.find("left") != std::string::npos){
            foot_height = both_foot_heights[0];
            current_leg = "left";
        }
        else if(joint_name.find("right") != std::string::npos){
            foot_height = both_foot_heights[1];
            current_leg = "right";
        }
        else{
            foot_height = both_foot_heights[0];
            current_leg = "left";
            RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "We could not determine if joint " << joint_name << " was left or right. Assuming test joint and using left foot height");
        }

        // getting the correct bounds
        // if the current leg has ascending desired positions,  then we use the ascending bounds
        // otherwise we use the descending bounds
        double lower_bound;
        double upper_bound;

        if(isAscending(current_leg)){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), current_leg << " leg is ascending");
            lower_bound = ascending_lower_bound;
            upper_bound = ascending_upper_bound;
        } else{
            RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), current_leg << " leg is descending");
            lower_bound = descending_lower_bound;
            upper_bound = descending_upper_bound;
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "lower bound: " << lower_bound << " upper bound: " << upper_bound);

        // calculate far the foot is in the 'fuzzy shifting range'
        float height_percentage = (upper_bound - foot_height)/(upper_bound - lower_bound);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "height percentage: " << height_percentage);

        // calculating the weights for both position and torque
        float torque_weight = torque_range * height_percentage;

        // clamp the torque weight to its min and max percentage
        torque_weight = std::max(minimum_torque_percentage, std::min(torque_weight, maximum_torque_percentage));
        float position_weight = 1 - torque_weight;

        joints.push_back(std::make_tuple(joint_name, position_weight, torque_weight));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "name: " << joint_name << " pos weight: " << position_weight << " torque weight: " << torque_weight);
    }

    return joints;
}

std::string FuzzyGenerator::getStanceLeg(std::vector<double> foot_heights){
    // if the left foot significantly lower than the right foot, then left is the stance leg
    return (foot_heights[1] - foot_heights[0]) > std::numeric_limits<double>::epsilon() ? "left" : "right";
}

bool FuzzyGenerator::isAscending(std::string leg){
    // if the desired positions of the leg are strictly increasing over the z-axis over x iterations, we say the leg is ascending
    // if the foot is staying at the same height, it is safest to assume it is on the ground, and we will be ascending soon
    auto equal_and_smaller = [this](float a, float b) -> bool {
        {
            return (a <= b);
        }
    };

    std::vector<float> last_iterations = leg.compare("left") == 0 ? log[0] : log[1];

    if(last_iterations.size() <= 1){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "not enough data: " << last_iterations.size());
        return leg.compare("left") == 0 ? true : false;     
    }

    float deltaZ = last_iterations.back() - last_iterations.end()[-2];
    delta_avg = (delta_avg * alpha) + (deltaZ * (1 - alpha));

    if(delta_avg >= 0){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), " is ascending");
        return true;
    } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), " is descending");
        return false;     
    }


    // // the first few iterations we only set the left leg as swing and the right as stance
    // if(last_iterations.size() < height_history_size){
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "not enough data: " << last_iterations.size());
    //     return leg.compare("left") == 0 ? true : false;
    // }

    // if(last_iterations[last_iterations.size()-height_history_size] <= last_iterations.back()){
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), " is ascending");
    //     return true;
    // } else {
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), " is descending");
    //     return false;
    // }

}

std::vector<std::tuple<std::string, float, float>>  FuzzyGenerator::getTorqueRanges(){
    std::vector<std::tuple<std::string, /*position_weight=*/float, /*torque_weight=*/float>> joints;

    YAML::Node joints_config = config_["joints"];
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "we got " << leg << " joints:");

    for(YAML::const_iterator it=joints_config.begin();it!=joints_config.end();++it) {
        std::string joint_name = it->first.as<std::string>();
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "" << joint_name);

        float min_torque = joints_config[joint_name]["minimum_torque"].as<float>();
        float max_torque = joints_config[joint_name]["maximum_torque"].as<float>();
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "with min torque " << min_torque);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "with max torque " << max_torque);

        joints.push_back(std::make_tuple(joint_name, min_torque, max_torque));
    }
    return joints;
}