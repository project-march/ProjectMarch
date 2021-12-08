#include <librealsense2/rs.hpp>
#include <iostream>
#include <algorithm> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include "utilities/realsense_to_pcl.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include "preprocessor.h"
#include "foot_position_finder.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include "std_msgs/String.h"

std::string TOPIC_CAMERA_FRONT = "/camera_front/depth/color/points";
std::string TOPIC_TEST_CLOUDS = "/test_clouds";

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;


void pointcloudCallback(const sensor_msgs::PointCloud2 input_cloud)
{
    PointCloud converted_cloud;
    pcl::fromROSMsg(input_cloud, converted_cloud);
    PointCloud::Ptr pointcloud = boost::make_shared<PointCloud>(converted_cloud);
    // processPointCloud(pointcloud);
    ROS_INFO("I heard something from the simulation");
}

void processPointCloud(PointCloud::Ptr pointcloud)
{
    std::vector<Point> found_points;

    NormalCloud::Ptr normalcloud(new NormalCloud());
    NormalsPreprocessor preprocessor(pointcloud, normalcloud);
    preprocessor.preprocess();

    std::vector<double> search_region = {-0.5, 0.5, 0, 1, -2, 2};
    FootPositionFinder position_finder(pointcloud, search_region, 'l');
    std::vector<Point> position_queue;
    position_finder.findFootPositions(&position_queue);

    unsigned int N = 5;
    if (position_queue.size() > 0) {
        Point p = position_queue[0];
        if (found_points.size() < N)
            found_points.push_back(p);
        else {
            std::rotate(found_points.begin(), found_points.begin() + 1, found_points.end());
            found_points[N-1] = p;

            double x_avg = 0, y_avg = 0, z_avg = 0;

            for (Point &p : found_points) {
                x_avg += p.x; y_avg += p.y; z_avg += p.z;
            }

            x_avg /= N; y_avg /= N; z_avg /= N;
            Point avg(x_avg, y_avg, z_avg);

            std::vector<Point> non_outliers;

            for (Point &p : found_points) {
                if (sqrt(pow(p.x - avg.x, 2) + pow(p.y - avg.y, 2) +  pow(p.z - avg.z, 2)) < 0.05)
                    non_outliers.push_back(p);
            }
            if (non_outliers.size() < N)
                std::cout << non_outliers.size() << std::endl;
            x_avg = 0, y_avg = 0, z_avg = 0;
            for (Point &p : non_outliers) {
                x_avg += p.x; y_avg += p.y; z_avg += p.z;
            }
            x_avg /= N; y_avg /= N; z_avg /= N;
            Point avg_final(x_avg, y_avg, z_avg);
            if (non_outliers.size() == N)
            ROS_INFO("Hello world");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_realsense_reader");
    ros::NodeHandle n_;
    ros::Publisher point_publisher_ = n_.advertise<std_msgs::String>("foot_position", 100);

    bool simulation = true;

    if (!simulation)
    {
        rs2::pointcloud pc;
        rs2::points points;

        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        // cfg.enable_stream(StreamType.COLOR, 640, 480, StreamFormat.RGB8)
        pipe.start(cfg);

        // Declare filters
        rs2::decimation_filter dec_filter;
        rs2::spatial_filter spat_filter;
        rs2::temporal_filter temp_filter;

        while (ros::ok())
        {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::depth_frame depth = frames.get_depth_frame();
            rs2::frame filtered = depth;

            filtered = dec_filter.process(filtered);
            filtered = spat_filter.process(filtered);
            filtered = temp_filter.process(filtered);
        
            points = pc.calculate(filtered);
            PointCloud::Ptr pointcloud = points_to_pcl(points);

            ROS_INFO("I heard something from the Realsense");
            // processPointCloud(pointcloud);
            ros::spinOnce();
        }
    }
    else
    {
        ros::Subscriber pointcloud_subscriber_ = n_.subscribe(TOPIC_CAMERA_FRONT, 100, pointcloudCallback);
        ros::spin();
    }

    return 0;
}
