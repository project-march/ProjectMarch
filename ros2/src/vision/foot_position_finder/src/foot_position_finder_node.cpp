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

// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include "stb_image_write.h"

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

int main()
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

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    viewer->addCoordinateSystem (1.0);

    // PointCloud::Ptr pointcloud(new PointCloud());
    // NormalCloud::Ptr normalcloud(new NormalCloud());
    // pcl::io::loadPLYFile("datasets/stairs4.ply", *pointcloud);

    auto s0 = std::chrono::high_resolution_clock::now();
    auto s1 = std::chrono::high_resolution_clock::now();

    viewer->spinOnce();

    std::vector<Point> found_points;

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        // rs2::frame color = frames.get_color_frame();
        rs2::frame filtered = depth;

        // if (auto vf = color.as<rs2::video_frame>())
        // {
        //     auto stream = color.get_profile().stream_type();
        //     if (vf.is<rs2::depth_frame>()) vf = color_map.process(color);

        //     // Write images to disk
        //     std::stringstream png_file;
        //     png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
        //     stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
        //                    vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
        //     std::cout << "Saved " << png_file.str() << std::endl;
        // }


        // stbi_write_png("test.png", vf.get_width(), vf.get_height(),
        //        vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());

        filtered = dec_filter.process(filtered);
        filtered = spat_filter.process(filtered);
        filtered = temp_filter.process(filtered);
        
        s0 = std::chrono::high_resolution_clock::now();
        points = pc.calculate(filtered);

        PointCloud::Ptr pointcloud = points_to_pcl(points);
        NormalCloud::Ptr normalcloud(new NormalCloud());
        NormalsPreprocessor preprocessor(pointcloud, normalcloud);
        preprocessor.preprocess();

        std::vector<double> search_region = {-0.5, 0.5, 0, 1, -2, 2};
        FootPositionFinder position_finder(pointcloud, search_region, 'l');
        std::vector<Point> position_queue;
        position_finder.findFootPositions(&position_queue);

        s1 = std::chrono::high_resolution_clock::now();
        std::cout << "Total " << 1.e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(s1 - s0).count() << std::endl;

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        viewer->addPointCloud<pcl::PointXYZ> (pointcloud, "pc");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pc");

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
                    viewer->addSphere(avg_final, 0.02, 1.0, 0, 0, std::to_string(0));
            }
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(50));


        // for (std::size_t i = 0; i < position_queue.size(); i++)
        // {
        //     viewer->addSphere(position_queue[i], 0.02, 1.0, 0, 0, std::to_string(i));
        //     break;
        // }



        // auto heights = position_finder.getHeights();
        // auto derivatives_ = position_finder.getDerivatives();

        // PointCloud::Ptr flat = points_to_pcl(points);
        // flat->points.resize(30 * 30);

        // int count = 0;
        // for (int i = 0; i < 30; i++)
        // {
        //     for (int j = 0; j < 30; j++)
        //     {
        //         if (derivatives_[j][i] < 0.05)
        //         {
        //             (*flat)[count].x = ((double) i /30) - 0.5 + 1/30/2;
        //             (*flat)[count].y = ((double) (30 - j) / 30) - 0 - 1/30/2;
        //             (*flat)[count].z = heights[j][i];
        //             count++;
        //         }
        //     }
        // }

        // flat->points.resize(count);
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(flat, 0, 200, 0);
        // viewer->addPointCloud<pcl::PointXYZ> (flat, single_color, "flat");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "flat");

        viewer->spinOnce();
    }
    
    return 0;
}
