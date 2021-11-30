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

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

int main()
{
    rs2::pointcloud pc;
    rs2::points points;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
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

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::frame filtered = depth;

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
        std::cout << "Total " << 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(s1-s0).count() << std::endl;

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        viewer->addPointCloud<pcl::PointXYZ> (pointcloud, "pc");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pc");

        for (std::size_t i = 0; i < position_queue.size(); i++)
        {
            viewer->addSphere(position_queue[i], 0.02, 1.0, 0, 0, std::to_string(i));
            break;
        }

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
