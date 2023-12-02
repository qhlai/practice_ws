//main.cpp
#include <iostream>
#include <assert.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>


using namespace pcl;
using namespace pcl::io;
using PointCloudTYPE  = pcl::PointXYZRGBA;
int main( int argc, char** argv )
{

    // if (argc != 2)
    // {
    //     std::cout<<"error"<<std::endl;
    //     return -1;
    // }
 
    // std::string input_file = argv[1];
    // pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    // pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( input_file, cloud );

    // std::cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<std::endl;
 

    pcl::PLYReader reader;
    pcl::PointCloud<PointCloudTYPE>::Ptr cloud1 (new pcl::PointCloud<PointCloudTYPE>);
    reader.read<PointCloudTYPE>("/home/hanglok/catkin_ws/cloud.ply", *cloud1);
    pcl::io::savePCDFile("/home/hanglok/catkin_ws/cloud.pcd", *cloud1 );

    return 0;
}