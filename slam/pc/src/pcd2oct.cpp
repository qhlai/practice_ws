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
    std::string input_file ="/home/hanglok/catkin_ws/cloud.pcd";
    pcl::PointCloud<PointCloudTYPE> cloud;
    pcl::io::loadPCDFile<PointCloudTYPE> ( input_file, cloud );

    std::cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<std::endl;

    std::cout<<"copy data into octomap..."<<std::endl;
    octomap::OcTree tree( 0.001 );
    for (auto p:cloud.points){
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }
    tree.updateInnerOccupancy();
    tree.writeBinary( "/home/hanglok/catkin_ws/cloud.ot" );
    return 0;
}