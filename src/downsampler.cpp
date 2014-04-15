//#include "PointCloudMatcher.h"// ????

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h> // Um die Punktewolke zu visualisieren! (funktioniert noch nicht)

#include <iostream>                         // Speicherung in pcd-file
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>                  // "        "
#include <pcl/point_types.h>                // "        "

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/filters/voxel_grid.h>


int main(int argc, char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read ("test.pcd", *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
         << " data points (" << pcl::getFieldsList (*cloud) << ").";

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
         << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    pcl::PCDWriter writer;
    writer.write ("test_downsampled.pcd", *cloud_filtered,
           Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);


return (0);


}


