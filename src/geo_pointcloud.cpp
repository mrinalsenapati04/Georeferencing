#include <iostream>
#include <math.h>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ros/ros.h"
#include <time.h>
#include <bits/stdc++.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#define M_PI 3.14159265358979323846

#include </home/vm/catkin_ws/src/geo_referencing/include/geo_referencing/geo_functions.h>

int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    std::string pcd_path = "/home/vm/catkin_ws/src/geo_referencing/test.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from test_pcd.pcd with the following fields: " << std::endl;

    ros::Time::init();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        std::vector<double> cen_enu = {centroid[0], centroid[1], centroid[2]};
        std::vector<double> orgllh = {75.089123, 83.561067, 414.894501}; // Input lLiDAR's llh should be taken from INS
        //ros::Time time1 = ros::Time::now();
        std::vector<double> cen_llh = enu2llh(cen_enu, orgllh);
       // ros::Time time2 = ros::Time::now();
        //std::cout << "execution time is  " << (time2 - time1) << std::endl;
        std::cout.precision(15);
        std::cout << "Obj " << j << std::fixed << " locations are " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << cen_llh[0] << " " << cen_llh[1] << " " << cen_llh[2] << std::endl;
        j++;
    }

    return 0;
}