/* This program continously takes messages from INS and LiDAR and gives geo-referenced point cloud.
   The tutorial is taken from https://gist.github.com/alexsleat/1372845/7e39518cfa12ac91aca4378843e55862eb9ed41d
   and http://wiki.ros.org/message_filters#Example_.28Python.29-1
*/

#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <limits>
#include <utility>

#include <vector>
#include <cmath>
#include <string.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>

#include </home/vm/catkin_ws/src/geo_referencing/include/geo_referencing/geo_functions.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const sensor_msgs::NavSatFix::ConstPtr &ins_msg, const sensor_msgs::PointCloud2::ConstPtr &msg)
{

    sensor_msgs::NavSatFix gps_data = *ins_msg;
    double lat = gps_data.latitude;
    double lon = gps_data.longitude;
    double height = gps_data.altitude;
    std::cout << "Lat lon height is " << lat << " " << lon << " " << height << std::endl;
   // std::cout << "\n" <<std::endl;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *input_cloud = cloud;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(.5);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(input_cloud->points[*pit]); //*
        }
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        std::vector<double> cen_enu = {centroid[0], centroid[1], centroid[2]};
        std::vector<double> orgllh = {lat, lon, height}; // Input lLiDAR's llh should be taken from INS

        std::vector<double> cen_llh = enu2llh(cen_enu, orgllh);

        std::cout.precision(15);
        std::cout << "Obj " << j << std::fixed << " locations are " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << cen_llh[0] << " " << cen_llh[1] << " " << cen_llh[2] << std::endl;
        
        j++;
        if (j > 10)
        {
            break;
        }
    }
    std::cout << "\n\n\n"<<std::endl; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "geo_referencing");
    ros::NodeHandle nh;
    std::cout << "About to setup callback\n";

    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/published_topic", 10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> ins_sub(nh, "/gps_topic", 10);

    //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, sensor_msgs::PointCloud2> sync(ins_sub, lidar_sub, 10);

    //sync.registerCallback(boost::bind(&callback, _1, _2));
    typedef sync_policies::ApproximateTime<NavSatFix, PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), ins_sub, lidar_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    std::cout << "Here at the end \n";

    ros::spin();
    return 0;
}
