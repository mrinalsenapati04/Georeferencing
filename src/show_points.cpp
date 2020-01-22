

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

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    
    for (int i=0;i<cloud.points.size();i++){
        std::cout.precision(15);
        std::cout <<"The object " << i<<" locations are : "<<std::fixed<< cloud.points[i].x << " " <<cloud.points[i].y<<" "<<cloud.points[i].z<<std::endl;
    }
    std::cout << "\n\n"<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "geo_referencing");
    ros::NodeHandle nh;
    std::cout << "About to setup callback\n";

    ros::Subscriber sub = nh.subscribe("/georeferenced_topic", 1, cloud_cb);

    ros::spin();
    return 0;
}