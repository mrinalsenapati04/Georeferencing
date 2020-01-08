#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/registration/icp.h>
#include <math.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <octomap/octomap.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/PointIndices.h>
#include <ctime>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <ctime>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <ros/ros.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_key.h>
#include <pcl/kdtree/kdtree.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/people/head_based_subcluster.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl/people/head_based_subcluster.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

int count = 1;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<PointCloud>("/published_topic", 1);
    pub2_ = n_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/chatter", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/velodyne_points", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {

    //Sensor msgs to pointcloud2 pointer
    pcl::PointCloud<pcl::PointXYZI> cloud1;
    pcl::fromROSMsg(*msg, cloud1);
    //pcl::PCLPointCloud2Ptr cloud(new pcl::PCLPointCloud2());
    //pcl::toPCLPointCloud2(cloud1, *cloud);

    //-----------------------------------Ground Removed---------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud = cloud1;

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    ros::Time::init();
    ros::Time begin = ros::Time::now();
    // std::cout<<begin<<std::endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    int count_groud = 0;
    float xmax, xmin, ymax, ymin;
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {

      if (i == 0)
      {
        xmax = cloud->points[i].x;
        xmin = cloud->points[i].x;
        ymax = cloud->points[i].y;
        ymin = cloud->points[i].y;
      }
      else
      {
        if (xmax <= cloud->points[i].x)
          xmax = cloud->points[i].x;
        if (xmin >= cloud->points[i].x)
          xmin = cloud->points[i].x;
        if (ymax <= cloud->points[i].y)
          ymax = cloud->points[i].y;
        if (ymin >= cloud->points[i].y)
          ymin = cloud->points[i].y;
      }
    }

    //std::cout<<"range of x valoues"<<xmin<<"to"<<xmax<<std::endl;
    //std::cout<<"range of y valoues"<<ymin<<"to"<<ymax<<std::endl;
    float xminround = ceilf(xmin - 1);
    float xmaxround = ceilf(xmax + 1);
    float yminround = ceilf(ymin - 1);
    float ymaxround = ceilf(ymax + 1);
    int xbin, ybin;
    float d = 0.5;
    xbin = (xmaxround - xminround) / d;
    ybin = (ymaxround - yminround) / d;
    std::cout << "Number of Bins " << xbin << "   " << ybin << std::endl;
    static std::vector<int> vect[1000][1200];

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      int dx, dy;
      pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
      dx = (pt.x - xmin) / d;
      dy = (pt.y - ymin) / d;
      //std::cout<<dx<<"   "<<dy<<std::endl;
      vect[dx][dy].push_back(i);
    }
   
    float zmin[xbin + 1][ybin + 1];
    float zmax[xbin + 1][ybin + 1];
    float zavg[xbin + 1][ybin + 1];
    for (int i = 1; i <= xbin; ++i)
      for (int j = 1; j <= ybin; ++j)
      {
        zavg[i][j] = 0;
        int count = 0;
        for (int k = 0; k < vect[i][j].size(); ++k)
        {
          zavg[i][j] = zavg[i][j] + cloud->points[vect[i][j][k]].z;
          if (count == 0)
          {
            zmin[i][j] = cloud->points[vect[i][j][k]].z;
            ros::Time begin = ros::Time::now();
            //std::cout<<begin<<std::endl;
            zmax[i][j] = cloud->points[vect[i][j][k]].z;
            count = count + 1;
          }
          else
          {
            if (zmin[i][j] >= cloud->points[vect[i][j][k]].z)
              zmin[i][j] = cloud->points[vect[i][j][k]].z;
            if (zmax[i][j] <= cloud->points[vect[i][j][k]].z)
              zmax[i][j] = cloud->points[vect[i][j][k]].z;
          }
        }
        zavg[i][j] = zavg[i][j] / vect[i][j].size();
        //std::cout<<"avg z for"<<i<<"and"<<j<<"is equal to"<<zavg[i][j]<<"     ";
      }
    float th1 = 0.2;
    float th2 = 0.3;
    float th3 = 0.1;
    float ground_threshold = 0.1;
    float f = 5;
    int count_ground = 0;

    for (int i = 1; i <= xbin; ++i)
      for (int j = 1; j <= ybin; ++j)
      {
        if ((zmin[i][j] < th1) && (zmax[i][j] - zmin[i][j] > th2))
        {
          for (int k = 0; k < vect[i][j].size(); ++k)
          {

            if ((cloud->points[vect[i][j][k]].z < zmin[i][j] + ground_threshold))
            {
              inliers->indices.push_back(vect[i][j][k]);
              count_groud++;
            }
          }
        }
        else if ((zmin[i][j] < th1) && (th3 < zmax[i][j] - zmin[i][j]) && (zmax[i][j] - zmin[i][j] > th2))
        {
          for (int k = 0; k < vect[i][j].size(); ++k)
          {

            if (cloud->points[vect[i][j][k]].z < (zmin[i][j] + (zmax[i][j] - zmin[i][j]) / f))
            {
              inliers->indices.push_back(vect[i][j][k]);
              count_groud++;
            }
          }
        }
        else if ((zmin[i][j] < th1) && (th3 > zmax[i][j] - zmin[i][j]))
        {
          for (int k = 0; k < vect[i][j].size(); ++k)
          {

            inliers->indices.push_back(vect[i][j][k]);
            count_groud++;
          }
        }
        else
          continue;
      }

    pcl::PointCloud<pcl::PointXYZI> cloud_filtered;
    std::cout << "number of ground points" << count_groud << std::endl;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(cloud_filtered);

    ros::Time time_at_end = ros::Time::now();
    //std::cout<<time_at_end<<std::endl;
    std::cout << "time for ground removal " << time_at_end - begin << std::endl;

    //------------------------------------------------------------------------------------------
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCLPointCloud2Ptr cloud3(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(cloud_filtered, *cloud3);
    // *cloud3=cloud_filtered;
    pcl::PointCloud<pcl::PointXYZI> cloud_msgs;
    pcl::fromPCLPointCloud2(*cloud3, cloud_msgs);
    sensor_msgs::PointCloud2 rosCloud;
    pcl::toROSMsg(cloud_msgs, rosCloud);

    //--------------------------------------------------------------------------------------------
    //.... do something with the input and generate the output...

    // clear the vector
    for (int i = 1; i <= xbin; ++i)
      for (int j = 1; j <= ybin; ++j)
      {
        vect[i][j].clear();
      }
    std::cout << "vector cleared" << std::endl;

    //------------------------------------------------------------------------------------------------------------------
    rosCloud.header.frame_id="/pub_link";
    rosCloud.header.stamp=ros::Time::now();
    pub_.publish(rosCloud);
    //pub2_.publish(array);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher pub2_;
  ros::Subscriber sub_;

}; //End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
