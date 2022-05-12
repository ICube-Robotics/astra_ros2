#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "shape_msgs/msg/mesh.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <Eigen/Geometry> 


using namespace std::chrono_literals;
using std::placeholders::_1;

class SurfaceNode : public rclcpp::Node
{
  public:
    SurfaceNode(): Node("surface_node")
    {
      cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud", 2, std::bind(&SurfaceNode::topic_callback, this, _1));
      marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("surface", 1);
      cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_filtered", 1);

      
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg.get(),*cloud);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations(1000);
      seg.setDistanceThreshold (0.01);

      int id = 1;
      int initial_size = cloud->size();

      // while (cloud->size () > 0.5 * initial_size)
      for(auto i=0ul;i<2;i++)
      {
        if(i == 1) seg.setModelType (pcl::SACMODEL_CIRCLE3D );
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;

        if (inliers->indices.size () == 0)
        {
          PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
          return;
        }

        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloudF;
        extract.filter(cloudF);
        cloud->swap(cloudF);

        visualization_msgs::msg::Marker marker;
        // meshToMarkerMsg(triangles,marker);
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::CUBE;
        if(i == 1) marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.header.frame_id = "astra";
        
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.001;
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::ADD;

        if(i == 0){
          marker.pose.position.x = 0;
          marker.pose.position.y = 0;
          marker.pose.position.z = -coefficients->values[3]/coefficients->values[2];

          Eigen::Quaternion<double> qu(0,0,0,1);
          Eigen::Quaternion<double> qn(0,coefficients->values[0],coefficients->values[1],coefficients->values[2]);
          auto q = qu*qn;
          q.w() = 1-q.w();
          q.normalize(); 
          marker.pose.orientation.w = q.w();
          marker.pose.orientation.x = q.x();
          marker.pose.orientation.y = q.y();
          marker.pose.orientation.z = q.z();
        }
        if(i == 1){
          marker.pose.position.x = coefficients->values[0];
          marker.pose.position.y = coefficients->values[1];
          marker.pose.position.z = coefficients->values[2];

          Eigen::Quaternion<double> qu(0,0,0,1);
          Eigen::Quaternion<double> qn(0,coefficients->values[4],coefficients->values[5],coefficients->values[6]);
          auto q = qu*qn;
          q.w() = 1-q.w();
          q.normalize(); 

          marker.scale.x = 2*coefficients->values[3];
          marker.scale.y = 2*coefficients->values[3];
          marker.scale.z = 0.001;

          marker.color.r = 0.0;
          marker.color.g = 1.0;

          marker.pose.orientation.w = q.w();
          marker.pose.orientation.x = q.x();
          marker.pose.orientation.y = q.y();
          marker.pose.orientation.z = q.z();
        }


        marker_publisher_->publish(marker);
        id ++;
      }
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud,cloud_msg);

      cloud_msg.header.stamp = this->get_clock()->now();
      cloud_msg.header.frame_id = "astra";

      cloud_publisher_->publish(cloud_msg);

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

    

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurfaceNode>());
  rclcpp::shutdown();
  return 0;
}

