#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"


#include <astra/astra.hpp>




using namespace std::chrono_literals;

class AstraNode : public rclcpp::Node, public astra::FrameListener
{
  public:
    AstraNode(): Node("astra_node")
    {

      this->declare_parameter("field_of_view", std::vector<double>{-800, 800, -800, 800, 700, 1200});
      foV_ = this->get_parameter("field_of_view").as_double_array();

      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 1);
      publisher_depth_img_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 1);
      publisher_ir_img_ = this->create_publisher<sensor_msgs::msg::Image>("ir_image", 1);
      publisher_color_img_ = this->create_publisher<sensor_msgs::msg::Image>("color_image", 1);


      astra::initialize();

      astra::StreamSet streamSet;
      reader_= streamSet.create_reader();


      auto depthStream = reader_.stream<astra::DepthStream>();
      auto pointStream = reader_.stream<astra::PointStream>();
      auto irStream = reader_.stream<astra::InfraredStream>();
      auto colorStream = reader_.stream<astra::ColorStream>();
      depthStream.start();
      pointStream.start();
      irStream.start();
      colorStream.start();

      reader_.add_listener(*this);
      while(rclcpp::ok()){
        // RCLCPP_INFO(this->get_logger(),"updated");
        astra_update();
      }

    }
    ~AstraNode(){
      reader_.remove_listener(*this);
      astra::terminate();
    }

    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override
    {
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
        const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();
        const astra::InfraredFrame16 irFrame = frame.get<astra::InfraredFrame16>();
        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

        if (colorFrame.is_valid())
        {
            std::vector<uint8_t> data;
            data.reserve(colorFrame.length() * 3);
            for(auto i=0ul; i< colorFrame.length();i++){
              data.push_back(colorFrame.data()[i].r);
              data.push_back(colorFrame.data()[i].g);
              data.push_back(colorFrame.data()[i].b);
            }
            
            auto img = sensor_msgs::msg::Image();
            img.header.stamp = this->get_clock()->now();
            img.header.frame_id = "astra";
            img.height = colorFrame.height();
            img.width = colorFrame.width();
            img.is_bigendian = false;
            img.step = colorFrame.height() * colorFrame.width();
            img.data = data;
            img.encoding = sensor_msgs::image_encodings::RGB8;
            publisher_color_img_->publish(img);
        }
        if (depthFrame.is_valid())
        {
            std::vector<uint8_t> data(depthFrame.data(),depthFrame.data()+depthFrame.length());
            
            auto img = sensor_msgs::msg::Image();
            img.header.stamp = this->get_clock()->now();
            img.header.frame_id = "astra";
            img.height = depthFrame.height();
            img.width = depthFrame.width();
            img.is_bigendian = false;
            img.step = depthFrame.height() * depthFrame.width();
            img.data = data;
            img.encoding = sensor_msgs::image_encodings::MONO8;
            publisher_depth_img_->publish(img);
        }
        if (irFrame.is_valid())
        {
            std::vector<uint8_t> data(irFrame.data(),irFrame.data()+irFrame.length());
            
            auto ir_img = sensor_msgs::msg::Image();
            ir_img.header.stamp = this->get_clock()->now();
            ir_img.header.frame_id = "astra";
            ir_img.height = irFrame.height();
            ir_img.width = irFrame.width();
            ir_img.is_bigendian = false;
            ir_img.step = irFrame.height() * irFrame.width();
            ir_img.data = data;
            ir_img.encoding = sensor_msgs::image_encodings::MONO8;
            publisher_ir_img_->publish(ir_img);
        }
        if (pointFrame.is_valid())
        {
          

          std::vector<astra::Vector3f> data;
          data.reserve(pointFrame.length());
          uint nbr_valid_data = 0;
          for(auto i=0ul; i< pointFrame.length();i++){
            if( pointFrame.data()[i].x>=foV_[0] && pointFrame.data()[i].x<=foV_[1] &&
                pointFrame.data()[i].y>=foV_[2] && pointFrame.data()[i].y<=foV_[3] &&
                pointFrame.data()[i].z>=foV_[4] && pointFrame.data()[i].z<=foV_[5] ){

                  data.push_back(pointFrame.data()[i]);
                  nbr_valid_data++;
            }
          }

          data.resize(nbr_valid_data);
          
          sensor_msgs::msg::PointCloud2 cloud_msg;
          cloud_msg.header.stamp = this->get_clock()->now();
          cloud_msg.header.frame_id = "astra";
          cloud_msg.height = 1;
          cloud_msg.width = data.size();
          sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
          modifier.setPointCloud2Fields(3,  "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
          modifier.resize(data.size());
          sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
          sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
          sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

          for(size_t i=0; i<data.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
              *iter_x = data[i].x*0.001;
              *iter_y = data[i].y*0.001;
              *iter_z = data[i].z*0.001;
          }
  
          publisher_->publish(cloud_msg);
        }
    }


  private:
    

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_ir_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_color_img_;
    std::vector<double> foV_;

    astra::StreamReader reader_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AstraNode>());
  rclcpp::shutdown();
  return 0;
}