#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

//TODO:
// ADD timestamp to msgs 

class Multicam_Node : public rclcpp::Node
{
  public:
    Multicam_Node()
    : Node("multicam_node")
    {
      cap_.open(2, cv::CAP_V4L);

      if(!cap_.isOpened()){

        RCLCPP_INFO(this->get_logger(), "Unable to open camera");
        std::exit(EXIT_FAILURE);
      } else{

        RCLCPP_INFO(this->get_logger(), "Camera opened succesfully");
      }
      
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("img_publisher", 10);
      timer_ = this->create_wall_timer(
      33ms, std::bind(&Multicam_Node::image_callback, this));
    }

    ~Multicam_Node(){

      RCLCPP_INFO(this->get_logger(), "Closing camera...");
      cap_.release();      
    }

  private:
    void image_callback()
    {
      cv::Mat color_frame, gray_frame; 
      cap_ >> color_frame;

      if(!color_frame.empty()){

        cv::cvtColor(color_frame, gray_frame, cv::COLOR_BGR2GRAY);

        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"mono8",gray_frame).toImageMsg();


        /*
        cv::imshow("Image", gray_frame);

        if( cv::waitKey(5) == 27){

          std::exit(EXIT_FAILURE);
        }
        */

        img_msg->header.stamp = this->get_clock()->now();

        publisher_->publish(*img_msg.get());
      }
    }


    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Multicam_Node>());
  rclcpp::shutdown();
  return 0;
}
