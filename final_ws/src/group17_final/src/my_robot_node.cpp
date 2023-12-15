#include <my_robot_node.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// needed for the listener
#include <tf2/exceptions.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>


//Aruco camera callback method
void MyRobotNode::aruco_cam_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){

   if(!msg->marker_ids.empty()){//to check if the topic is pusblishing null
    marker_id_=msg->marker_ids[0];
    RCLCPP_INFO_STREAM(this->get_logger(),"FOUND Aruco marker ID: "<<msg->marker_ids[0]);
    aruco_cam_subscriber_.reset();
   }
    else{
   
     RCLCPP_INFO_STREAM(this->get_logger(),"NO Aruco marker FOUND ");
    }

}



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyRobotNode>("my_robot_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
}