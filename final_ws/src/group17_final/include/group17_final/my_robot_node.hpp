#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include<nav_msgs/msg/odometry.hpp>
// for dynamic broadcaster
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
using namespace std::chrono_literals;

class MyRobotNode : public rclcpp::Node{

    public:
     MyRobotNode(std::string node_name) : Node(node_name){

         //********************Subscriber**************************
        
        aruco_cam_subscriber_=this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::aruco_cam_sub_cb,this,std::placeholders::_1));
        
        rclcpp::QoS qos(10); qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        //part_cam_subscriber_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/advanced_logical_camera/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb,this,std::placeholders::_1));
        

     }

     private:
     //###########################-----------SUBSCRIBER------------########################

     //******************Attribubtes*******************

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_cam_subscriber_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber_;

    //**********************Methods***********************
    /**
     * @brief Timer callback for Aruco camera to continously read the feed
     * 
     * @param msg A type of a message sent by the TOPIC /aruco_markers
     */
    void aruco_cam_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
    /**
     * @brief Timer callback for Advanced logical camera detecting a part
     * 
     * @param msg A type of message sent by the TOPIC mage/Advanced_logical_camera/Images
     */
    void part_cam_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    //###########################-----------ADDITIONAL METHODS------------########################

    //###########################-----------ADDITIONAL ATTRIBUTES------------########################
    int marker_id_;
    bool found_marker_id_=false;


};