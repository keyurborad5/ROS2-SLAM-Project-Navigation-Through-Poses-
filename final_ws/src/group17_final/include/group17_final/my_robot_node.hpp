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

        std::vector<std::string>wp{"wp1","wp2","wp3","wp4","wp5"};
        for (auto &n:wp){
            this->declare_parameter("aruco_0."+ n+".color","white");
            this->declare_parameter("aruco_1."+ n+".color","black");
            std::string aruco_0_color = this->get_parameter("aruco_0."+n+".color").as_string();
            aruco_0_waypoints_.push_back(aruco_0_color);
            std::string aruco_1_color = this->get_parameter("aruco_1."+n+".color").as_string();
            aruco_1_waypoints_.push_back(aruco_1_color);
        }

        for(auto &i:aruco_0_waypoints_){
            RCLCPP_INFO_STREAM(this->get_logger(),i);
        }

        //************************Broadcaster******************************
        // initialize the transform broadcaster
        part_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // Load a buffer of transforms
        part_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        part_tf_buffer_->setUsingDedicatedThread(true);
         


        //************************Listener******************************
        // load a buffer of transforms
        part_tf_listener_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
        part_tf_listener_buffer_->setUsingDedicatedThread(true);
        part_transform_listener_ =std::make_shared<tf2_ros::TransformListener>(*part_tf_listener_buffer_);


        //********************Subscriber**************************
        
        // aruco_cam_subscriber_=this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::aruco_cam_sub_cb,this,std::placeholders::_1));
        
        rclcpp::QoS qos(10); qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        

        part_cam_subscriber1_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera1/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb1,this,std::placeholders::_1));
        part_cam_subscriber2_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera2/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb2,this,std::placeholders::_1));
        part_cam_subscriber3_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera3/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb3,this,std::placeholders::_1));
        part_cam_subscriber4_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera4/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb4,this,std::placeholders::_1));
        part_cam_subscriber5_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera5/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb5,this,std::placeholders::_1));


     }

     private:

    //###########################-----------BROADCASTER------------########################

    //******************Broadcaster Attribubtes*******************

    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> part_tf_buffer_;
    /*!< MyRobotNode object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_;

    //**********************Broadcaster Methods***********************
    /**
     * @brief Timer to broadcast the transform
     *
     */
    void part_broadcaster(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr);

    //###########################-----------LISTENER------------########################

     //******************Listener Attribubtes*******************
    std::unique_ptr<tf2_ros::Buffer> part_tf_listener_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> part_transform_listener_{nullptr};
    
    //**********************Listener Methods***********************
    /**
     * @brief Listen to a part transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void part_listen_transform(const std::string &source_frame, const std::string &target_frame);

    

    
    //###########################-----------SUBSCRIBER------------########################

    //******************Subscriber Attribubtes*******************

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_cam_subscriber_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber1_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber2_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber3_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber4_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber5_;


    //**********************Subscriber Methods***********************
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
    void part_cam_sub_cb1(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void part_cam_sub_cb2(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void part_cam_sub_cb3(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void part_cam_sub_cb4(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void part_cam_sub_cb5(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);


    //###########################-----------ADDITIONAL METHODS------------########################

    //###########################-----------ADDITIONAL ATTRIBUTES------------########################
    int marker_id_;
    bool found_marker_id_=false;
    int part_color_;
    std::vector<std::string> aruco_0_waypoints_;
    std::vector<std::string> aruco_1_waypoints_;
    std::vector<std::string> advanced_camera_topics_{"mage/camera1/image","mage/camera2/image","mage/camera3/image","mage/camera4/image","mage/camera5/image"};
    std::vector<std::string> advanced_camera_frames_{"camera1_frame","camera2_frame","camera3_frame","camera4_frame","camera5_frame"};
    std::string camera_frame_;
    std::vector<double> detection_;


};