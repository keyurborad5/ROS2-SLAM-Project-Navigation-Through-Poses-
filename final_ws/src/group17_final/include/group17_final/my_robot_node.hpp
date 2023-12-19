#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// for dynamic broadcaster
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
using namespace std::chrono_literals;

class MyRobotNode : public rclcpp::Node{

    public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
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
        part_tf_broadcaster_1_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // Load a buffer of transforms
        part_tf_buffer_1_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        part_tf_buffer_1_->setUsingDedicatedThread(true);

        


        //************************Listener******************************
        // load a buffer of transforms
        part_tf_listener_buffer_1_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
        part_tf_listener_buffer_1_->setUsingDedicatedThread(true);
        part_transform_listener_1_ =std::make_shared<tf2_ros::TransformListener>(*part_tf_listener_buffer_1_);

       


        //********************Publisher**************************

        // initialize the publisher
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        //********************Subscriber**************************
        //*******aruco cam subscriber**********
        aruco_cam_subscriber_=this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::aruco_cam_sub_cb,this,std::placeholders::_1));
        //*******camera detecting part subscriber*****
        part_cam_subscriber1_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera1/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb1,this,std::placeholders::_1));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        part_cam_subscriber2_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera2/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb2,this,std::placeholders::_1));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        part_cam_subscriber3_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera3/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb3,this,std::placeholders::_1));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        part_cam_subscriber4_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera4/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb4,this,std::placeholders::_1));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        part_cam_subscriber5_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera5/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb5,this,std::placeholders::_1));
        std::this_thread::sleep_for(std::chrono::seconds(5));
        odom_subscriber_=this->create_subscription<nav_msgs::msg::Odometry>("odom",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::odom_sub_cb,this,std::placeholders::_1));

        //************************Action******************************
        // initialize the client
        client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
        

     }

     private:

    //###########################-----------ACTION SERVER------------########################

    //******************Action Attribubtes*******************
    /**
     * @brief Action client for the action server navigate_to_pose
     *
     */
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_;
    //   rclcpp::TimerBase::SharedPtr timer_;

    //******************Action Methods*******************
    /**
     * @brief Response from the server after sending the goal
     */
    void goal_response_callback(
        std::shared_future<GoalHandleNavigation::SharedPtr> future);
    /**
     * @brief Feedback received while the robot is driving towards the goal
     *
     * @param feedback
     */
    void feedback_callback(
        GoalHandleNavigation::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
    /**
     * @brief Result after the action has completed
     *
     * @param result
     */
    void result_callback(const GoalHandleNavigation::WrappedResult& result);


    //###########################-----------BROADCASTER------------########################

    //******************Broadcaster Attribubtes*******************

    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> part_tf_buffer_1_;
    /*!< MyRobotNode object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_1_;
    

    //**********************Broadcaster Methods***********************
    
    /**
     * @brief To broadcast the frame and listen frame in odom 
     * 
     * @param msg input header and poses of the detected object in the given format
     * @param cam_frame input the head frame ID of the detected part frame 
     * @param source_frame name the frame to which you want to tranform the frame ID
     * @param target_frame name the frame ID of the detected part
     */
    void part_transformer(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg, const std::string &cam_frame,  const std::string &source_frame, const std::string &target_frame);
    

    //###########################-----------LISTENER------------########################

     //******************Listener Attribubtes*******************
    std::unique_ptr<tf2_ros::Buffer> part_tf_listener_buffer_1_;
    std::shared_ptr<tf2_ros::TransformListener> part_transform_listener_1_{nullptr};


    //###########################-----------PUBLISHER------------########################

    //******************Publisher Attribubtes*******************  

    /**
     * @brief Publisher to the topic /initialpose
    *
    */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    //**********************Publisher Methods***********************
     // set the initial pose for navigation
    /**
     * @brief Set the initial pose object
     * 
     * @param msg msg is pose of robot from odom topic
     * @return * void 
     */
    void set_initial_pose(nav_msgs::msg::Odometry::SharedPtr msg);


    //###########################-----------SUBSCRIBER------------########################

    //******************Subscriber Attribubtes*******************

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_cam_subscriber_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber1_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber2_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber3_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber4_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber5_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

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

     /**
     * @brief A Timer Callback for reading the Odometry data from Topic odom
     * 
     * @param msg A type of message sent by the Topic /odom
     */
    void odom_sub_cb(nav_msgs::msg::Odometry::SharedPtr msg);


    //###########################-----------ADDITIONAL METHODS------------########################
   /**
    * @brief To send goal to bot to navigate through poses
    * 
    */
    void send_goal();
    /**
     * @brief Create a Pose object
     * 
     * @param x x position of pose
     * @param y y position of pose
     * @param z z position of pose
     * @return geometry_msgs::msg::PoseStamped returns vector of pose in PoseStamped type
     */
    geometry_msgs::msg::PoseStamped createPose(double x, double y, double z);
    /**
     * @brief Get the waypoints coordinates object which is sorted according to waypoint color based on aruco marker ID
     * 
     */
    void get_waypoints_coordinates();
    //###########################-----------ADDITIONAL ATTRIBUTES------------########################
    int marker_id_;
    bool found_marker_id_=false;
    double part_color_;
    std::vector<std::string> aruco_0_waypoints_;
    std::vector<std::string> aruco_1_waypoints_;
    std::vector<std::string> follow_waypoints_; //for waypoints in color
    std::vector<int> follow_waypoints_n_;// for waypoints in the color mapped integer 
    std::vector<std::vector<double>> waypoints_coordinates_;


    std::string camera_frame_;
    std::vector<std::vector<double>> parts_vector_;
    GoalHandleNavigation::SharedPtr goal_handle_;


};