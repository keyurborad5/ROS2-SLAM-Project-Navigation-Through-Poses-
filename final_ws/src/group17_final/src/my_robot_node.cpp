#include <my_robot_node.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// needed for the listener
#include <tf2/exceptions.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>


//A broadcaster to assign a frame for the detected object
void MyRobotNode::part_broadcaster(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped part_dynamic_transform_stamped;

    // RCLCPP_INFO(this->get_logger(), "Broadcasting dynamic_frame");
    part_dynamic_transform_stamped.header.stamp = this->get_clock()->now();

    part_dynamic_transform_stamped.header.frame_id = camera_frame_.c_str();
    part_dynamic_transform_stamped.child_frame_id = "part";

    part_color_= msg->part_poses[0].part.color;
    //part_type_= msg->part_poses[0].part.type;

    switch(part_color_){
        case mage_msgs::msg::Part::BLUE:
        RCLCPP_INFO_STREAM(this->get_logger(), "DETECTED PART BLUE COLOR: ");
        break;
        case mage_msgs::msg::Part::RED:
        RCLCPP_INFO_STREAM(this->get_logger(), "DETECTED PART RED COLOR: ");
        break;
        case mage_msgs::msg::Part::GREEN:
        RCLCPP_INFO_STREAM(this->get_logger(), "DETECTED PART GREEN COLOR: ");
        break;
        case mage_msgs::msg::Part::PURPLE:
        RCLCPP_INFO_STREAM(this->get_logger(), "DETECTED PART PURPLE COLOR: ");
        break;
        case mage_msgs::msg::Part::ORANGE:
        RCLCPP_INFO_STREAM(this->get_logger(), "DETECTED PART ORANGE COLOR: ");
        break;
        RCLCPP_INFO_STREAM(this->get_logger(), "DETECTED NO COLOR: ");
        default:
        break;
    }

    part_dynamic_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    part_dynamic_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    part_dynamic_transform_stamped.transform.translation.z = -msg->part_poses[0].pose.position.z;//had to make it neg because it was detecting object below the floor

    part_dynamic_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    part_dynamic_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    part_dynamic_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    part_dynamic_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;
   
    // Send the transform
    part_tf_broadcaster_->sendTransform(part_dynamic_transform_stamped);
    
    // RCLCPP_INFO_STREAM(this->get_logger(), "PART advance cam Broadcasting_dynamic_frame : "<<msg->part_poses[0].pose.position.x;);
}

// A listerner that would tansform a detected part in odom frame
void MyRobotNode::part_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = part_tf_listener_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;
    RCLCPP_INFO_STREAM(this->get_logger(), "DETECTED PART COLOR: "<<part_color_);

    // //converting detected geometery quaternion to tf2 quaternion
    // tf2::Quaternion q( 
    //     pose_out.orientation.x,
    //     pose_out.orientation.y,
    //     pose_out.orientation.z,
    //     pose_out.orientation.w);
    // //Utlising Utlis to convert quaternion to rpy
    // std::array<double, 3> euler = utils_ptr_->set_euler_from_quaternion(q);
    //initialising local variable as vector of data of detected object
     std::vector<std::variant<int,double> >detection={part_color_,pose_out.position.x,
                                                pose_out.position.y,
                                                pose_out.position.z,
                                                pose_out.orientation.x,
                                                pose_out.orientation.y,
                                                pose_out.orientation.z,
                                                pose_out.orientation.w};
    //passing the above vector to logg all the detected parts
    parts_vector_.push_back(detection);
    
}

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

//Part's Advanced logical camera 1 callback method
void MyRobotNode::part_cam_sub_cb1(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    if(!msg->part_poses.empty()){//Very necesaary otherwise node will die by it self; checking if topic is publishing or not
    
    //RCLCPP_INFO_STREAM(this->get_logger(),"Part Advance Camera callback : ");

    //once detected part, then we can broadcast a new frame at the detected location 
    camera_frame_="camera1_frame";
    part_broadcaster(msg);
    
    //after broadcasting we can listen the part frame wrt to any frame ,here we want wrt odom frame
    part_listen_transform("map", "part");
         
    RCLCPP_INFO_STREAM(this->get_logger(),"Continue Part subscription camera 1: ");

    part_cam_subscriber1_.reset(); 
        
        }
    else{    //RCLCPP_INFO_STREAM(this->get_logger(),"NO Part Detected : ");
    }

}
//Part's Advanced logical camera 2 callback method
void MyRobotNode::part_cam_sub_cb2(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    if(!msg->part_poses.empty()){//Very necesaary otherwise node will die by it self; checking if topic is publishing or not
    
    //RCLCPP_INFO_STREAM(this->get_logger(),"Part Advance Camera callback : ");

    //once detected part, then we can broadcast a new frame at the detected location 
    camera_frame_="camera2_frame";
    part_broadcaster(msg);
    //after broadcasting we can listen the part frame wrt to any frame ,here we want wrt odom frame
    part_listen_transform("map", "part");

         
    RCLCPP_INFO_STREAM(this->get_logger(),"Continue Part subscription camera 2: ");

    part_cam_subscriber2_.reset(); 
        
        }
    else{    //RCLCPP_INFO_STREAM(this->get_logger(),"NO Part Detected : ");
    }

}

//Part's Advanced logical camera 3 callback method
void MyRobotNode::part_cam_sub_cb3(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    if(!msg->part_poses.empty()){//Very necesaary otherwise node will die by it self; checking if topic is publishing or not
    
    //RCLCPP_INFO_STREAM(this->get_logger(),"Part Advance Camera callback : ");

    //once detected part, then we can broadcast a new frame at the detected location 
    camera_frame_="camera3_frame";
    part_broadcaster(msg);
    //after broadcasting we can listen the part frame wrt to any frame ,here we want wrt odom frame
    part_listen_transform("map", "part");

         
    RCLCPP_INFO_STREAM(this->get_logger(),"Continue Part subscription camera 3: ");

    part_cam_subscriber3_.reset(); 
        
        }
    else{    //RCLCPP_INFO_STREAM(this->get_logger(),"NO Part Detected : ");
    }

}
//Part's Advanced logical camera 4 callback method
void MyRobotNode::part_cam_sub_cb4(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    if(!msg->part_poses.empty()){//Very necesaary otherwise node will die by it self; checking if topic is publishing or not
    
    //RCLCPP_INFO_STREAM(this->get_logger(),"Part Advance Camera callback : ");

    //once detected part, then we can broadcast a new frame at the detected location 
    camera_frame_="camera4_frame";
    part_broadcaster(msg);
    //after broadcasting we can listen the part frame wrt to any frame ,here we want wrt odom frame
    part_listen_transform("map", "part");

         
    RCLCPP_INFO_STREAM(this->get_logger(),"Continue Part subscription camera 4: ");

    part_cam_subscriber4_.reset(); 
        
        }
    else{    //RCLCPP_INFO_STREAM(this->get_logger(),"NO Part Detected : ");
    }

}
//Part's Advanced logical camera 5 callback method
void MyRobotNode::part_cam_sub_cb5(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    if(!msg->part_poses.empty()){//Very necesaary otherwise node will die by it self; checking if topic is publishing or not
    
    //RCLCPP_INFO_STREAM(this->get_logger(),"Part Advance Camera callback : ");

    //once detected part, then we can broadcast a new frame at the detected location 
    camera_frame_="camera5_frame";
    part_broadcaster(msg);
    //after broadcasting we can listen the part frame wrt to any frame ,here we want wrt odom frame
    part_listen_transform("map", "part");

         
    RCLCPP_INFO_STREAM(this->get_logger(),"Continue Part subscription camera 5: ");

    part_cam_subscriber5_.reset(); 
        
        }
    else{    //RCLCPP_INFO_STREAM(this->get_logger(),"NO Part Detected : ");
    }

}





int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyRobotNode>("my_robot_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
}