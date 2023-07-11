#include <iostream>
#include <string>
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros/mavros.h>

serial::Serial ser;


int main (int argc, char** argv){
    ros::init(argc, argv, "uwb_loc");
    ros::NodeHandle nh;

    ros::Publisher ros_pub_uwb_position = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1);
    ros::Rate loop_rate(40);
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    geometry_msgs::PoseStamped geo_pose_uwb_position_prev;
    
    while(ros::ok()){

        ros::spinOnce();

        geometry_msgs::PoseStamped geo_pose_uwb_position;
        geo_pose_uwb_position.header.stamp = ros::Time::now();
        // Need to fix this orientation .. Almost Done !
        geo_pose_uwb_position.header.frame_id = "/MAV_FRAME_LOCAL_ENU"; 

        if(ser.available()){
            
            std_msgs::String result;
            
            result.data = ser.read(ser.available());
            
            // from std_msgs::String to geometry_msgs::PoseStamped
            //std::cout << result.data << std::endl;

            int i_init_loc = 0;
            // Find X value
            std::string str_first_seperator  = ":";
            std::string str_second_seperator = ",";

            int i_first_idx  = result.data.find(str_first_seperator, i_init_loc) + 1;
            int i_second_idx = result.data.find(str_second_seperator, i_init_loc) ;
            int i_length     = i_second_idx - i_first_idx;
            float i_x_value    = stof(result.data.substr( i_first_idx , i_length ))/1000;
            i_init_loc = i_second_idx+1;  // add 1 for find next seperation point.
            // Find Y value
            i_first_idx  = result.data.find(str_first_seperator, i_init_loc) + 1;
            i_second_idx = result.data.find(str_second_seperator, i_init_loc) ;
            i_length     = i_second_idx - i_first_idx;
            float i_y_value    = stof(result.data.substr( i_first_idx , i_length ))/1000;
            i_init_loc = i_second_idx+1;  // add 1 for find next seperation point.
            // Find Z value
            str_second_seperator = "\n";
            i_first_idx  = result.data.find(str_first_seperator, i_init_loc) + 1;
            i_second_idx = result.data.find(str_second_seperator, i_init_loc) ;
            i_length     = i_second_idx - i_first_idx;
            float i_z_value    = stof(result.data.substr( i_first_idx , i_length ))/1000;
            std::cout << "X: " << i_x_value << " Y:  " << i_y_value << " Z: " << i_z_value << std::endl;

            // Send to Mavros
            geo_pose_uwb_position.pose.position.x = i_x_value;
            geo_pose_uwb_position.pose.position.y = i_y_value;
            geo_pose_uwb_position.pose.position.z = i_z_value;

            geo_pose_uwb_position_prev.pose.position.x = i_x_value;
            geo_pose_uwb_position_prev.pose.position.y = i_y_value;
            geo_pose_uwb_position_prev.pose.position.z = i_z_value;
        }
        else
        {
            geo_pose_uwb_position.pose.position.x = geo_pose_uwb_position_prev.pose.position.x;
            geo_pose_uwb_position.pose.position.y = geo_pose_uwb_position_prev.pose.position.y;
            geo_pose_uwb_position.pose.position.z = geo_pose_uwb_position_prev.pose.position.z;
            std::cout << "X: " <<  geo_pose_uwb_position.pose.position.x  << " Y:  " <<  geo_pose_uwb_position.pose.position.y  << " Z: " <<  geo_pose_uwb_position.pose.position.z << " (Prev)" << std::endl;
        }

        // Publish PoseStamped data to mavros.
        ros_pub_uwb_position.publish(geo_pose_uwb_position);
        loop_rate.sleep();

    }
    
}

