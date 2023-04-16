/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <iostream>
#include <string>
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros/mavros.h>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
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

    
    
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            
            std_msgs::String result;
            
            result.data = ser.read(ser.available());
            
            // from std_msgs::String to geometry_msgs::PoseStamped
            geometry_msgs::PoseStamped geo_pose_uwb_position;
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
            //std::cout << "X: " << i_x_value << " Y:  " << i_y_value << " Z: " << i_z_value << std::endl;

            // Send to Mavros
            geo_pose_uwb_position.pose.position.x = i_x_value;
            geo_pose_uwb_position.pose.position.y = i_y_value;
            geo_pose_uwb_position.pose.position.z = i_z_value;
            read_pub.publish(result);
            ros_pub_uwb_position.publish(geo_pose_uwb_position);
            // Publish PoseStamped data to mavros.
        }
        loop_rate.sleep();

    }
}

