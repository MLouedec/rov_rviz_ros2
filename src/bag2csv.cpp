//
// Created by morgan on 08/02/24.
// extract specific data from the bag file and save it in a csv file to be used in Matlab or python
//
#include <chrono>
#include <string>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

using namespace std::chrono_literals;

class BAG2CSV: public rclcpp::Node{
public:
    BAG2CSV(): Node("bag2csv"){
        // get the parameter of the name of the csv file
        this->declare_parameter("csv_full_path","no_path");
        std::string filepath = this->get_parameter("csv_full_path").as_string();

        // print the filepath
        RCLCPP_INFO(this->get_logger(), "csv file path: %s", filepath.c_str());

        // delete the file if it exists
        std::remove(filepath.c_str());

        // open the csv file, reset it and set the header line
        file.open(filepath,std::ofstream::app);
        file << "time,u_rx,u_ry,u_rz,pz,roll,pitch,yaw" << std::endl;

        // subscribe to the topics
        inputs_subscriber = this->create_subscription<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10, std::bind(&BAG2CSV::inputs_callback, this, std::placeholders::_1));
        mavros_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/mavros/global_position/local", 10, std::bind(&BAG2CSV::mavros_callback, this, std::placeholders::_1));

        // The timer callback to print text in the console
        timer_ = this->create_wall_timer(100ms, std::bind(&BAG2CSV::timer_callback, this));

        inputs = Eigen::VectorXd::Zero(16);
        position = Eigen::VectorXd::Zero(3);
        euler_angles = Eigen::VectorXd::Zero(3);
        simulation_time = 0;
        init_time = false;
    }
    // close the file when the node is destroyed
    ~BAG2CSV(){
        file.close();
    }
private:
    Eigen::VectorXd inputs; // inputs of the ROV
    Eigen::VectorXd position; // mavros position
    Eigen::VectorXd euler_angles; // mavros orientation
    std::ofstream file; // csv file to save the data
    double simulation_time; // relative time since the initial time
    double t0; // initial time of the rosbag

    bool init_time;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr inputs_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavros_subscriber;

    void inputs_callback(const mavros_msgs::msg::OverrideRCIn::SharedPtr msg){
        for(int i=0;i<16;i++){
            inputs(i) = msg->channels[i];
        }
    }

    void mavros_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        double rosbag_time = pow(10,-9)*msg->header.stamp.nanosec+msg->header.stamp.sec;
        if(!init_time){
            t0 = rosbag_time;
            init_time = true;
        }
        simulation_time = rosbag_time - t0;
        position(0) = msg->pose.pose.position.x;
        position(1) = msg->pose.pose.position.y;
        position(2) = msg->pose.pose.position.z;

        tf2::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(euler_angles(0),euler_angles(1),euler_angles(2));

//        simulation_time = pow(10,-9)*msg->header.stamp.nanosec+msg->header.stamp.sec;
//        simulation_time = msg->header.stamp.nanosec;
//        simulation_time = msg->header.stamp.sec;
    }

    void timer_callback(){
        RCLCPP_INFO(this->get_logger(), "simulation time: %f", simulation_time);

        // save the data in a csv file
        file << simulation_time << "," << inputs(4) << "," << inputs(5) << "," << inputs(2) << "," <<
        position(2) << "," << euler_angles(0) << "," << euler_angles(1) << "," << euler_angles(2) << std::endl;
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<BAG2CSV> node = std::make_shared<BAG2CSV>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}