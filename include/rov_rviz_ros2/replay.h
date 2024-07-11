// Purpose: Header file for the rov_rviz_node.cpp file
// This node can be used to visualize the ROVs in rviz from the rosbag file

#include <chrono>
#include <string>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message2/msg/usbl.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
//#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Scalar.h"


//#include <geometry_msgs/Pose.h>
//#include <visualization_msgs/Marker.h>
//#include <geometry_msgs/Quaternion.h>
//#include <geometry_msgs/WrenchStamped.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Odometry.h>
//#include <mavros_msgs/OverrideRCIn.h>
//#include <message2/Usbl.h>

using namespace std::chrono_literals;

float sawtooth(float x){
    return 2*atan(tan(x/2));
}

void pose_yaw_offset(geometry_msgs::msg::Pose& pose, float yaw_offset);

void set_color(visualization_msgs::msg::Marker::SharedPtr marker,  const std::string& rov_frame);



class RovRviz : public rclcpp::Node{
public:
    RovRviz(): Node("rov_rviz_ros2"){

        // get the name of the ROVs
        this->declare_parameter("rovA_name","no_name");
        rovA_name = this->get_parameter("rovA_name").as_string();
        RCLCPP_INFO(this->get_logger(),"We are in the presence of: %s", rovA_name.c_str());

        this->declare_parameter("rovB_name","no_name");
        rovB_name = this->get_parameter("rovB_name").as_string();
        RCLCPP_INFO(this->get_logger(),"We are in the presence of: %s", rovB_name.c_str());

        // ------------------------------
        // subscribers and publishers
        // ------------------------------

        // usbl measurement
        sub_usbl = this->create_subscription<message2::msg::Usbl>(
                "/bouee/usbl",
                10, std::bind(&RovRviz::ubslCallback, this, std::placeholders::_1));

        sub_bouee_imu = this->create_subscription<geometry_msgs::msg::Quaternion>(
                "/bouee/imu_data",
                10, std::bind(&RovRviz::bouee_imu_Callback, this, std::placeholders::_1));

        // mavros to get z and attitude
        sub_mavros_rovA = this->create_subscription<nav_msgs::msg::Odometry>(
                "/"+rovA_name+"/mavros/global_position/local",
                10, std::bind(&RovRviz::mavros_rovA_Callback, this, std::placeholders::_1));

        sub_mavros_rovB = this->create_subscription<nav_msgs::msg::Odometry>(
                "/"+rovB_name+"/mavros/global_position/local",
                10, std::bind(&RovRviz::mavros_rovB_Callback, this, std::placeholders::_1));

        // get the position of the targets
        sub_target_rovA =  this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/"+rovA_name+"/coord_objectif",
                10, std::bind(&RovRviz::target_rovA_Callback, this, std::placeholders::_1));

        sub_target_rovB =  this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/"+rovB_name+"/coord_objectif",
                10, std::bind(&RovRviz::target_rovB_Callback, this, std::placeholders::_1));

        // get the motors commands
        sub_RC_rovA = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
                "/"+rovA_name+"/mavros/rc/override",
                10, std::bind(&RovRviz::RC_rovA_Callback, this, std::placeholders::_1));

        sub_RC_rovB = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
                "/"+rovB_name+"/mavros/rc/override",
                10, std::bind(&RovRviz::RC_rovB_Callback, this, std::placeholders::_1));

        // give the position of the last usbl measurement
        pub_marker_rovA_measurement = this->create_publisher<visualization_msgs::msg::Marker>("/"+rovA_name+"/marker",10);
        pub_marker_rovB_measurement = this->create_publisher<visualization_msgs::msg::Marker>("/"+rovB_name+"/marker",10);

        // give the position of the target of the ROVs
        pub_marker_rovA_target = this->create_publisher<visualization_msgs::msg::Marker>("/"+rovA_name+"/target_marker", 10);
        pub_marker_rovB_target = this->create_publisher<visualization_msgs::msg::Marker>("/"+rovB_name+"/target_marker", 10);

        // draw the triangle (with the targets of the ROVs)
        pub_triangle = this->create_publisher<visualization_msgs::msg::Marker>("/triangle_maker", 10);

        // give the control input of the ROVs
        pub_control_input_rovA = this->create_publisher<geometry_msgs::msg::TwistStamped>("/"+rovA_name+"/control_marker", 10);
        pub_control_input_rovB = this->create_publisher<geometry_msgs::msg::TwistStamped>("/"+rovB_name+"/control_marker", 10);

        // The timer callback to print text in the console
        timer_ = this->create_wall_timer(10ms, std::bind(&RovRviz::timer_callback, this));

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // initialise the variables
//        mission_time = 0;
//        usbl_rotation_matrix.setIdentity();
        control_input_rovA = geometry_msgs::msg::TwistStamped();
        control_input_rovB = geometry_msgs::msg::TwistStamped();
        pose_measured_rovA = std::make_shared<geometry_msgs::msg::PoseStamped>();
        pose_measured_rovB = std::make_shared<geometry_msgs::msg::PoseStamped>();
        pose_target_rovA = std::make_shared<geometry_msgs::msg::PoseStamped>();
        pose_target_rovB = std::make_shared<geometry_msgs::msg::PoseStamped>();
        pose_bouee = std::make_shared<geometry_msgs::msg::PoseStamped>();

        RCLCPP_INFO(this->get_logger(),"rov_rviz_node initialized");
    }
private:

//    Eigen::Matrix3d usbl_rotation_matrix; // heading of the usbl

//    double mission_time; // ROS time during the mission (rosbag time is not the same), given by mavros

    std::string rovA_name; // name of the ROVs used to name topics
    std::string rovB_name;

    geometry_msgs::msg::TwistStamped control_input_rovA; // wrench of the ROVs
    geometry_msgs::msg::TwistStamped control_input_rovB;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_measured_rovA; // measured pose of the ROV ROVs (only the x and y are measured)
    geometry_msgs::msg::PoseStamped::SharedPtr pose_measured_rovB; // the rest ( z yaw pitch roll) come from the measurement

    geometry_msgs::msg::PoseStamped::SharedPtr pose_target_rovA; // pose of ROVs' target
    geometry_msgs::msg::PoseStamped::SharedPtr pose_target_rovB;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_bouee; // pose of the buoy holding the USBL

    rclcpp::TimerBase::SharedPtr timer_;

    double h_imu_usbl = 0.42; // m
    double biais_usbl_IMU = 0.0; // rad

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  pub_marker_rovA_measurement; // publisher for the marker of the ROVs
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_rovB_measurement;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_rovA_target; // publisher for the target of the ROVs
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_rovB_target;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_control_input_rovA; // publisher for the twist of the ROVs
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_control_input_rovB;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_triangle; // publisher for the triangle (with the targets of the ROVs)

    rclcpp::Subscription<message2::msg::Usbl>::SharedPtr sub_usbl; // (x,y) positions are given by the usbl
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_bouee_imu; // IMU quaternions of the buoy holding the USBL
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_mavros_rovA; // depth and orientation of the ROVs
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_mavros_rovB;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_rovA; // pose of the targets of the ROVs
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_rovB;
    rclcpp::Subscription< mavros_msgs::msg::OverrideRCIn>::SharedPtr sub_RC_rovA; // RC commands of the ROVs
    rclcpp::Subscription< mavros_msgs::msg::OverrideRCIn>::SharedPtr sub_RC_rovB;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void broadcast_tf_pose_i(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string& child_rov_frame,
                             const std::string& parent_frame_name);
    void publish_marker_i(const std::string& rov_frame, const std::string& frame_id, float alpha, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub);
    void publish_control_input_i(const std::string& rov_frame);
    void publish_target();
    void publish_robots_measurement();
    void publish_control_input();
    void publish_triangle();
    void publish_measured_position();

    void target_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string& name);
    void mavros_Callback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string& name);
    void ubslCallback(const message2::msg::Usbl::SharedPtr msg);
    void bouee_imu_Callback(const geometry_msgs::msg::Quaternion::SharedPtr msg);

    void target_rovA_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_Callback(msg, rovA_name);
    }

    void target_rovB_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_Callback(msg, rovB_name);
    }

    void mavros_rovA_Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        mavros_Callback(msg, rovA_name);
    }

    void mavros_rovB_Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        mavros_Callback(msg, rovB_name);
    }

    void RC_rovA_Callback(const mavros_msgs::msg::OverrideRCIn::SharedPtr msg)
    {
        RC_Callback(msg,control_input_rovA);
    }

    void RC_rovB_Callback(const mavros_msgs::msg::OverrideRCIn::SharedPtr msg)
    {
        RC_Callback(msg,control_input_rovB);
    }

    void RC_Callback(const mavros_msgs::msg::OverrideRCIn::SharedPtr msg, geometry_msgs::msg::TwistStamped& twist);

    void timer_callback(){
        publish_robots_measurement();
        publish_target();
        broadcast_tf_pose_i(pose_bouee, "bouee","map");
        publish_control_input();
        publish_triangle();
    }

};