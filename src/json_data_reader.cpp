//
// Created by morgan on 12/09/23.
// read the json file of the ROV simulation to publish the data to the ROS system for a Rviz display
//
#include <chrono>
#include <string>
#include <fstream>
#include <jsoncpp/json/json.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>


using namespace std::chrono_literals;

class NodeDataReader: public rclcpp::Node{
public:
    NodeDataReader(): Node("data_reader"){
        // get the filepath from the parameters
        this->declare_parameter("file_path","no_path");
        std::string filepath = this->get_parameter("file_path").as_string();

        // print the filepath
        RCLCPP_INFO(this->get_logger(), "filepath: %s", filepath.c_str());

        // import the Json file and parse it
        std::ifstream file(filepath,std::ifstream::binary);
        Json::Value data;
        file >> data;

        N = data["header"]["N"].asInt(); // number of ROVs
        dt = data["header"]["dt"].asDouble(); // time step
        simu_data = data["simu_data"];
        k=0;
        k_max = simu_data.size();

        // create a tf publisher to give all the pose of the ROVs
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ROV marker publisher
        publisher_marker_1 = this->create_publisher<visualization_msgs::msg::Marker>("/rov1_marker",10);
        publisher_marker_2 = this->create_publisher<visualization_msgs::msg::Marker>("/rov2_marker",10);

        // The timer callback to print text in the console
        timer_ = this->create_wall_timer(500ms, std::bind(&NodeDataReader::timer_callback, this));
    }
private:
    void timer_callback(){
        // get current time
        rclcpp::Time t_ = this->now();
        if(k<k_max) {
            // extract data from the json file
            Json::Value simu_data_i = simu_data[k]["L_eta"];

            // send the tf transformations for each ROV
            for (int j = 0; j < N; j++) {
                geometry_msgs::msg::TransformStamped ts;
                ts.header.stamp = t_;
                ts.header.frame_id = "world";
                ts.child_frame_id = "rov_" + std::to_string(j+1);

                ts.transform.translation.x = simu_data_i[j][0].asDouble();
                ts.transform.translation.y = simu_data_i[j][1].asDouble();
                ts.transform.translation.z = simu_data_i[j][2].asDouble();

                float roll = simu_data_i[j][3].asDouble();
                float pitch = simu_data_i[j][4].asDouble();
                float yaw = simu_data_i[j][5].asDouble();
                RCLCPP_INFO(this->get_logger(), "yaw: %f", yaw);

                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);
                ts.transform.rotation.x = q.x();
                ts.transform.rotation.y = q.y();
                ts.transform.rotation.z = q.z();
                ts.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(ts);
                k += 1;
            }
        }else{
            RCLCPP_INFO(this->get_logger(), "End of the simulation");
        }

        // display the marker of the ROVs
        visualization_msgs::msg::Marker marker;

        marker.header.stamp = t_;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.mesh_resource = "package://rov_rviz/meshs/ROV_simple.stl";

        tf2::Quaternion q;
        q.setRPY(0, 0, -M_PI/2);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.header.frame_id = "rov_1";
        marker.ns = "rov_1";
        publisher_marker_1->publish(marker);

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.header.frame_id = "rov_2";
        marker.ns = "rov_2";
        publisher_marker_2->publish(marker);

    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker_1; // 2 maker publisher can be used
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker_2;

    int N;
    double dt;
    int k; // number of time steps
    int k_max; // maximum number of time steps
    Json::Value simu_data;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<NodeDataReader> node = std::make_shared<NodeDataReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}