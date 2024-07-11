#include <rov_rviz_ros2/replay.h>

// ------------------------------
// tool function
// ------------------------------

void pose_yaw_offset(geometry_msgs::msg::Pose& pose, float yaw_offset){
    // add the offset to the yaw of the pose
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    m.setRPY(roll, pitch, yaw+yaw_offset);
    m.getRotation(q);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
}

void set_color(visualization_msgs::msg::Marker& marker,  const std::string& rov_frame){
    // set the color of the marker depending on the ROV name
    if(rov_frame.find("clyde")!=std::string::npos){
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
    }else if(rov_frame.find("blinky")!=std::string::npos) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }else if(rov_frame.find("inky")!=std::string::npos){
        marker.color.r = 0.0;
        marker.color.g = 0.3;
        marker.color.b = 1.0;
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid name for ROV color: %s", rov_frame.c_str());
    }

    if(rov_frame.find("estimation")!=std::string::npos){
        marker.color.r += 0.2;
        marker.color.g += 0.2;
        marker.color.b += 0.2;
    }
}

// ------------------------------
// subscriber callback
// ------------------------------

void RovRviz::ubslCallback(const message2::msg::Usbl::SharedPtr msg)
{
    // Parse the message to the (x,y) position of the ROV (in the usbl frame)
    std_msgs::msg::Header header = msg->header;
    float id = msg->beacon_id;

    float range = msg->range;
    float azimuth = msg->azimuth;
    float elevation = msg->elevation;

    // local coordinates
    float x_local = range*cos(azimuth*M_PI/180)*cos(elevation*M_PI/180); // les deux X sont alignés
    float y_local = -range*sin(azimuth*M_PI/180)*cos(elevation*M_PI/180); // passage axe IMU (IMU avec Z vers le bas)
    float z_local = abs(pose_measured_rovA->pose.position.z);
    Eigen::Vector3d P_usbl_frame;
    P_usbl_frame << x_local, y_local, z_local+h_imu_usbl; // on rapporte ces coordonnées au centre de l'IMU (pas centre de rotation de la bouée par contre)

    // rotate with the roation matrix of the USBL
    double phi;
    double theta;
    double psy;
    tf2::Quaternion q(pose_bouee->pose.orientation.x, pose_bouee->pose.orientation.y,
                      pose_bouee->pose.orientation.z, pose_bouee->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(phi, theta, psy);

    // matrice transformation euler A
    double angle = - phi;
    Eigen::Matrix3d A0;
    A0 << 1, 0, 0,
            0, cos(angle), -sin(angle),
            0, sin(angle), cos(angle);

    angle = -theta;
    Eigen::Matrix3d A1;
    A1 << cos(angle), 0, sin(angle),
            0, 1, 0,
            -sin(angle), 0, cos(angle);

    angle = -psy;
    Eigen::Matrix3d A2;
    A2 << cos(angle), -sin(angle), 0,
            sin(angle), cos(angle), 0,
            0, 0, 1;

    Eigen::Matrix3d A = A2*A1*A0;

    // matrice rotation biais orientation entre IMU et USBL
    angle = biais_usbl_IMU;
    Eigen::Matrix3d B;
    B << cos(angle), -sin(angle), 0,
            sin(angle), cos(angle), 0,
            0, 0, 1;

    Eigen::Vector3d P_global_frame = B*A*P_usbl_frame;

    if(id == 2){ // it is rovA
        pose_measured_rovA->header = header;
        pose_measured_rovA->pose.position.x = P_global_frame(0);
        pose_measured_rovA->pose.position.y = P_global_frame(1);

    } else if(id == 1){ // it is rovB
        pose_measured_rovB->header = header;
        pose_measured_rovB->pose.position.x = P_global_frame(0);
        pose_measured_rovB->pose.position.y = P_global_frame(1);

    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid id for ROV id: %f", id);
    }
}

void RovRviz::mavros_Callback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string& name)
{
    // Parse the message to the depth and orientation of the ROV
    std_msgs::msg::Header header;
    geometry_msgs::msg::Pose pose;
    header = msg->header;
    pose = msg->pose.pose;
//    mission_time = msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;

    if(name == rovA_name){
        pose_measured_rovA->header = header;
        pose_measured_rovA->pose.orientation = pose.orientation;
        pose_yaw_offset(pose_measured_rovA->pose,-M_PI/2); // TODO why is there a 90 deg offset on the heading data ?
        pose_measured_rovA->pose.position.z = pose.position.z;

    }else if(name == rovB_name){
        pose_measured_rovB->header = header;
        pose_measured_rovB->pose.orientation = pose.orientation;
        pose_measured_rovB->pose.position.z = pose.position.z;
        pose_yaw_offset(pose_measured_rovB->pose,-M_PI/2);  // TODO why is there a 90 deg offset on the heading data ?

    }else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid name for ROV: %s", name.c_str());
    }
}

void  RovRviz::target_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string& name){
    float x_local = msg->pose.position.x;
    float y_local = msg->pose.position.y;
    float z = -msg->pose.position.z;
    float x_global = x_local;
    float y_global = y_local;

    if (name == rovA_name){
        pose_target_rovA = msg;
        pose_target_rovA->pose.position.x = x_global;
        pose_target_rovA->pose.position.y = y_global;
        pose_target_rovA->pose.position.z = z; // inverse the z axis
        pose_target_rovA->pose.orientation = msg->pose.orientation;
    }else if(name == rovB_name){
        pose_target_rovB = msg;
        pose_target_rovB->pose.position.x = x_global;
        pose_target_rovB->pose.position.y = y_global;
        pose_target_rovB->pose.position.z = z; // inverse the z axis
        pose_target_rovB->pose.orientation = msg->pose.orientation;
    }else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid name for ROV target: %s", name.c_str());
    }
}

void RovRviz::RC_Callback(const mavros_msgs::msg::OverrideRCIn::SharedPtr msg, geometry_msgs::msg::TwistStamped& control_input){
    // Parse the message to the linear control_input of the ROV
    // a pwm of 500 is about 40N of thrust
    float k1= 40.;
    control_input.twist.linear.x = k1*(float)(msg->channels[4]-1500)/500.;
    control_input.twist.linear.y = -k1*(float)(msg->channels[5]-1500)/500.; // this channel is inverted
    control_input.twist.linear.z = k1*(float)(msg->channels[2]-1500)/500.;
    control_input.twist.angular.z = k1*(float)(msg->channels[3]-1500)/500.;
    control_input.twist.angular.y = k1*(float)(msg->channels[0]-1500)/500.;
    control_input.twist.angular.x = k1*(float)(msg->channels[1]-1500)/500.;
}

// ------------------------------
// publisher related function
// ------------------------------

void RovRviz::publish_robots_measurement(){
    broadcast_tf_pose_i(pose_measured_rovA, rovA_name,"map");
    broadcast_tf_pose_i(pose_measured_rovB, rovB_name,"map");
    publish_marker_i(rovA_name, rovA_name, 1, pub_marker_rovA_measurement);
    publish_marker_i(rovB_name, rovB_name, 1, pub_marker_rovB_measurement);
}

void RovRviz::publish_target(){
    broadcast_tf_pose_i(pose_target_rovA, "target_"+rovA_name,"map");
    broadcast_tf_pose_i(pose_target_rovB, "target_"+rovB_name,"map");
    publish_marker_i(rovA_name,"target_"+rovA_name,0.25,pub_marker_rovA_target);
    publish_marker_i(rovB_name,"target_"+rovB_name,0.25,pub_marker_rovB_target);
}

void RovRviz::broadcast_tf_pose_i(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                                  const std::string& child_frame_name,
                                  const std::string& parent_frame_name)
{
    // if the message is not empty
    if(msg) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame_name.c_str();
        t.child_frame_id = child_frame_name.c_str();

        t.transform.translation.x = msg->pose.position.x;
        t.transform.translation.y = msg->pose.position.y;
        t.transform.translation.z = msg->pose.position.z;
        t.transform.rotation.x = msg->pose.orientation.x;
        t.transform.rotation.y = msg->pose.orientation.y;
        t.transform.rotation.z = msg->pose.orientation.z;
        t.transform.rotation.w = msg->pose.orientation.w;

        tf_broadcaster_->sendTransform(t);
    }
}

void RovRviz::publish_marker_i(const std::string& rov_frame, const std::string& frame_id, float alpha, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub){
    visualization_msgs::msg::Marker marker_rov;
    marker_rov.header.frame_id = frame_id;
    marker_rov.header.stamp =  this->get_clock()->now();
    marker_rov.ns = "/"+rov_frame;
    marker_rov.id = 0;
    marker_rov.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_rov.action = visualization_msgs::msg::Marker::ADD;
    marker_rov.mesh_resource = "package://rov_rviz_ros2/meshs/ROV_simple.stl";

    tf2::Quaternion q;
    q.setRPY(0, 0, -M_PI/2); // rotation of the mesh
    marker_rov.pose.orientation.x = q.x();
    marker_rov.pose.orientation.y = q.y();
    marker_rov.pose.orientation.z = q.z();
    marker_rov.pose.orientation.w = q.w();

    marker_rov.pose.position.z = -0.1;

    marker_rov.scale.x = 0.01;
    marker_rov.scale.y = 0.01;
    marker_rov.scale.z = 0.01;
    marker_rov.color.a = alpha;

    set_color(marker_rov, frame_id);

    pub->publish(marker_rov);
}

void RovRviz::publish_control_input_i(const std::string& rov_frame){
    // publish the twist of the ROV
    if(rov_frame == rovA_name) {
        control_input_rovA.header.stamp =  this->get_clock()->now();
        control_input_rovA.header.frame_id = rov_frame;
        pub_control_input_rovA->publish(control_input_rovA);
    }else if(rov_frame == rovB_name) {
        control_input_rovB.header.stamp =  this->get_clock()->now();
        control_input_rovB.header.frame_id = rov_frame;
        pub_control_input_rovB->publish(control_input_rovB);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid name for ROV control_input: %s", rov_frame.c_str());
    }
}

void RovRviz::publish_control_input(){
    publish_control_input_i(rovA_name);
    publish_control_input_i(rovB_name);
}

void RovRviz::publish_triangle() {
    if (pose_measured_rovA && pose_measured_rovB) {
        // create marker message
        visualization_msgs::msg::Marker marker_triangle;
        marker_triangle.header.frame_id = "map";
        marker_triangle.header.stamp = this->get_clock()->now();
        marker_triangle.id = 0;
        marker_triangle.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_triangle.action = visualization_msgs::msg::Marker::ADD;
        marker_triangle.scale.x = 0.1;
        marker_triangle.color.a = 1.0;
        marker_triangle.color.r = 0.0;
        marker_triangle.color.g = 1.0;
        marker_triangle.color.b = 0.0;

        marker_triangle.pose.orientation.w = 1.0;

        // add the points
        geometry_msgs::msg::Point pA;
        pA.x = pose_target_rovA->pose.position.x;
        pA.y = pose_target_rovA->pose.position.y;
        pA.z = pose_target_rovA->pose.position.z;
        marker_triangle.points.push_back(pA);

        geometry_msgs::msg::Point pB;
        pB.x = pose_target_rovB->pose.position.x;
        pB.y = pose_target_rovB->pose.position.y;
        pB.z = pose_target_rovB->pose.position.z;
        marker_triangle.points.push_back(pB);

        geometry_msgs::msg::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
        marker_triangle.points.push_back(p);
        marker_triangle.points.push_back(pA);

        // publish the marker
        pub_triangle->publish(marker_triangle);
    }
}


void RovRviz::bouee_imu_Callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
    // Conversion quaternions in rotation matrix
//    Eigen::Quaterniond q_eigen(msg->w, msg->x, msg->y, msg->z);
//    usbl_rotation_matrix.setIdentity();
//    usbl_rotation_matrix = q_eigen.toRotationMatrix();
    pose_bouee->pose.orientation.x = msg->x;
    pose_bouee->pose.orientation.y = msg->y;
    pose_bouee->pose.orientation.z = msg->z;
    pose_bouee->pose.orientation.w = msg->w;
}


//    self.qx_boue = msg->x
//    self.qy_boue = msg->y
//    self.qz_boue = msg->z
//    self.qw_boue = msg->w
//
//    orientq=(self.qw_boue, self.qx_boue, self.qy_boue, self.qz_boue)
//    ### Conversion quaternions in rotation matrix
//    self.Phi_bouee_rad, self.Theta_bouee_rad, self.Psy_bouee_rad = Rotation.from_quat([orientq[1], orientq[2], orientq[3],   orientq[0]]).as_euler("xyz") # Roulis, Tangage, Lacet

//# coordonnée local
//    X0 = self.L_x_beacon[beacon_ID] # les deux X sont alignés
//    Y0 = -self.L_y_beacon[beacon_ID]  # passage axe IMU (IMU avec Z vers le bas)
//    Z0 = abs(self.L_z_beacon[beacon_ID])
//
//
//    Vx = np.array([X0 , Y0 , Z0]) # vecteur coordonnée local de l'USBL
//
//# on rapporte ces coordonnées au centre de l'IMU (pas centre de rotation de la bouée par contre)
//    Vx = (Vx + np.array([0.0 , 0.0 , self.h_imu_usbl]) )

//# angle d'euleur
//    phi, theta, psi = self.Phi_bouee_rad, self.Theta_bouee_rad, self.Psy_bouee_rad
//#print("phi = ",phi*180/pi, ' theta = ', theta*180/pi, ' psi =', psi*180/pi )
//
//# matrice transformation euler A
//    angle = - phi
//    A0 = np.array([[1 ,0 , 0 ],
//    [0, np.cos(angle), - np.sin(angle)],
//    [0, np.sin(angle), np.cos(angle)] ])
//
//    angle = -theta
//    A1 = np.array([[ np.cos(angle),0 , np.sin(angle) ],
//    [0,1, 0],
//    [-np.sin(angle), 0, np.cos(angle)] ])
//    angle = -psi
//    A2 = np.array([
//    [ np.cos(angle), - np.sin(angle), 0],
//    [ np.sin(angle), np.cos(angle),0 ],
//    [0,0,1] ])
//    A = np.dot(A2,np.dot(A1,A0))
//
//
//# matrice rotation biais orientation entre IMU et USBL
//    angle = self.biais_usbl_IMU
//    B = np.array([[np.cos(angle), - np.sin(angle), 0],
//    [np.sin(angle), np.cos(angle), 0],
//    [0, 0, 1]])
//
//# changement de repere
//    Vx_global = np.dot(B, np.dot(A,Vx))
//
//#print('Xg = ', Vx_global[0], '/ Yg = ', Vx_global[1], '/ Zg = ', Vx_global[2])
//
//# stockage
//    self.L_x_global_beacon[beacon_ID]  = Vx_global[0]
//    self.L_y_global_beacon[beacon_ID]  = Vx_global[1]
//    self.L_z_global_beacon[beacon_ID]  = Vx_global[2]

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr <RovRviz> node = std::make_shared<RovRviz>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
