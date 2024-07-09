#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <chrono>
#include <memory>
#include "tools.h"
#include <sstream>
#include <string>
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "rosgraph_msgs/msg/clock.hpp" 
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>

class VectorFieldController : public rclcpp::Node
{
public:
  VectorFieldController()
  : Node("vector_field_controller_node")
  {
    initialize_common_params();
    refresh_common_params();
    if(debug){
      debug_params();
    }

    //Tfs listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //QOS:
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    //subscribers
    keep_alive_subscriber = this->create_subscription<std_msgs::msg::Bool>(keep_alive_topic, sensor_qos, std::bind(&VectorFieldController::aliveCallback, this, std::placeholders::_1));
    attractive_point_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(attractive_point_topic, sensor_qos, std::bind(&VectorFieldController::attractivePointCallback, this, std::placeholders::_1));
    repulsive_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(repulsive_scan_topic, sensor_qos, std::bind(&VectorFieldController::repulsiveScanCallback, this, std::placeholders::_1));
    //Clock
    if(use_sim_time){
        clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", sensor_qos, std::bind(&VectorFieldController::ClockCallback, this, std::placeholders::_1));
    }

    //publishers
    cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>(publish_cmd_topic, 10);
    cmd_vector_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(publish_command_feedback_topic, 10);
    if(publish_field){
      vector_field_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(publish_field_topic, 10);
    }

    //main loop
    timer_ = create_wall_timer(std::chrono::milliseconds(int(1000/rate)), std::bind(&VectorFieldController::compute_field, this));

  }

private:

  void aliveCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    keep_alive_msg = msg;
  }

  void attractivePointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    attractive_point_msg = msg;
    attractive_point = msg->point; //directly update attractive point
  }

  void repulsiveScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    raw_scan_mutex.lock();
    repulsive_scan_msg = msg;
    raw_scan_mutex.unlock();
  }

  void ClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
      // Update the current timestamp when the clock message is received
      simu_timestamp = msg->clock;
  }

  void update_stamp(){
    if(use_sim_time){ //if use_sim_time = true
        current_global_stamp = simu_timestamp;
    }
    else{
        current_global_stamp = clock->now();
    }
  }

  void compute_field(){
    std::stringstream debug_ss;

    debug_ss << "\n\n///COMPUTE FIELD STARTED///" << std::endl;

    if(repulsive_scan_msg != nullptr && attractive_point_msg != nullptr){
      debug_ss << "Getting robot state and repulsive obstacles from TFs and scan on topic '"<< repulsive_scan_topic <<"'"<<std::endl;
      int res = update_tf(debug_ss);
      if(update_2Dpoints_from_scan(vector_map_robot.x,vector_map_robot.y)){
        debug_ss << "Scan successfully transformed to repulsive points (Transformation: `"<< robot_frame << "' to '" << map_frame <<"' got)" << std::endl;
        geometry_msgs::msg::Vector3 wanted_speed;
        //compute speed according to robot position
        debug_ss  << "Atractive point (x,y): (" << attractive_point.x<< " ; " << attractive_point.y<< ")" << std::endl;
        minus_grad(wanted_speed,vector_map_robot.x,vector_map_robot.y);
        //publish commands
        publish_cmd(wanted_speed.x,wanted_speed.y,0.0);
        if(publish_field){
          publishVectorField();
        }
      }
      else{
        debug_ss << "Coudn't transform the scan to repulsive points, aborting..." << std::endl;
      }
    }
    else{
      debug_ss << "waiting to receive scan on topic '"<< repulsive_scan_topic <<"'"<<std::endl;
      debug_ss << "waiting to receive point on topic '"<< attractive_point_topic <<"'"<<std::endl;
    }
    
    if(debug){
      std::string debug_msg = debug_ss.str();
      write_debug(debug_file_path, debug_msg);
    }
  }

  int update_2Dpoints_from_scan(double offset_x, double offset_y){
    // Clear all elements of the former vector
    repulsive_points.clear();
    std::vector<double> ranges;

    raw_scan_mutex.lock();
    int scan_reso= repulsive_scan_msg->ranges.size();
    for(int i=0; i<scan_reso;i++){
      ranges.push_back(repulsive_scan_msg->ranges[i]);
    }
    raw_scan_mutex.unlock();

    for(int i=0;i<scan_reso;i++){
      auto new_point = std::make_shared<geometry_msgs::msg::Point>();
      double pos_x = 0.0;
      double pos_y = 0.0;
      double angle = index_to_angle(i,scan_reso);
      double range = ranges[i];
      if(range != INFINITY){ //we skip infinity points, they are not obstacles
        //get pos of obstacle in the robot frame
        get_pos(pos_x, pos_y,angle,range,0.0,0.0);
        //convert robot frame pose to map frame with TFs
        new_point->x = pos_x + offset_x; 
        new_point->y = pos_y + offset_y;
        //add repulsive point
        repulsive_points.push_back(*new_point);
        //RCLCPP_INFO(this->get_logger(), "x: %f, y:%f",new_point->x,new_point->y);
      }
    }
    return 1;
  }

  /*void grad_repulse(){//we compute the repulsive part only once
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(2, 1);
    P(0,0) = off_vect_x;
    P(1,0) = off_vect_y;
    
    return 
  }*/

  void minus_grad(geometry_msgs::msg::Vector3 &wanted_speed, double pos_x, double pos_y){
    double ka = k_attract;
    double kr = k_repulse;
    double grad_v_x = 0.0;
    double grad_v_y = 0.0;

    //repulsive points
    
    for(int i=0; i<repulsive_points.size(); i++){
      double norm_term = sqrt(pow(pos_x-repulsive_points[i].x,2)+pow(pos_y-repulsive_points[i].y,2));
      double exp_term = exp(kr-norm_term);
      if(norm_term > 0.0){
          grad_v_x += (pos_x-repulsive_points[i].x)*(exp_term/norm_term);
          grad_v_y += (pos_y-repulsive_points[i].y)*(exp_term/norm_term);
      }
    }

    //attractive point
    double norm_ka_term = pow(sqrt(pow(pos_x-attractive_point.x,2) + pow(pos_y-attractive_point.y,2)),ka-2);
    grad_v_x += std::clamp((pos_x-attractive_point.x)*(-ka*norm_ka_term),-1.0,1.0);
    grad_v_y += std::clamp((pos_y-attractive_point.y)*(-ka*norm_ka_term),-1.0,1.0);

    wanted_speed.x = std::clamp(grad_v_x,-1.0,1.0);
    wanted_speed.y = std::clamp(grad_v_y,-1.0,1.0);
  }

  int update_tf(std::stringstream &debug_ss){
    try {
      geometry_msgs::msg::TransformStamped transf =tf_buffer_->lookupTransform(map_frame, robot_frame, tf2::TimePointZero);
      geometry_msgs::msg::Vector3 vector_newframe_sensor = transf.transform.translation;  //translation vector from new_frame to frame_sensor
      geometry_msgs::msg::Vector3 rotate_newframe_sensor = adapt_angle(quaternion_to_euler3D(transf.transform.rotation));
      vector_map_robot.x = vector_newframe_sensor.x;
      vector_map_robot.y = vector_newframe_sensor.y;
      vector_map_robot.z = rotate_newframe_sensor.z;
      debug_ss  << "robots state (x,y,heading): (" << vector_map_robot.x<< " ; " << vector_map_robot.y<< " ; " << vector_map_robot.z<< ")" << std::endl;
    }
    catch (const std::exception& e) {
      debug_ss  << "Couldn't get transformation " << robot_frame << " --> " << map_frame << std::endl;
    }
  }

  void publishVectorField()
  {
      visualization_msgs::msg::MarkerArray marker_array;

      // Set the frame_id and timestamp for the marker array
      int nb_marker = field_grid_reso_x*field_grid_reso_y;
      marker_array.markers.resize(nb_marker);
      double dx = field_grid_x_elong/(field_grid_reso_x-1);
      double dy = field_grid_y_elong/(field_grid_reso_y-1);
      for (int i = 0; i < nb_marker; ++i)
      {
          visualization_msgs::msg::Marker& marker = marker_array.markers[i];

          // Set the marker type to ARROW
          marker.type = visualization_msgs::msg::Marker::ARROW;

          // Set the marker action to ADD
          marker.action = visualization_msgs::msg::Marker::ADD;

          // Set the frame_id and timestamp for the marker
          marker.header.frame_id = "map"; // or your desired frame_id
          marker.header.stamp = current_global_stamp;

          // Set the marker ID
          marker.id = i;

          // Set the scale of the arrow
          marker.scale.x = 0.03; // shaft diameter
          marker.scale.y = 0.03; // head diameter
          marker.scale.z = 0.03; // head lenght

          // Set the color of the arrow
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 0.8;

          // Set the start and end points of the arrow
          geometry_msgs::msg::Point start_point, end_point;
          start_point.x = vector_map_robot.x+(fmod(i,field_grid_reso_x)*dx-field_grid_x_elong/2); //vector_map_robot.x+(fmod(i*dx,field_grid_x_elong)-field_grid_x_elong/2);
          start_point.y = vector_map_robot.y+((int(i/field_grid_reso_x)*dy)-field_grid_y_elong/2);
          start_point.z = 0.0;
          geometry_msgs::msg::Vector3 pos_speed;
          minus_grad(pos_speed,start_point.x,start_point.y);
          end_point.x = start_point.x + pos_speed.x*arrows_size_multiplier;
          end_point.y = start_point.y + pos_speed.y*arrows_size_multiplier;
          end_point.z = 0.0;
          marker.points.push_back(start_point);
          marker.points.push_back(end_point);
      }

      // Publish the marker array
      vector_field_publisher->publish(marker_array);
  }

  void publish_cmd(double vx, double vy, double w){
    // create a twist message
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = vx;
    twist_msg.linear.y = vy;
    twist_msg.angular.z = w;
    //TDM Put commands in robots frame (use transform 2D points)

    // create a twist stamped message
    auto twist_stamped_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    update_stamp();
    twist_stamped_msg->header.stamp = current_global_stamp;
    twist_stamped_msg->header.frame_id = robot_frame;
    twist_stamped_msg->twist = twist_msg;
    twist_stamped_msg->twist.linear.x *= 3.0*arrows_size_multiplier;
    twist_stamped_msg->twist.linear.y *= 3.0*arrows_size_multiplier;

    // publish the twist stamped message
    cmd_publisher->publish(twist_msg);
    cmd_vector_publisher->publish(*twist_stamped_msg);
  }

  void initialize_common_params(){
      this->declare_parameter("rate", 20.0);
      this->declare_parameter("repulsive_scan_topic", "scan");
      this->declare_parameter("attractive_point_topic", "clicked_point");
      this->declare_parameter("publish_cmd_topic", "cmd_vel");
      this->declare_parameter("publish_command_feedback_topic", "cmd_vel_vector");
      this->declare_parameter("keep_alive_topic", "vector_field_controller_alive");
      this->declare_parameter("keep_alive_timeout", 1.0);
      this->declare_parameter("robot_frame", "base_link");
      this->declare_parameter("map_frame", "map");
      this->declare_parameter("control_type", "omni");
      this->declare_parameter("k_repulse", 1.0);
      this->declare_parameter("k_attract", 1.0);
      this->declare_parameter("v_min", 0.1);
      this->declare_parameter("v_max", 1.0);
      this->declare_parameter("publish_field", false);
      this->declare_parameter("publish_field_topic", "vector_field");
      this->declare_parameter("field_grid_reso_x", 10);
      this->declare_parameter("field_grid_reso_y", 10);
      this->declare_parameter("field_grid_x_elong", 5.0);
      this->declare_parameter("field_grid_y_elong", 5.0);
      this->declare_parameter("arrows_size_multiplier", 0.1);
      this->declare_parameter("debug", false);
      this->declare_parameter("debug_file_path", "ros2_vector_field_controller_debug.txt");
  }

  void refresh_common_params(){
      this->get_parameter("use_sim_time",use_sim_time); //managed by launch file
      this->get_parameter("rate", rate);
      this->get_parameter("repulsive_scan_topic", repulsive_scan_topic);
      this->get_parameter("attractive_point_topic", attractive_point_topic);
      this->get_parameter("publish_cmd_topic", publish_cmd_topic);
      this->get_parameter("publish_command_feedback_topic", publish_command_feedback_topic);
      this->get_parameter("keep_alive_topic", keep_alive_topic);
      this->get_parameter("keep_alive_timeout", keep_alive_timeout);
      this->get_parameter("robot_frame", robot_frame);
      this->get_parameter("map_frame", map_frame);
      this->get_parameter("control_type", control_type);
      this->get_parameter("k_repulse", k_repulse);
      this->get_parameter("k_attract", k_attract);
      this->get_parameter("v_min", v_min);
      this->get_parameter("v_max", v_max);
      this->get_parameter("publish_field", publish_field);
      this->get_parameter("publish_field_topic", publish_field_topic);
      this->get_parameter("field_grid_reso_x", field_grid_reso_x);
      this->get_parameter("field_grid_reso_y", field_grid_reso_y);
      this->get_parameter("field_grid_x_elong", field_grid_x_elong);
      this->get_parameter("field_grid_y_elong", field_grid_y_elong);
      this->get_parameter("arrows_size_multiplier", arrows_size_multiplier);
      this->get_parameter("debug",debug);
      this->get_parameter("debug_file_path",debug_file_path);
  }

  void debug_params(){
      std::stringstream debug_ss;
      debug_ss << "\nPARAMETERS:"
              << "\nuse_sim_time: " << use_sim_time
              << "\nrate: " << rate
              << "\nrepulsive_scan_topic: " << repulsive_scan_topic
              << "\nattractive_point_topic: " << attractive_point_topic
              << "\npublish_cmd_topic: " << publish_cmd_topic
              << "\npublish_command_feedback_topic: " << publish_command_feedback_topic
              << "\nkeep_alive_topic: " << keep_alive_topic
              << "\nkeep_alive_timeout: " << keep_alive_timeout
              << "\nrobot_frame: " << robot_frame
              << "\nmap_frame: " << map_frame
              << "\ncontrol_type: " << control_type
              << "\nk_repulse: " << k_repulse
              << "\nk_attract: " << k_attract
              << "\nv_min: " << v_min
              << "\nv_max: " << v_max
              << "\npublish_field: " << publish_field
              << "\npublish_field_topic: " << publish_field_topic
              << "\nfield_grid_reso_x: " << field_grid_reso_x
              << "\nfield_grid_reso_y: " << field_grid_reso_y
              << "\nfield_grid_x_elong: "<< field_grid_x_elong
              << "\nfield_grid_y_elong: "<< field_grid_y_elong
              << "\narrows_size_multiplier: "<< arrows_size_multiplier
              << "\ndebug: " << debug
              << "\ndebug_file_path: " << debug_file_path;
      std::string debug_msg = debug_ss.str();
      write_debug(debug_file_path, debug_msg, false);
      //RCLCPP_INFO(this->get_logger(), "%s",debug_msg.c_str());
  }

  void write_debug(std::string file, std::string text,  bool append = true){
      //RCLCPP_INFO(this->get_logger(), "%s",text.c_str());
      std::lock_guard<std::mutex> lock(mutex_debug_file);
      std::ofstream debug_file_ss(file, append ? std::ios::app : std::ios::trunc);
      if (debug_file_ss.is_open()){
          debug_file_ss << text;
          debug_file_ss.close();
      }
      else{
          RCLCPP_ERROR(this->get_logger(), "Could not open file for debugging: %s",file.c_str());
      }
  }

  //parameters
  bool use_sim_time = false; //used
  double rate; //used
  std::string repulsive_scan_topic; //used
  std::string attractive_point_topic; //used
  std::string publish_cmd_topic; //used
  std::string publish_command_feedback_topic; //used
  std::string keep_alive_topic; //used
  double keep_alive_timeout;
  std::string robot_frame; //used
  std::string map_frame; //used
  std::string control_type;
  double k_repulse; //used
  double k_attract; //used
  double v_min;
  double v_max;
  bool publish_field; //used
  std::string publish_field_topic; //used
  int field_grid_reso_x; //used
  int field_grid_reso_y; //used
  double field_grid_x_elong; //used
  double field_grid_y_elong; //used
  double arrows_size_multiplier; //used
  bool debug; //used
  std::string debug_file_path; //used

  //ROS variables
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr keep_alive_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr attractive_point_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr repulsive_scan_subscriber;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vector_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vector_field_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  //store received data
  sensor_msgs::msg::LaserScan::SharedPtr repulsive_scan_msg;
  geometry_msgs::msg::PointStamped::SharedPtr attractive_point_msg; //not directly used, we use directly 'attractive_point'
  std_msgs::msg::Bool::SharedPtr keep_alive_msg;

  //store data to compute
  std::vector<geometry_msgs::msg::Point> repulsive_points;
  geometry_msgs::msg::Point attractive_point;
  geometry_msgs::msg::Vector3 vector_map_robot; //(x,y,heading) offsets

  //concurrence
  std::mutex mutex_debug_file;
  std::mutex raw_scan_mutex;

  //transformations listening
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  //clock
  builtin_interfaces::msg::Time simu_timestamp; //used for simuation
  rclcpp::Clock::SharedPtr clock; //used if not a simulation
  builtin_interfaces::msg::Time current_global_stamp; //store current time

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VectorFieldController>());
  rclcpp::shutdown();
  return 0;
}