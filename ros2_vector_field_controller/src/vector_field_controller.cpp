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

bool flow_debug = false; //used to print debug lines

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
    keep_alive_subscriber = this->create_subscription<std_msgs::msg::Bool>(input_keep_alive_topic, sensor_qos, std::bind(&VectorFieldController::aliveCallback, this, std::placeholders::_1));
    attractive_point_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(input_attractive_point_topic, sensor_qos, std::bind(&VectorFieldController::attractivePointCallback, this, std::placeholders::_1));
    repulsive_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(input_repulsive_scan_topic, sensor_qos, std::bind(&VectorFieldController::repulsiveScanCallback, this, std::placeholders::_1));
    cmd_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(input_cmd_topic, sensor_qos, std::bind(&VectorFieldController::cmdInputCallback, this, std::placeholders::_1));
    //Clock
    if(use_sim_time){
        clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", sensor_qos, std::bind(&VectorFieldController::ClockCallback, this, std::placeholders::_1));
    }
    else{
        clock = this->get_clock();
    }

    //publishers
    cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic, 10);
    cmd_vector_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_command_feedback_topic, 10);
    if(publish_field){
      vector_field_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_field_topic, 10);
    }

    //main loop
    timer_ = create_wall_timer(std::chrono::milliseconds(int(1000/rate)), std::bind(&VectorFieldController::compute_field, this));

  }

private:

  void aliveCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    keep_alive_mutex.lock();
    keep_alive_msg = msg;
    keep_alive_mutex.unlock();
    update_stamp();
    keep_alive_msg_last_stamp= current_global_stamp;
  }

  void attractivePointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    attractive_point_mutex.lock();
    attractive_point_msg = msg;
    attractive_point_mutex.unlock();
  }

  void repulsiveScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    raw_scan_mutex.lock();
    repulsive_scan_msg = msg;
    raw_scan_mutex.unlock();
  }

  void cmdInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    input_cmd_mutex.lock();
    input_cmd_msg = msg;
    input_cmd_mutex.unlock();
    update_stamp();
    input_cmd_msg_last_stamp = current_global_stamp;
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

  void check_alive(){
    if(keep_alive_msg != nullptr){
      keep_alive_mutex.lock();
      keep_alive = keep_alive_msg->data;
      keep_alive_mutex.unlock();
    }
    else{
      keep_alive = false;
    }
  }

  void compute_field(){
    std::stringstream debug_ss;

    debug_ss << "\n\n///COMPUTE FIELD STARTED///" << std::endl;
    
    if(repulsive_scan_msg != nullptr && (attractive_point_msg != nullptr || input_cmd_msg!=nullptr)){
      //extract received data, and store it in class variable
      bool attractive_point_received = false;
      bool cmd_msg_received = false;
      if(attractive_point_msg != nullptr){
        attractive_point_mutex.lock();
        attractive_point = attractive_point_msg->point;
        attractive_point_mutex.unlock();
        attractive_point_received = true;
        debug_ss << "Attractive point Received! (x,y): (" << attractive_point.x<< " ; " << attractive_point.y<< ")" << std::endl;
      }
      if(input_cmd_msg!=nullptr){
        input_cmd_mutex.lock();
        input_cmd = *input_cmd_msg;
        input_cmd_mutex.unlock();
        cmd_msg_received = true;
        debug_ss << "Speed commands Received! (Priority over Attractive point) (vx,vy,w): (" << input_cmd.linear.x<< " ; " << input_cmd.linear.y << " ; " << input_cmd.angular.z << ")" << std::endl;
      }
      if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK1");}
      debug_ss << "Getting robot state and repulsive obstacles from TFs and scan on topic '"<< input_repulsive_scan_topic <<"'"<<std::endl;
      int res = update_tf(debug_ss);
      if(update_2Dpoints_from_scan(vector_map_robot.x,vector_map_robot.y)){
        debug_ss << "Scan successfully transformed to repulsive points (Transformation: `"<< robot_frame << "' to '" << map_frame <<"' got)" << std::endl;
        geometry_msgs::msg::Vector3 wanted_speed_world_frame;
        //compute speed according to robot position
        if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK2");}
        minus_grad(wanted_speed_world_frame,vector_map_robot.x,vector_map_robot.y,attractive_point_received,cmd_msg_received,true,debug_ss);
        debug_ss  << "Speed to follow (with collision avoidance) (world's frame)(vx,vy,w): (" << wanted_speed_world_frame.x<< " ; " << wanted_speed_world_frame.y<< " ; " << wanted_speed_world_frame.z<< ")" << std::endl;
        //rotate vector from world to robot_frame
        geometry_msgs::msg::Vector3 wanted_speed_rob_frame;
        if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK3");}
        rotate2Dvect(wanted_speed_rob_frame,wanted_speed_world_frame,-vector_map_robot.z);
        debug_ss  << "Speed to follow (with collision avoidance) (robot's frame)(vx,vy,w): (" << wanted_speed_rob_frame.x<< " ; " << wanted_speed_rob_frame.y<<" ; " << wanted_speed_rob_frame.z<< ")" << std::endl;
        //publish commands
        if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK4");}
        publish_cmd(wanted_speed_rob_frame.x,wanted_speed_rob_frame.y,0.0);
        debug_ss  << "Speed command Published on topic: " << output_cmd_topic << std::endl;
        debug_ss  << "Speed command Visualisation Published on topic: " << output_command_feedback_topic << std::endl;
        if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK5");}
        if(publish_field){
          publishVectorField(attractive_point_received,cmd_msg_received);
          debug_ss  << "Field Published on topic: " << output_field_topic << std::endl;
        }
      }
      else{
        debug_ss << "Coudn't transform the scan to repulsive points, aborting..." << std::endl;
      }
      if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK6");}
    }
    else{
      if(repulsive_scan_msg != nullptr){debug_ss << "waiting to receive scan on topic '"<< input_repulsive_scan_topic <<"'"<<std::endl;}
      if(!(attractive_point_msg != nullptr || input_cmd_msg!=nullptr)){
        debug_ss << "waiting to receive point on topic '"<< input_attractive_point_topic <<"' or speed command on topic '"<< input_cmd_topic <<"'"<<std::endl;
        debug_ss << "\n!!! To have the point following working, you have to keep publishing 'true' (std_msgs/Bool Message) on the topic: '"<< input_keep_alive_topic <<"' !!!"<<std::endl;
        debug_ss << "The above is not necessary when using a speed command for assissted teleoperation. But the computation will pause if your 'keep_alive_timeout' value is exceeded on '" << input_cmd_topic << "'." <<std::endl;
      }
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
        get_pos(pos_x, pos_y,angle+vector_map_robot.z,range,0.0,0.0); 
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

  void minus_grad(geometry_msgs::msg::Vector3 &wanted_speed, double pos_x, double pos_y, bool attractive_point_received,bool cmd_msg_received, bool for_robot_cmd, std::stringstream &debug_ss){
    /*Used class variables:
      double max_spd_norm
      double zero_padding_radius
      double min_spd_norm
      double min_goal_dist
      double ka_force 
      double kr_dist
      double kr_height
      attractive_point
      repulsive_points
    */
    /*
    for_robot_cmd: This function is also used to compute the arrows when showing the vector field, so we need to specify if this function is use for that or for generating the robot command.
    Some behavior depend on it. 
    */

    //Compute clamped repulsive command
    geometry_msgs::msg::Vector3 cl_rep_cmd;
    compute_rep_cmd(cl_rep_cmd,pos_x,pos_y);

    //Compute clamped attractivity command
    geometry_msgs::msg::Vector3 cl_attr_cmd;
    bool point_following_mode = attractive_point_received && !cmd_msg_received; //cmd have priority over point following
    compute_attr_cmd(cl_attr_cmd,pos_x,pos_y,point_following_mode);
    //Merge and clamp commands
    geometry_msgs::msg::Vector3 cmd;
    cmd.x = cl_rep_cmd.x + cl_attr_cmd.x;
    cmd.y = cl_rep_cmd.y + cl_attr_cmd.y;
    cmd.z = cl_attr_cmd.z;
    double cmd_norm = sqrt(pow(cmd.x,2)+pow(cmd.y,2));
    double remap_ratio = 0.0;
    if (cmd_norm>0.0){
      remap_ratio = clamp_not_zero(cmd_norm,0.0,max_spd_norm,0.0)/cmd_norm;
    }

    geometry_msgs::msg::Vector3 cl_cmd;
    cl_cmd.x = cmd.x*remap_ratio;
    cl_cmd.y = cmd.y*remap_ratio;
    cl_cmd.z = cmd.z;

    if(point_following_mode){ //we compute stop condition for point following (when goal is reached)
      double pa_dist = sqrt(pow(pos_x-attractive_point.x,2)+pow(pos_y-attractive_point.y,2)); //distance between current position and attractive point
      if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK2.1");}
      double t_now = TimeToDouble(current_global_stamp);
      if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK2.2");}
      double t_last = TimeToDouble(keep_alive_msg_last_stamp);
      if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK2.3");}
      double delay = t_now - t_last;
      bool alive_msg_timed_out = delay>keep_alive_timeout;
      if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK2.4");}
      check_alive();
      if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK2.5");}
      if(alive_msg_timed_out){ //timeout for receiving alive topic
        cl_cmd.x = 0.0;
        cl_cmd.y = 0.0;
        cl_cmd.z = 0.0;
        if(for_robot_cmd){
          attractive_point_msg = nullptr;
          RCLCPP_WARN(this->get_logger(),"The topic %s timed out! Configure and check debug file if this is unexpected",input_keep_alive_topic.c_str());
        }
        debug_ss << "\nTopic timeout! Computation stopped." << std::endl;
        debug_ss << "Last message received on '" << input_keep_alive_topic << "' at " << t_last << "s" << std::endl;
        debug_ss << "Current time: " << t_now << "s" << std::endl;
        debug_ss << "Delay: " << delay << "s" << std::endl;
        debug_ss << "Maximum delay authorized (timeout): " << keep_alive_timeout << "s" << std::endl;
      }
      else if(keep_alive == false){  //received topic for alive is false (user ask to stop point following)
        cl_cmd.x = 0.0;
        cl_cmd.y = 0.0;
        cl_cmd.z = 0.0;
        if(for_robot_cmd){
          attractive_point_msg = nullptr;
          RCLCPP_ERROR(this->get_logger(), "\n!!! To have the point following working, you have to keep publishing 'true' (std_msgs/Bool Message) on the topic: '%s' !!!",input_keep_alive_topic.c_str());
        }
        debug_ss << "\nComputation stopped!." << std::endl;
        debug_ss << "Message on topic '" << input_keep_alive_topic << "' is set to 'false' or Unavailable." << std::endl;
        debug_ss << "\n!!! To have the point following working, you have to keep publishing 'true' (std_msgs/Bool Message) on the topic: '"<< input_keep_alive_topic <<"' !!!"<<std::endl;
      }
      else if(pa_dist <= min_goal_dist){ //goal reached
        cl_cmd.x = 0.0;
        cl_cmd.y = 0.0;
        cl_cmd.z = 0.0;
        debug_ss << "GOAL REACHED:\n"
         << "Distance to Goal " << std::fixed << std::setprecision(5) << pa_dist << "\n"
         << "Distance to consider sucess: " << std::fixed << std::setprecision(5) << min_goal_dist << std::endl;
        if(for_robot_cmd){attractive_point_msg = nullptr;}
      }
      else{
        debug_ss << "GOAL NOT REACHED:\n"
         << "Distance to Goal " << std::fixed << std::setprecision(5) << pa_dist << "\n"
         << "Distance to consider sucess: " << std::fixed << std::setprecision(5) << min_goal_dist << std::endl;
       }
    }
    else{ //we compute stop condition for speed commands following (when message is timed out)
      double t_now = TimeToDouble(current_global_stamp);
      double t_last = TimeToDouble(input_cmd_msg_last_stamp);
      double delay = t_now - t_last;
      bool cmd_msg_timed_out = delay>keep_alive_timeout;
      double cmd_norm = sqrt(pow(cl_attr_cmd.x,2)+pow(cl_attr_cmd.y,2)); //only base on linear speed
      if(cmd_msg_timed_out){
        cl_cmd.x = 0.0;
        cl_cmd.y = 0.0;
        cl_cmd.z = 0.0;
        if(for_robot_cmd){
          input_cmd_msg = nullptr;
          RCLCPP_WARN(this->get_logger(),"The topic %s timed out! Configure and check debug file if this is unexpected",input_cmd_topic.c_str());
        }
        debug_ss << "\nTopic timeout! Computation stopped." << std::endl;
        debug_ss << "Last message received on '" << input_cmd_topic << "' at " << t_last << "s" << std::endl;
        debug_ss << "Current time: " << t_now << "s" << std::endl;
        debug_ss << "Delay: " << delay << "s" << std::endl;
        debug_ss << "Maximum delay authorized (timeout): " << keep_alive_timeout << "s" << std::endl;
      }
      else if(cmd_norm == 0.0){ //we also keep the robot not moving (only linear motion) if the assissted teleoperation speeds are (0,0,0), this avoid the robot to moved by itself when an obstacle approach it.
        cl_cmd.x = 0.0;
        cl_cmd.y = 0.0;
      }
    }

    wanted_speed.x = cl_cmd.x;
    wanted_speed.y = cl_cmd.y;
    wanted_speed.z = cl_cmd.z;

  }

  void compute_attr_cmd(geometry_msgs::msg::Vector3 &cmd_result,double x,double y, bool point_following_mode){
    if(!point_following_mode){ //If we have a speed command to follow (priority over point following)
      double cmd_norm = sqrt(pow(input_cmd.linear.x,2) + pow(input_cmd.linear.y,2));
      if(cmd_norm > zero_padding_radius){ //then we should apply the received CMD rather than doing the point following
        double remap_ratio = clamp_not_zero(cmd_norm,min_spd_norm,max_spd_norm,0.0)/cmd_norm;
        geometry_msgs::msg::Vector3 rob_frame_cmd;
        rob_frame_cmd.x = input_cmd.linear.x*remap_ratio;
        rob_frame_cmd.y = input_cmd.linear.y*remap_ratio;
        rob_frame_cmd.z = input_cmd.angular.z;
        geometry_msgs::msg::Vector3 world_frame_cmd;
        rotate2Dvect(world_frame_cmd,rob_frame_cmd,vector_map_robot.z);
        cmd_result.x = world_frame_cmd.x;
        cmd_result.y = world_frame_cmd.y;
        cmd_result.z = world_frame_cmd.z;
      }
      else{
        cmd_result.x = 0.0;
        cmd_result.y = 0.0;
        cmd_result.z = 0.0;
      }
    }
    else{ 
      //otherwise we do the point following
      double cmd_d_x = ka_force*(attractive_point.x-x);
      double cmd_d_y = ka_force*(attractive_point.y-y);
      double remap_ratio = 0.0;
      double cmd_norm = sqrt(pow(cmd_d_x,2) + pow(cmd_d_y,2));
      if(cmd_norm > 0.0){ //otherwise remap ratio keep default value of 0.0
          remap_ratio = clamp_not_zero(cmd_norm,min_spd_norm,max_spd_norm,0.0001)/cmd_norm;
      }
      cmd_result.x = cmd_d_x*remap_ratio;
      cmd_result.y = cmd_d_y*remap_ratio;
      cmd_result.z = 0.0;
    }
  }

  void compute_rep_cmd(geometry_msgs::msg::Vector3 &cmd_result,double x,double y){
    double minus_grad_v_x = 0.0;
    double minus_grad_v_y = 0.0;
    for(int i=0; i<repulsive_points.size(); i++){
        double norm_term = sqrt(pow(x-repulsive_points[i].x,2) + pow(y-repulsive_points[i].y,2));
        double exp_term = exp(kr_slope*(kr_dist-norm_term));
        if(norm_term > 0.0){
            minus_grad_v_x += kr_height*(x-repulsive_points[i].x)*(exp_term/norm_term);
            minus_grad_v_y += kr_height*(y-repulsive_points[i].y)*(exp_term/norm_term);
        }
    }
    double remap_ratio=0.0;
    double cmd_norm = sqrt(pow(minus_grad_v_x,2) + pow(minus_grad_v_y,2));
    if(cmd_norm>0.0){
      //else it will keep default value of 0.0
      remap_ratio = clamp_not_zero(cmd_norm,0.0,max_spd_norm,0.0)/cmd_norm;
    }
    cmd_result.x = minus_grad_v_x*remap_ratio;
    cmd_result.y = minus_grad_v_y*remap_ratio;
  }

  double clamp_not_zero(double val,double min_lim,double max_lim,double null_radius){ 
    //values should be norms > 0.0
    double new_val=val;
    if (val > max_lim){
        new_val = max_lim;
    }
    else if(val < min_lim && val>null_radius){
        new_val = min_lim;
    }
    else if(val < null_radius){
        new_val = 0.0;
    }
    return new_val;
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

  void publishVectorField(bool attractive_point_received,bool cmd_msg_received)
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
          std::stringstream dummy;
          minus_grad(pos_speed,start_point.x,start_point.y,attractive_point_received,cmd_msg_received,false,dummy);
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

    // create a twist stamped message
    if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK4.1");}
    auto twist_stamped_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK4.1.2");}
    update_stamp();
    if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK4.1.3");}
    twist_stamped_msg->header.stamp = current_global_stamp;
    twist_stamped_msg->header.frame_id = robot_frame;
    twist_stamped_msg->twist = twist_msg;
    twist_stamped_msg->twist.linear.x *= 3.0*arrows_size_multiplier;
    twist_stamped_msg->twist.linear.y *= 3.0*arrows_size_multiplier;
    if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK4.2");}

    // publish the twist stamped message
    cmd_publisher->publish(twist_msg);
    if(flow_debug){RCLCPP_INFO(this->get_logger(), "OK4.3");}
    cmd_vector_publisher->publish(*twist_stamped_msg);
  }

  void initialize_common_params(){
    this->declare_parameter("rate", 20.0);
    this->declare_parameter("input_repulsive_scan_topic", "scan");
    this->declare_parameter("input_attractive_point_topic", "clicked_point");
    this->declare_parameter("input_cmd_topic", "cmd_vel_teleop_joy");
    this->declare_parameter("output_cmd_topic", "cmd_vel_navig");
    this->declare_parameter("output_command_feedback_topic", "cmd_vel_vector");
    this->declare_parameter("input_keep_alive_topic", "vector_field_controller_alive");
    this->declare_parameter("keep_alive_timeout", 1.0);
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("control_type", "omni");
    this->declare_parameter("max_spd_norm", 1.0);
    this->declare_parameter("zero_padding_radius", 0.01);
    this->declare_parameter("min_spd_norm", 0.1);
    this->declare_parameter("min_goal_dist", 0.1);
    this->declare_parameter("ka_force", 3.0);
    this->declare_parameter("kr_dist", 0.8);
    this->declare_parameter("kr_height", 1.0);
    this->declare_parameter("kr_slope", 30.0);
    this->declare_parameter("publish_field", true);
    this->declare_parameter("output_field_topic", "vector_field");
    this->declare_parameter("field_grid_reso_x", 30);
    this->declare_parameter("field_grid_reso_y", 30);
    this->declare_parameter("field_grid_x_elong", 6.0);
    this->declare_parameter("field_grid_y_elong", 6.0);
    this->declare_parameter("arrows_size_multiplier", 0.1);
    this->declare_parameter("debug", true);
    this->declare_parameter("debug_file_path", "/home/jrluser/Desktop/test_workspace/src/ros2-vector-field-controller/ros2_vector_field_controller/ros2_vector_field_controller_debug.txt");
  }

  void refresh_common_params(){
    this->get_parameter("use_sim_time", use_sim_time); //managed by launch file
    this->get_parameter("rate", rate);
    this->get_parameter("input_repulsive_scan_topic", input_repulsive_scan_topic);
    this->get_parameter("input_attractive_point_topic", input_attractive_point_topic);
    this->get_parameter("input_cmd_topic", input_cmd_topic);
    this->get_parameter("output_cmd_topic", output_cmd_topic);
    this->get_parameter("output_command_feedback_topic", output_command_feedback_topic);
    this->get_parameter("input_keep_alive_topic", input_keep_alive_topic);
    this->get_parameter("keep_alive_timeout", keep_alive_timeout);
    this->get_parameter("robot_frame", robot_frame);
    this->get_parameter("map_frame", map_frame);
    this->get_parameter("control_type", control_type);
    this->get_parameter("max_spd_norm", max_spd_norm);
    this->get_parameter("zero_padding_radius", zero_padding_radius);
    this->get_parameter("min_spd_norm", min_spd_norm);
    this->get_parameter("min_goal_dist", min_goal_dist);
    this->get_parameter("ka_force", ka_force);
    this->get_parameter("kr_dist", kr_dist);
    this->get_parameter("kr_height", kr_height);
    this->get_parameter("kr_slope", kr_slope);
    this->get_parameter("publish_field", publish_field);
    this->get_parameter("output_field_topic", output_field_topic);
    this->get_parameter("field_grid_reso_x", field_grid_reso_x);
    this->get_parameter("field_grid_reso_y", field_grid_reso_y);
    this->get_parameter("field_grid_x_elong", field_grid_x_elong);
    this->get_parameter("field_grid_y_elong", field_grid_y_elong);
    this->get_parameter("arrows_size_multiplier", arrows_size_multiplier);
    this->get_parameter("debug", debug);
    this->get_parameter("debug_file_path", debug_file_path);
  }

  void debug_params(){
    std::stringstream debug_ss;
    debug_ss << "\nPARAMETERS:"
            << "\nuse_sim_time: " << use_sim_time
            << "\nrate: " << rate
            << "\ninput_repulsive_scan_topic: " << input_repulsive_scan_topic
            << "\ninput_attractive_point_topic: " << input_attractive_point_topic
            << "\ninput_cmd_topic: " << input_cmd_topic
            << "\noutput_cmd_topic: " << output_cmd_topic
            << "\noutput_command_feedback_topic: " << output_command_feedback_topic
            << "\ninput_keep_alive_topic: " << input_keep_alive_topic
            << "\nkeep_alive_timeout: " << keep_alive_timeout
            << "\nrobot_frame: " << robot_frame
            << "\nmap_frame: " << map_frame
            << "\ncontrol_type: " << control_type
            << "\nmax_spd_norm: " << max_spd_norm
            << "\nzero_padding_radius: " << zero_padding_radius
            << "\nmin_spd_norm: " << min_spd_norm
            << "\nmin_goal_dist: " << min_goal_dist
            << "\nka_force: " << ka_force
            << "\nkr_dist: " << kr_dist
            << "\nkr_height: " << kr_height
            << "\nkr_slope: " << kr_slope
            << "\npublish_field: " << publish_field
            << "\noutput_field_topic: " << output_field_topic
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
  std::string input_repulsive_scan_topic; //used
  std::string input_attractive_point_topic; //used
  std::string input_cmd_topic; //used
  std::string output_cmd_topic; //used
  std::string output_command_feedback_topic; //used
  std::string input_keep_alive_topic; //used
  double keep_alive_timeout; //used
  std::string robot_frame; //used
  std::string map_frame; //used
  std::string control_type;
  double max_spd_norm; //used
  double zero_padding_radius; //used
  double min_spd_norm; //used
  double min_goal_dist; //used
  double ka_force; //used
  double kr_dist; //used
  double kr_height; //used
  double kr_slope; //used
  bool publish_field; //used
  std::string output_field_topic; //used
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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vector_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vector_field_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  //store received data
  sensor_msgs::msg::LaserScan::SharedPtr repulsive_scan_msg;
  geometry_msgs::msg::PointStamped::SharedPtr attractive_point_msg; //not directly used, we use directly 'attractive_point'
  geometry_msgs::msg::Twist::SharedPtr input_cmd_msg;
  std_msgs::msg::Bool::SharedPtr keep_alive_msg;

  //store data to compute
  std::vector<geometry_msgs::msg::Point> repulsive_points;
  geometry_msgs::msg::Point attractive_point;
  geometry_msgs::msg::Twist input_cmd;
  double init_last_times = 0.0;
  builtin_interfaces::msg::Time input_cmd_msg_last_stamp = DoubleToTime(init_last_times);
  geometry_msgs::msg::Vector3 vector_map_robot; //(x,y,heading) offsets
  bool keep_alive = false; //Used to pause or start the computing
  builtin_interfaces::msg::Time keep_alive_msg_last_stamp = DoubleToTime(init_last_times);

  //concurrence
  std::mutex mutex_debug_file;
  std::mutex attractive_point_mutex;
  std::mutex input_cmd_mutex;
  std::mutex raw_scan_mutex;
  std::mutex keep_alive_mutex;

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