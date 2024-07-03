/*
LAST MODIF(DD/MM/YYYY): 17/06/2024
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <fstream>
#include <iostream> 
#include "rosgraph_msgs/msg/clock.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <cmath>
#include <Eigen/Dense>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/odometry.hpp"

nav_msgs::msg::Odometry extrapolate_odometry(nav_msgs::msg::Odometry &prev_prev_odom, nav_msgs::msg::Odometry &prev_odom, double time_to_add);
nav_msgs::msg::Odometry interpolate_odometry(nav_msgs::msg::Odometry &odom1, nav_msgs::msg::Odometry &odom2, double ratio, bool extrapolate);
nav_msgs::msg::Odometry create_zero_odometry();
double odom_to_heading(nav_msgs::msg::Odometry &odom);
nav_msgs::msg::Odometry get_estimated_odom(double target_time, std::vector<nav_msgs::msg::Odometry> &odoms_list, bool extrapolate, bool log, std::stringstream &debug_ss);
void sav_odom(std::vector<nav_msgs::msg::Odometry> &odoms_list, nav_msgs::msg::Odometry odom, double current_time, double odom_delay_limit);

bool stringToBool(std::string &str);
int transform_opened_scan(sensor_msgs::msg::LaserScan::SharedPtr to_transform_scan, double off_vect_x,double off_vect_y, double off_tetha, std::stringstream &debug_ss);
double rads_to_degrees(double rads);
double degrees_to_rads(double degrees);
std::string print_tf(geometry_msgs::msg::TransformStamped transform);
int copy_ranges(sensor_msgs::msg::LaserScan::SharedPtr host_scan, sensor_msgs::msg::LaserScan::SharedPtr target_scan);
//geometry_msgs::msg::TransformStamped tf_offset(geometry_msgs::msg::TransformStamped &relative_transform, geometry_msgs::msg::TransformStamped tf1, geometry_msgs::msg::TransformStamped tf2);
//void shift_cloud_from_tf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::msg::TransformStamped transf);
void MatProd_fast4_4(double (&M)[4][4],double (&A)[4][4],double (&B)[4][4]);
void MatProd_fast3_3(double (&M)[3][3],double (&A)[3][3],double (&B)[3][3]);
void MatProd_fast4_Vect(double (&M)[4][1],double (&A)[4][4],double (&B)[4][1]);
void to_identity3_3(double (&mat)[3][3]);
void to_identity4_4(double (&mat)[4][4]);
double TimeToDouble(builtin_interfaces::msg::Time& stamp);
builtin_interfaces::msg::Time DoubleToTime(double& seconds);
bool consider_val(int current_ind, int start_ind, int end_ind);
int angle_to_index(double alpha, int resolution);
void remap_scan(sensor_msgs::msg::LaserScan::SharedPtr input_scan, sensor_msgs::msg::LaserScan::SharedPtr output_scan, std::vector<double> &mask_to_update);
int remap_scan_index(int prev_ind, double prev_angle_start, double prev_angle_end, double prev_reso, double new_angle_start, double new_angle_end, double new_reso);
double sawtooth(double x);
geometry_msgs::msg::Vector3 quaternion_to_euler3D(geometry_msgs::msg::Quaternion quat);
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> get_4Dmatrix_from_transform(geometry_msgs::msg::TransformStamped transf);
void get_4Dmatrix_from_transform_fast(double (&M)[4][4],geometry_msgs::msg::TransformStamped transf);
Eigen::MatrixXd rot_matrix_from_euler(geometry_msgs::msg::Vector3 euler_angles);
void rot_matrix_from_euler_fast(double (&R)[3][3], geometry_msgs::msg::Vector3 euler_angles);
double scalar_projection(const Eigen::VectorXd& a, const Eigen::VectorXd& b);
double scalar_projection_fast(double (&a)[3], double (&b)[3]);

std::string vector3ToString(geometry_msgs::msg::Vector3& vector);
geometry_msgs::msg::Vector3 stringToVector3(std::string& str);
geometry_msgs::msg::Vector3 adapt_angle(geometry_msgs::msg::Vector3 vect);
double index_to_angle(int ind, int resolution, double elongation = 2 * M_PI);

int filter_360_data(sensor_msgs::msg::LaserScan::SharedPtr to_transform_scan, std::vector<double> &mask_to_update ,double start_angle, double end_angle, double angle_origin_offset, double min_range, double max_range, std::stringstream &debug_ss);
void get_pos(double &x, double &y,double alpha,double val,double x_off,double y_off);
int transform_360_data(sensor_msgs::msg::LaserScan::SharedPtr to_transform_scan, double off_vect_x,double off_vect_y, double off_tetha, std::stringstream &debug_ss);
void transform_2D_point(double &result_x, double &result_y,double init_x, double init_y, Eigen::MatrixXd &M1_2);