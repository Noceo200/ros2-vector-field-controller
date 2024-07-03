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
#include "tools.h"
#include "nav_msgs/msg/odometry.hpp"

nav_msgs::msg::Odometry extrapolate_odometry(nav_msgs::msg::Odometry &prev_prev_odom, nav_msgs::msg::Odometry &prev_odom, double time_to_add) {
    nav_msgs::msg::Odometry extrapolated_odom;

    // Calculate the time difference between the previous odometry messages
    double dt = TimeToDouble(prev_odom.header.stamp) - TimeToDouble(prev_prev_odom.header.stamp);

    // Ensure the time difference is positive
    if (dt <= 0) {
        throw std::runtime_error("Extrapolation of odometry: Invalid time difference between odometry messages, prev_odom stamp should be > prev_prev_odom stamp");
    }

    // Calculate linear velocities
    double vx = (prev_odom.pose.pose.position.x - prev_prev_odom.pose.pose.position.x) / dt;
    double vy = (prev_odom.pose.pose.position.y - prev_prev_odom.pose.pose.position.y) / dt;
    double vz = (prev_odom.pose.pose.position.z - prev_prev_odom.pose.pose.position.z) / dt;

    // Extrapolate position
    extrapolated_odom.pose.pose.position.x = prev_odom.pose.pose.position.x + vx * time_to_add;
    extrapolated_odom.pose.pose.position.y = prev_odom.pose.pose.position.y + vy * time_to_add;
    extrapolated_odom.pose.pose.position.z = prev_odom.pose.pose.position.z + vz * time_to_add;

    // Calculate angular velocities (quaternion differentiation)
    auto q1 = prev_prev_odom.pose.pose.orientation;
    auto q2 = prev_odom.pose.pose.orientation;

    double dq_w = (q2.w - q1.w) / dt;
    double dq_x = (q2.x - q1.x) / dt;
    double dq_y = (q2.y - q1.y) / dt;
    double dq_z = (q2.z - q1.z) / dt;

    // Extrapolate orientation
    extrapolated_odom.pose.pose.orientation.w = prev_odom.pose.pose.orientation.w + dq_w * time_to_add;
    extrapolated_odom.pose.pose.orientation.x = prev_odom.pose.pose.orientation.x + dq_x * time_to_add;
    extrapolated_odom.pose.pose.orientation.y = prev_odom.pose.pose.orientation.y + dq_y * time_to_add;
    extrapolated_odom.pose.pose.orientation.z = prev_odom.pose.pose.orientation.z + dq_z * time_to_add;

    // Normalize the quaternion
    double norm = sqrt(
        extrapolated_odom.pose.pose.orientation.w * extrapolated_odom.pose.pose.orientation.w +
        extrapolated_odom.pose.pose.orientation.x * extrapolated_odom.pose.pose.orientation.x +
        extrapolated_odom.pose.pose.orientation.y * extrapolated_odom.pose.pose.orientation.y +
        extrapolated_odom.pose.pose.orientation.z * extrapolated_odom.pose.pose.orientation.z
    );

    extrapolated_odom.pose.pose.orientation.w /= norm;
    extrapolated_odom.pose.pose.orientation.x /= norm;
    extrapolated_odom.pose.pose.orientation.y /= norm;
    extrapolated_odom.pose.pose.orientation.z /= norm;

    return extrapolated_odom;
}

nav_msgs::msg::Odometry interpolate_odometry(nav_msgs::msg::Odometry &odom1, nav_msgs::msg::Odometry &odom2, double ratio) {
    nav_msgs::msg::Odometry interpolated_odom;

    // Perform linear interpolation for position
    interpolated_odom.pose.pose.position.x = odom1.pose.pose.position.x * (1 - ratio) + odom2.pose.pose.position.x * ratio;
    interpolated_odom.pose.pose.position.y = odom1.pose.pose.position.y * (1 - ratio) + odom2.pose.pose.position.y * ratio;
    interpolated_odom.pose.pose.position.z = odom1.pose.pose.position.z * (1 - ratio) + odom2.pose.pose.position.z * ratio;

    // Slerp for quaternion interpolation
    auto quat1 = odom1.pose.pose.orientation;
    auto quat2 = odom2.pose.pose.orientation;
    double dot_product = quat1.w * quat2.w + quat1.x * quat2.x + quat1.y * quat2.y + quat1.z * quat2.z;
    double omega = acos(std::min(1.0, std::abs(dot_product)));

    if (std::abs(omega) < 0.001) {
        // If quaternions are very close, use linear interpolation
        interpolated_odom.pose.pose.orientation.x = quat1.x * (1 - ratio) + quat2.x * ratio;
        interpolated_odom.pose.pose.orientation.y = quat1.y * (1 - ratio) + quat2.y * ratio;
        interpolated_odom.pose.pose.orientation.z = quat1.z * (1 - ratio) + quat2.z * ratio;
        interpolated_odom.pose.pose.orientation.w = quat1.w * (1 - ratio) + quat2.w * ratio;
    } else {
        // Use Slerp for interpolation
        double sin_omega = sin(omega);
        double inv_sin_omega = 1 / sin_omega;
        double scale1 = sin((1 - ratio) * omega) * inv_sin_omega;
        double scale2 = sin(ratio * omega) * inv_sin_omega;
        interpolated_odom.pose.pose.orientation.x = (scale1 * quat1.x + scale2 * quat2.x);
        interpolated_odom.pose.pose.orientation.y = (scale1 * quat1.y + scale2 * quat2.y);
        interpolated_odom.pose.pose.orientation.z = (scale1 * quat1.z + scale2 * quat2.z);
        interpolated_odom.pose.pose.orientation.w = (scale1 * quat1.w + scale2 * quat2.w);
    }

    return interpolated_odom;
}

nav_msgs::msg::Odometry create_zero_odometry() {
    nav_msgs::msg::Odometry zero_odom;
    zero_odom.pose.pose.position.x = 0.0;
    zero_odom.pose.pose.position.y = 0.0;
    zero_odom.pose.pose.position.z = 0.0;
    zero_odom.pose.pose.orientation.x = 0.0;
    zero_odom.pose.pose.orientation.y = 0.0;
    zero_odom.pose.pose.orientation.z = 0.0;
    zero_odom.pose.pose.orientation.w = 1.0;
    zero_odom.twist.twist.linear.x = 0.0;
    zero_odom.twist.twist.linear.y = 0.0;
    zero_odom.twist.twist.linear.z = 0.0;
    zero_odom.twist.twist.angular.x = 0.0;
    zero_odom.twist.twist.angular.y = 0.0;
    zero_odom.twist.twist.angular.z = 0.0;
    return zero_odom;
}

double odom_to_heading(nav_msgs::msg::Odometry &odom) {
    auto euler = quaternion_to_euler3D(odom.pose.pose.orientation);
    return adapt_angle(euler).z;
}

nav_msgs::msg::Odometry get_estimated_odom(double target_time, std::vector<nav_msgs::msg::Odometry> &odoms_list,  bool extrapolate, bool log, std::stringstream &debug_ss) {
    nav_msgs::msg::Odometry estimated_odom = create_zero_odometry();
    int nb_odoms = odoms_list.size();

    if (log) {debug_ss << "------------------ODOM detail------------------";}

    if (odoms_list.empty()) {
        if (log) {
            debug_ss << "\nList of odometry is empty. Using zero_odometry until messages received." << std::endl;
        }
        return estimated_odom;
    }

    if (target_time <= TimeToDouble(odoms_list.front().header.stamp)) {
        if (log) {
            debug_ss << "\nTarget time is before or equal the oldest recorded odometry. Returning the oldest odometry."
                     << "\nTarget time: " << target_time << " s"
                     << "\n(x,y,tetha,stamp): (" << odoms_list.front().pose.pose.position.x << ","
                     << odoms_list.front().pose.pose.position.y << ","
                     << odom_to_heading(odoms_list.front()) << ","
                     << TimeToDouble(odoms_list.front().header.stamp) << ")"<< std::endl;
        }
        return odoms_list.front();
    }

    if (target_time >= TimeToDouble(odoms_list.back().header.stamp)) {
        if(extrapolate){
            if(nb_odoms>1){
                //extrapolation
                auto prev_odom = odoms_list[nb_odoms-1];
                auto prev_prev_odom = odoms_list[nb_odoms-2];
                double prev_time = TimeToDouble(prev_odom.header.stamp);
                double time_to_extrapolate = target_time-prev_time;
                estimated_odom = extrapolate_odometry(prev_prev_odom, prev_odom, time_to_extrapolate);
                estimated_odom.header.stamp = DoubleToTime(target_time);
                if (log) {
                    debug_ss << "\nTarget time is after or equal the newest recorded odometry. Extrapolation."
                    << "\nTarget time: " << target_time << " s"
                    << "\n(x,y,tetha,stamp) last-1 : (" << prev_prev_odom.pose.pose.position.x << ","
                    << prev_prev_odom.pose.pose.position.y << ","
                    << odom_to_heading(prev_prev_odom) << ","
                    << TimeToDouble(prev_prev_odom.header.stamp) << ")"
                    << "\n(x,y,tetha,stamp) last : (" << prev_odom.pose.pose.position.x << ","
                    << prev_odom.pose.pose.position.y << ","
                    << odom_to_heading(prev_odom) << ","
                    << prev_time << ")"
                    << "\n(x,y,tetha,stamp) extra : (" << estimated_odom.pose.pose.position.x << ","
                    << estimated_odom.pose.pose.position.y << ","
                    << odom_to_heading(estimated_odom) << ","
                    << TimeToDouble(estimated_odom.header.stamp) << ")" << std::endl;
                }
                return estimated_odom;
            }
            else{
                if (log) {debug_ss << "\nTarget time is after or equal the newest recorded odometry. Extrapolation ON but not enough odometry values (<2): Returning the newest odometry.";}
            }

        }
        else{
            if (log) {debug_ss << "\nTarget time is after or equal the newest recorded odometry. Extrapolation OFF: Returning the newest odometry.";}
        }
        if (log) { 
            debug_ss << "\nTarget time: " << target_time << " s"
                    << "\n(x,y,tetha,stamp): (" << odoms_list.back().pose.pose.position.x << ","
                    << odoms_list.back().pose.pose.position.y << ","
                    << odom_to_heading(odoms_list.back()) << ","
                    << TimeToDouble(odoms_list.back().header.stamp) << ")"<< std::endl;
        }
        return odoms_list.back(); //return that in any case if no extrapolation or not enough odometry values  

    }

    for (size_t i = 1; i < nb_odoms; ++i) {
        if (target_time < TimeToDouble(odoms_list[i].header.stamp)) {
            auto prev_odom = odoms_list[i - 1];
            auto next_odom = odoms_list[i];
            double prev_time = TimeToDouble(prev_odom.header.stamp);
            double next_time = TimeToDouble(next_odom.header.stamp);
            double ratio = (target_time - prev_time) / (next_time - prev_time);
            estimated_odom = interpolate_odometry(prev_odom, next_odom, ratio);
            estimated_odom.header.stamp = DoubleToTime(target_time);
            if (log) {
                debug_ss << "\nTarget time is between odometries " << i - 1 << " and " << i << " on " << nb_odoms - 1 << ", interpolating."
                         << "\nTarget time: " << target_time << " s"
                         << "\n(x,y,tetha,stamp) prev : (" << prev_odom.pose.pose.position.x << ","
                         << prev_odom.pose.pose.position.y << ","
                         << odom_to_heading(prev_odom) << ","
                         << TimeToDouble(prev_odom.header.stamp) << ")"
                         << "\n(x,y,tetha,stamp) next : (" << next_odom.pose.pose.position.x << ","
                         << next_odom.pose.pose.position.y << ","
                         << odom_to_heading(next_odom) << ","
                         << TimeToDouble(next_odom.header.stamp) << ")"
                         << "\n(x,y,tetha,stamp) prev : (" << estimated_odom.pose.pose.position.x << ","
                         << estimated_odom.pose.pose.position.y << ","
                         << odom_to_heading(estimated_odom) << ","
                         << TimeToDouble(estimated_odom.header.stamp) << ")" << std::endl;
            }
            return estimated_odom;
        }
    }
}

void sav_odom(std::vector<nav_msgs::msg::Odometry> &odoms_list, nav_msgs::msg::Odometry odom, double current_time, double odom_delay_limit) {
    //sav odom on odoms_list and clean the list according to current_time and odom_delay_limit
    //we place the received odometry in the list considering its timestamp
    size_t nb_odoms = odoms_list.size();
    if (nb_odoms == 0) {
        odoms_list.push_back(odom);
        return;
    }

    int i = nb_odoms;
    while (i > 0 && TimeToDouble(odom.header.stamp) < TimeToDouble(odoms_list[i - 1].header.stamp)) {
        i--;
    }

    if (i == nb_odoms) {
        odoms_list.push_back(odom);
    } else {
        odoms_list.insert(odoms_list.begin() + i, odom);
    }

    // We delete outdated odometries
    nb_odoms = odoms_list.size(); // changed because we may have added an odometry
    i = 0;
    double outdated_min_time = current_time - odom_delay_limit;
    while (i < nb_odoms && TimeToDouble(odoms_list[i].header.stamp) < outdated_min_time) {
        i++;
    }

    if (i == 0) {
        // nothing to delete
    } else if (i == nb_odoms) {
        // they are all outdated, we just keep the last to still have an odometry
        odoms_list.erase(odoms_list.begin(), odoms_list.end() - 1); // we delete all but the last
    } else {
        // we found an index from which we should delete all odometry before
        odoms_list.erase(odoms_list.begin(), odoms_list.begin() + i);
    }

}



bool stringToBool(std::string &str) {
    // Convert string to lowercase for case-insensitive comparison
    std::string lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(), ::tolower);

    // Check for various representations of true and false
    if (lowerStr == "true" || lowerStr == "yes" || lowerStr == "1") {
        return true;
    } else{
        return false;
    }
}

int transform_opened_scan(sensor_msgs::msg::LaserScan::SharedPtr to_transform_scan, double off_vect_x,double off_vect_y, double off_tetha, std::stringstream &debug_ss){
    /*
    This function transform an 'opened' scan wich mean that the values will not 'slid' on a circle frame, but will be cut instead.
    to_transform_scan: current message to transform
    off_vect_x: vector new_origin=>lidar
    off_vect_y: vector new_origin=>lidar
    return transformed_scan: message in which datas will be stored
    Homogoneous transformations: https://www.fil.univ-lille.fr/portail/archive21-22/~aubert/m3d/m3d_transformation.pdf
    */
    //RCLCPP_INFO(this->get_logger(), "Transform_data: params : offx='%f'; offy='%f'; off_tetha='%f'",off_vect_x,off_vect_y,off_tetha);
    //RCLCPP_INFO(this->get_logger(), "transform 1");
    sensor_msgs::msg::LaserScan::SharedPtr transformed_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    transformed_scan->ranges.clear();
    int resolution = to_transform_scan->ranges.size();

    // Transform the LaserScan message
    for (int i = 0; i < resolution; ++i) { //initiate
        transformed_scan->ranges.push_back(INFINITY);
    }

    //let's be sure to manipulate angles from 0 to 2pi
    if (off_tetha<0){
        off_tetha=2*M_PI+off_tetha;
    }
    if (off_tetha == 2*M_PI){
        off_tetha=0;
    }
    //translation in homogeneous coordinates
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T(0,3) = off_vect_x;
    T(1,3) = off_vect_y;
    //rotation in homogeneous coordinates
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    R(0,0) = cos(off_tetha);
    R(0,1) = -sin(off_tetha);
    R(1,0) = sin(off_tetha);
    R(1,1) = cos(off_tetha);
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to sensor_frame
    Eigen::MatrixXd M1_2(4, 4);
    M1_2 = T*R;

    //debug_ss << "DEBUG: Pass Matrix:" << std::endl;
    //debug_ss << M1_2 << std::endl;

    //loop variables
    double init_x = 0;
    double init_y = 0;
    double init_val = 0;
    double init_angle = 0;
    double new_x = 0;
    double new_y = 0;
    double new_val = 0;
    double new_angle = 0;
    int new_index = 0;
    double max_angle = to_transform_scan->angle_max;
    double min_angle = to_transform_scan->angle_min;
    double elong = max_angle-min_angle;
    double angle_incr = elong/resolution; //can also be obtain with laserscan message

    for (int ind = 0; ind < resolution; ++ind) { //transform
        init_angle = index_to_angle(ind,resolution,elong); //should be between 0.0 and elongation, but will not coresspond to real angle because of min_angle offset
        init_val = to_transform_scan->ranges[ind];

        //points transformation
        if (!isinf(init_val)){ //if it is infinity, we don't need to compute as the default values in the new scan will be INFINITY
            get_pos(init_x, init_y,init_angle,init_val,0.0,0.0); //position in inital frame, vector: lidar=>point
            Eigen::MatrixXd X2(4, 1); //pos in sensor frame
            X2(0,0) = init_x;
            X2(1,0) = init_y;
            X2(2,0) = 0.0;
            X2(3,0) = 1.0;
            Eigen::MatrixXd X1(4, 1); //pos in newframe
            X1 = M1_2*X2;
            //Unpack
            double new_x2 = X1(0,0);
            double new_y2 = X1(1,0);
            new_angle = atan2(new_y2,new_x2);
            //we want angles from 0 to 2pi
            if (new_angle<0){
                new_angle=2*M_PI+new_angle;
            }
            if (new_angle == 2*M_PI){
                new_angle=0;
            }
            //RCLCPP_INFO(this->get_logger(), "Transform_data: newangle='%f'",new_angle);

            //new norm
            new_val = sqrt(pow(new_x2,2)+pow(new_y2,2));
            //new angle is between 0.0 and 2Pi so it can be outside our initial list
            new_index = angle_to_index(new_angle,static_cast<int>(round(2*M_PI/angle_incr))); //we want index as it would be if it was a 360 closed scan
            //and we crop the indexes outside the initial list
            if(new_index < resolution){
                transformed_scan->ranges[new_index] = std::min(static_cast<double>(transformed_scan->ranges[new_index]),new_val); //in case some area are no more accessible from the new origin
                //debug_ss << "Transform_data: initindex='" << ind << "'; initangle='" << init_angle << "'; initval='" << init_val << "'; initx='" << init_x << "'; inity='" << init_y << "'; newx2='" << new_x2 << "'; newy2='" << new_y2 << "'; newval='" << new_val << "'; newangle='" << new_angle << "'; newindex='" << new_index << "'";
            }
        }

    }
    //RCLCPP_INFO(this->get_logger(), "transform 3");

    //copy result ranges into to_transform_scan
    if(copy_ranges(to_transform_scan,transformed_scan)){ 
        return 1;
    } 
    else{
        debug_ss << "ERROR: filter_360_data: " << "Copy ranges failed" << '\n';
        return 0;
    }
}

double rads_to_degrees(double rads){
    return rads*180/M_PI;
}

double degrees_to_rads(double degrees){
    return degrees*M_PI/180;
}

int filter_360_data(sensor_msgs::msg::LaserScan::SharedPtr to_transform_scan, std::vector<double> &mask_to_update ,double start_angle, double end_angle, double angle_origin_offset, double min_range, double max_range, std::stringstream &debug_ss){
    /*
    Receive a to_transform_scan of a 360 scan only, the shift will not work if the scan message is not a 360 one.
    */    

    int resolution = to_transform_scan->ranges.size();
    //we shift all the values to bring back the origin angle to the x axis
    int index_shift = angle_to_index(angle_origin_offset,resolution); //index which is the first angle for the lidar
    sensor_msgs::msg::LaserScan::SharedPtr shifted_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    std::vector<double> shifted_mask;
    shifted_scan->ranges.clear();
    for (int i = 0; i < resolution; ++i) { 
        if(i+index_shift < resolution){ 
            shifted_scan->ranges.push_back(to_transform_scan->ranges[i+index_shift]);
            shifted_scan->intensities.push_back(to_transform_scan->intensities[i+index_shift]);
            shifted_mask.push_back(mask_to_update[i+index_shift]);
        }
        else{
            shifted_scan->ranges.push_back(to_transform_scan->ranges[i+index_shift-resolution]);
            shifted_scan->intensities.push_back(to_transform_scan->intensities[i+index_shift-resolution]);
            shifted_mask.push_back(mask_to_update[i+index_shift-resolution]);
        }
    }

    //we keep the wanted angles
    sensor_msgs::msg::LaserScan::SharedPtr filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    std::vector<double> filtered_mask;
    filtered_scan->ranges.clear();

    //compute index to consider according to min/max angles wanted
    int start_index = angle_to_index(start_angle, resolution);
    int end_index = angle_to_index(end_angle, resolution);

    // LaserScan message filling
    for (int i = 0; i < resolution; ++i) { 
        float dist = shifted_scan->ranges[i];
        if(consider_val(i, start_index, end_index) && dist<=max_range && dist>=min_range){ //if the value is autorized, we add it
            filtered_scan->ranges.push_back(shifted_scan->ranges[i]);
            filtered_scan->intensities.push_back(shifted_scan->intensities[i]);
        }
        else{
            filtered_scan->ranges.push_back(INFINITY);
            filtered_scan->intensities.push_back(0.0);
        }
        //mask generation
        if(consider_val(i, start_index, end_index)){
            filtered_mask.push_back(shifted_mask[i]);
        }
        else{
            filtered_mask.push_back(0.0);
        }
    }

    //copy result ranges into to_transform_scan
    //mask
    for(int i = 0; i < resolution; ++i){
        mask_to_update[i] = filtered_mask[i];
    }
    //laser_scan
    if(copy_ranges(to_transform_scan,filtered_scan)){ 
        return 1;
    } 
    else{
        debug_ss << "ERROR: filter_360_data: " << "Copy ranges failed" << '\n';
        return 0;
    }
}

void get_pos(double &x, double &y,double alpha,double val,double x_off,double y_off){
    x = val*cos(alpha)+x_off;
    y = val*sin(alpha)+y_off;
}

int transform_360_data(sensor_msgs::msg::LaserScan::SharedPtr to_transform_scan, double off_vect_x,double off_vect_y, double off_tetha, std::stringstream &debug_ss){
    /*
    to_transform_scan: current message to transform
    off_vect_x: vector new_origin=>lidar
    off_vect_y: vector new_origin=>lidar
    return transformed_scan: message in which datas will be stored
    Homogoneous transformations: https://www.fil.univ-lille.fr/portail/archive21-22/~aubert/m3d/m3d_transformation.pdf
    */
    //RCLCPP_INFO(this->get_logger(), "Transform_data: params : offx='%f'; offy='%f'; off_tetha='%f'",off_vect_x,off_vect_y,off_tetha);
    //RCLCPP_INFO(this->get_logger(), "transform 1");
    sensor_msgs::msg::LaserScan::SharedPtr transformed_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    transformed_scan->ranges.clear();
    int resolution = to_transform_scan->ranges.size();

    // Transform the LaserScan message
    for (int i = 0; i < resolution; ++i) { //initiate
        transformed_scan->ranges.push_back(INFINITY);
        transformed_scan->intensities.push_back(0.0);
    }

    //let's be sure to manipulate angles from 0 to 2pi
    if (off_tetha<0){
        off_tetha=2*M_PI+off_tetha;
    }
    if (off_tetha == 2*M_PI){
        off_tetha=0;
    }
    //translation in homogeneous coordinates
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T(0,3) = off_vect_x;
    T(1,3) = off_vect_y;
    //rotation in homogeneous coordinates
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    R(0,0) = cos(off_tetha);
    R(0,1) = -sin(off_tetha);
    R(1,0) = sin(off_tetha);
    R(1,1) = cos(off_tetha);
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to sensor_frame
    Eigen::MatrixXd M1_2(4, 4);
    M1_2 = T*R;

    //debug_ss << "DEBUG: Pass Matrix:" << std::endl;
    //debug_ss << M1_2 << std::endl;

    //loop variables
    double init_x = 0;
    double init_y = 0;
    double init_val = 0;
    double init_angle = 0;
    double new_x = 0;
    double new_y = 0;
    double new_val = 0;
    double new_angle = 0;
    int new_index = 0;

    for (int ind = 0; ind < resolution; ++ind) { //transform
        init_angle = index_to_angle(ind,resolution);
        init_val = to_transform_scan->ranges[ind];

        //points transformation
        if (!isinf(init_val)){ //if it is infinity, we don't need to compute as the default values in the new scan will be INFINITY
            get_pos(init_x, init_y,init_angle,init_val,0.0,0.0); //position in inital frame, vector: lidar=>point
            transform_2D_point(new_x, new_y,init_x,init_y,M1_2);
            new_angle = atan2(new_y,new_x);
            //we want angles from 0 to 2pi
            if (new_angle<0){
                new_angle=2*M_PI+new_angle;
            }
            if (new_angle == 2*M_PI){
                new_angle=0;
            }
            //RCLCPP_INFO(this->get_logger(), "Transform_data: newangle='%f'",new_angle);

            //new norm
            new_val = sqrt(pow(new_x,2)+pow(new_y,2));

            new_index = angle_to_index(new_angle,resolution);
            if(new_val < transformed_scan->ranges[new_index]){
                transformed_scan->ranges[new_index] = new_val;
                transformed_scan->intensities[new_index] = to_transform_scan->intensities[ind];
            }
            //(FORMER CODE)transformed_scan->ranges[new_index] = std::min(static_cast<double>(transformed_scan->ranges[new_index]),new_val); //depending on resolution several points can try to be at same index, and in case some area are no more accessible from the new origin
            //RCLCPP_INFO(this->get_logger(), "Transform_data: initindex='%d'; initangle='%f'; initval='%f'; initx='%f'; inity='%f'; newx2='%f'; newy2='%f'; newval='%f'; newangle='%f'; newindex='%f'",ind,init_angle,init_val,init_x,init_y,new_x2,new_y2,new_val,new_angle,new_index);
        }

    }
    //RCLCPP_INFO(this->get_logger(), "transform 3");

    //copy result ranges into to_transform_scan
    if(copy_ranges(to_transform_scan,transformed_scan)){ 
        return 1;
    } 
    else{
        debug_ss << "ERROR: filter_360_data: " << "Copy ranges failed" << '\n';
        return 0;
    }
}

void transform_2D_point(double &result_x, double &result_y,double init_x, double init_y, Eigen::MatrixXd &M1_2){
    Eigen::MatrixXd X2(4, 1); //pos in sensor frame
    X2(0,0) = init_x;
    X2(1,0) = init_y;
    X2(2,0) = 0.0;
    X2(3,0) = 1.0;
    Eigen::MatrixXd X1(4, 1); //pos in newframe
    X1 = M1_2*X2;
    //Unpack
    result_x = X1(0,0);
    result_y = X1(1,0);
}

int copy_ranges(sensor_msgs::msg::LaserScan::SharedPtr host_scan, sensor_msgs::msg::LaserScan::SharedPtr target_scan){
    try
    {
        int reso = host_scan->ranges.size();
        for (int i = 0; i < reso; ++i) { 
            host_scan->ranges[i] = target_scan->ranges[i];
            host_scan->intensities[i] = target_scan->intensities[i];
        }
        return 1;
    }
    catch(const std::exception& e)
    {
        return 0;
    }
}

geometry_msgs::msg::Vector3 adapt_angle(geometry_msgs::msg::Vector3 vect){
    //avoid flip with euleur angles at gimbla lock, work only for z component in that case.
    geometry_msgs::msg::Vector3 new_vect;
    if(abs(abs(vect.x)-M_PI)<0.01 && abs(abs(vect.y)-M_PI)<0.01){
        new_vect.x = 0.0;
        new_vect.y = 0.0;
        new_vect.z = M_PI+vect.z; 
    }
    else{
        new_vect.x = vect.x;
        new_vect.y = vect.y;
        new_vect.z = vect.z;
    }
    return new_vect;
}

std::string vector3ToString(geometry_msgs::msg::Vector3& vector) {
    std::string result = "(" +
        std::to_string(vector.x) + ", " +
        std::to_string(vector.y) + ", " +
        std::to_string(vector.z) + ")";
    return result;
}

geometry_msgs::msg::Vector3 stringToVector3(std::string& str) {
    geometry_msgs::msg::Vector3 vector;
    std::stringstream ss(str);
    // Temporary variables to store parsed values
    double x, y, z;
    // Read values from stringstream
    char delim;
    ss >> delim >> x >> delim >> y >> delim >> z >> delim;
    // Assign parsed values to the vector
    vector.x = x;
    vector.y = y;
    vector.z = z;
    return vector;
}

std::string print_tf(geometry_msgs::msg::TransformStamped transform){
    std::string debug_str;
    std::stringstream debug_ss;
    geometry_msgs::msg::Vector3 euleur_angles = quaternion_to_euler3D(transform.transform.rotation);
    geometry_msgs::msg::Vector3 euleur_angles_head = adapt_angle(euleur_angles);
    debug_ss << "Transform '" << transform.header.frame_id << "' --> '" << transform.child_frame_id
             << "\n    Translation: (" << transform.transform.translation.x << "," << transform.transform.translation.y << "," << transform.transform.translation.z << ") m"
             << "\n    Quaternion: (" << transform.transform.rotation.x << "," << transform.transform.rotation.y << "," << transform.transform.rotation.z <<  "," << transform.transform.rotation.w << ")" 
             << "\n    Euler: (" << rads_to_degrees(euleur_angles.x) << "," << rads_to_degrees(euleur_angles.y) << "," << rads_to_degrees(euleur_angles.z) << ") degs"
             << "\n    Heading_only: " << rads_to_degrees(euleur_angles_head.z)
             << std::endl;
    
    debug_str = debug_ss.str();
    return debug_str;
}


void MatProd_fast4_4(double (&M)[4][4],double (&A)[4][4],double (&B)[4][4]){
    for(int i =0; i< 4; i++){
        for(int j =0; j< 4; j++){
            M[i][j] = 0.0;
            for(int s =0; s< 4; s++){
                M[i][j] += A[i][s]*B[s][j];
            }
        }
    }
}

void MatProd_fast3_3(double (&M)[3][3],double (&A)[3][3],double (&B)[3][3]){
    for(int i =0; i< 3; i++){
        for(int j =0; j< 3; j++){
            M[i][j] = 0.0;
            for(int s =0; s< 3; s++){
                M[i][j] += A[i][s]*B[s][j];
            }
        }
    }
}

void MatProd_fast4_Vect(double (&M)[4][1],double (&A)[4][4],double (&B)[4][1]){
    for(int i =0; i< 4; i++){
        for(int j =0; j< 1; j++){
            M[i][j] = 0.0;
            for(int s =0; s< 4; s++){
                M[i][j] += A[i][s]*B[s][j];
            }
        }
    }
}

void to_identity3_3(double (&mat)[3][3]){
    for(int i =0; i< 3; i++){
        for(int j =0; j< 3; j++){
            if(j==i){
                mat[i][j]=1.0;
            }
            else{
                mat[i][j]=0.0;
            }
        }
    }
}

void to_identity4_4(double (&mat)[4][4]){
    for(int i =0; i< 4; i++){
        for(int j =0; j< 4; j++){
            if(j==i){
                mat[i][j]=1.0;
            }
            else{
                mat[i][j]=0.0;
            }
        }
    }
}

double scalar_projection_fast(double (&a)[3], double (&b)[3]) {
    //projection of 'a' into 'b': https://en.wikipedia.org/wiki/Vector_projection
    // Calculate dot product of a and b
    double dot_product = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    
    // Calculate squared norm of vector b
    double norm_b_squared = pow(b[0],2) + pow(b[1],2) + pow(b[2],2);
    
    // Calculate the projection
    double projection = (dot_product / norm_b_squared);
    
    return projection;
}

double scalar_projection(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    //projection of 'a' into 'b': https://en.wikipedia.org/wiki/Vector_projection
    // Calculate dot product of a and b
    double dot_product = a.dot(b);
    
    // Calculate squared norm of vector b
    double norm_b_squared = b.squaredNorm();
    
    // Calculate the projection
    double projection = (dot_product / norm_b_squared);
    
    return projection;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> get_4Dmatrix_from_transform(geometry_msgs::msg::TransformStamped transf){
    geometry_msgs::msg::Vector3 translation_ = transf.transform.translation;  //translation vector from new_frame to frame_sensor
    geometry_msgs::msg::Vector3 rotation_ = quaternion_to_euler3D(transf.transform.rotation);
    //Translation in homogeneous coordinates
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T(0,3) = translation_.x;
    T(1,3) = translation_.y;
    T(2,3) = translation_.z;
    //3D rotation matrix
    Eigen::MatrixXd R_temp = rot_matrix_from_euler(rotation_);
    //3D rotation in homogeneous coordinates
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    for(int i = 0; i<3; i++){
        for(int y = 0; y<3; y++){
            R(i,y) = R_temp(i,y);
        }
    }
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to init_frame

    Eigen::MatrixXd M1_2(4, 4);
    M1_2 = T*R;

    return std::make_tuple(T, R, M1_2);

}

void get_4Dmatrix_from_transform_fast(double (&M)[4][4],geometry_msgs::msg::TransformStamped transf){
    geometry_msgs::msg::Vector3 translation_ = transf.transform.translation;  //translation vector from new_frame to frame_sensor
    geometry_msgs::msg::Vector3 rotation_ = quaternion_to_euler3D(transf.transform.rotation);
    //Translation in homogeneous coordinates
    double T[4][4] = {
                    {1.0,0.0,0.0,translation_.x},
                    {0.0,1.0,0.0,translation_.y},
                    {0.0,0.0,1.0,translation_.z},
                    {0.0,0.0,0.0,1.0}
                    };
    //3D rotation matrix
    double R_temp[3][3];
    rot_matrix_from_euler_fast(R_temp,rotation_);
    //3D rotation in homogeneous coordinates
    double R[4][4];
    to_identity4_4(R);
    for(int i = 0; i<3; i++){
        for(int y = 0; y<3; y++){
            R[i][y] = R_temp[i][y];
        }
    }
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to init_frame
    double M1_2[4][4];
    MatProd_fast4_4(M1_2,T,R);
}

Eigen::MatrixXd rot_matrix_from_euler(geometry_msgs::msg::Vector3 euler_angles){
    Eigen::MatrixXd Rx = Eigen::MatrixXd::Identity(3, 3);
    Rx(1,1) = cos(euler_angles.x);
    Rx(1,2) = -sin(euler_angles.x);
    Rx(2,1) = sin(euler_angles.x);
    Rx(2,2) = cos(euler_angles.x);
    Eigen::MatrixXd Ry = Eigen::MatrixXd::Identity(3, 3);
    Ry(0,0) = cos(euler_angles.y);
    Ry(0,2) = sin(euler_angles.y);
    Ry(2,0) = -sin(euler_angles.y);
    Ry(2,2) = cos(euler_angles.y);
    Eigen::MatrixXd Rz = Eigen::MatrixXd::Identity(3, 3);
    Rz(0,0) = cos(euler_angles.z);
    Rz(0,1) = -sin(euler_angles.z);
    Rz(1,0) = sin(euler_angles.z);
    Rz(1,1) = cos(euler_angles.z);
    return Rx*Ry*Rz;
}

void rot_matrix_from_euler_fast(double (&R)[3][3], geometry_msgs::msg::Vector3 euler_angles){
    double Rx[3][3];
    to_identity3_3(Rx);
    Rx[1][1] = cos(euler_angles.x);
    Rx[1][2] = -sin(euler_angles.x);
    Rx[2][1] = sin(euler_angles.x);
    Rx[2][2] = cos(euler_angles.x);
    double Ry[3][3];
    to_identity3_3(Ry);
    Ry[0][0] = cos(euler_angles.y);
    Ry[0][2] = sin(euler_angles.y);
    Ry[2][0] = -sin(euler_angles.y);
    Ry[2][2] = cos(euler_angles.y);
    double Rz[3][3];
    to_identity3_3(Rz);
    Rz[0][0] = cos(euler_angles.z);
    Rz[0][1] = -sin(euler_angles.z);
    Rz[1][0] = sin(euler_angles.z);
    Rz[1][1] = cos(euler_angles.z);
    double R_temp[3][3];
    MatProd_fast3_3(R_temp,Ry,Rz);
    MatProd_fast3_3(R,Rx,R_temp);
}

geometry_msgs::msg::Vector3 quaternion_to_euler3D(geometry_msgs::msg::Quaternion quat){
    // Convert geometry_msgs::msg::Quaternion to Eigen::Quaterniond
    Eigen::Quaterniond eigen_quaternion(
        quat.w,
        quat.x,
        quat.y,
        quat.z
    );
    // Convert quaternion to Euler angles
    Eigen::Vector3d euler_angles = eigen_quaternion.toRotationMatrix().eulerAngles(0, 1, 2); 
    geometry_msgs::msg::Vector3 rot_vect;
    rot_vect.x = euler_angles(0);
    rot_vect.y = euler_angles(1);
    rot_vect.z = euler_angles(2);
    //RCLCPP_INFO(this->get_logger(), "ROT Quat: x='%.2f', y='%.2f', z='%.2f', w='%.2f'", quat.x,quat.y,quat.z,quat.w);
    //RCLCPP_INFO(this->get_logger(), "ROT Euler: x='%.2f', y='%.2f', z='%.2f'", rot_vect.x,rot_vect.y,rot_vect.z);
    return rot_vect;
}

double TimeToDouble(builtin_interfaces::msg::Time& stamp){
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time DoubleToTime(double& seconds){
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(seconds);
    time_msg.nanosec = static_cast<uint32_t>((seconds - time_msg.sec) * 1e9);
    return time_msg;
}

bool consider_val(int current_ind, int start_ind, int end_ind){
    // return true if current_ind is between start_ind and end_ind according to a circle reference.
    if(start_ind>end_ind){ //if interval pass throught the origin of the circle, we test considering the split into 2 interval
        return (current_ind>=start_ind || current_ind<=end_ind);
    }
    else{ // if values are equal, or classical ,we test as classic interval
        return (current_ind>=start_ind && current_ind<=end_ind);
    }
}

int angle_to_index(double alpha, int resolution){
    //return index of angle alpha, in a table with 'resolution' values placed from 0 to 360 degree.
    // Normalize the angle to the range [0, 2*M_PI)
    alpha = std::fmod(alpha, 2 * M_PI); //return negative number if alpha is negative initially
    //debug_ss << "\nAngle_to_index: "<< " mod: " << alpha;
    if (alpha < 0) {
        alpha += 2 * M_PI;
    }
    // Calculate the index
    int ind = static_cast<int>(round((alpha * resolution)/(2*M_PI)));
    //debug_ss <<  " alpha*reso: " << alpha*resolution << " ind: " << ind;
    return ind;
}

double index_to_angle(int ind, int resolution, double elongation){ //default elongation to 2*PI, see function signature
    return (ind*elongation)/resolution;
}

void remap_scan(sensor_msgs::msg::LaserScan::SharedPtr input_scan, sensor_msgs::msg::LaserScan::SharedPtr output_scan, std::vector<double> &mask_to_update){
    int prev_reso = input_scan->ranges.size();
    double prev_angle_start = input_scan->angle_min;
    double prev_angle_end = input_scan->angle_max;
    int new_reso = output_scan->ranges.size();
    double new_angle_start = output_scan->angle_min;
    double new_angle_end = output_scan->angle_max;
    for(int i =0; i<prev_reso; i++){
        //this remap will only be influenced if the source as not an origin of 0.0 and if the angle_increment is different from the wanted final scan
        int new_i = remap_scan_index(i, prev_angle_start, prev_angle_end, prev_reso, new_angle_start, new_angle_end, new_reso);
        //new_i is indexes in a 360deg scan with angle_incr and angle start of output_scan
        if(consider_val(new_i,0,new_reso-1)){
            if(input_scan->ranges[i]<output_scan->ranges[new_i]){
                output_scan->ranges[new_i] = input_scan->ranges[i];
                output_scan->intensities[new_i] = input_scan->intensities[i];
            }
            mask_to_update[new_i] = 1.0;
        }
        else{
            if(new_i < new_reso){ //otherwise if mask_to_udate is smaller, the index will be out of bounds 
                mask_to_update[new_i] = 0.0;
            }
        }
    }
}

int remap_scan_index(int prev_ind, double prev_angle_start, double prev_angle_end, double prev_reso, double new_angle_start, double new_angle_end, double new_reso){
    int new_ind_360;
    /*
    return the index in a new scan list. But return it as if the final scan was a 360deg scan, because otherwise some indexes are not possible, so this function needs to be use with consider_val to filter the points you want to keep.
    */
    double prev_elong = prev_angle_end - prev_angle_start;  
    double new_elong = new_angle_end - new_angle_start;
    double prev_angle_incr = prev_elong/prev_reso;
    double new_angle_incr = new_elong/new_reso;
    int reso_360 = static_cast<int>(round(2*M_PI/prev_angle_incr)); //resoluton that would have prev_list if on a 360deg circle list
    int prev_ind_360 = fmod(prev_ind + angle_to_index(prev_angle_start,reso_360),reso_360); //index that would be prev_ind in a 360 deg scan

    //offset gestion
    //the angle offset according to the new scan we have with prev_ind_360 is the start angle of the new scan
    int ind_offset = angle_to_index(new_angle_start,reso_360); //should return index between [0,reso_360]
    //new index considering offset in a 360deg scan with reso_360
    int new_ind_360_temp = fmod(prev_ind_360 - ind_offset,reso_360);
    if (new_ind_360_temp<0){
        new_ind_360_temp += reso_360;
    }

    //different reso gestion
    new_ind_360 = static_cast<int>(round((prev_angle_incr/new_angle_incr)*new_ind_360_temp));
    //new_ind_360 = angle_to_index(index_to_angle(new_ind_360_temp,reso_360),new_reso);

    return new_ind_360;
}

double sawtooth(double x) {
    return std::fmod(x+M_PI,2*M_PI)-M_PI;
}


/*TRASH (need to be tested before use)

geometry_msgs::msg::TransformStamped tf_offset(geometry_msgs::msg::TransformStamped &relative_transform, geometry_msgs::msg::TransformStamped tf1, geometry_msgs::msg::TransformStamped tf2){
    //give transformation from tf1 to tf2

    Eigen::Quaterniond rot_1(tf1.transform.rotation.w,
                            tf1.transform.rotation.x,
                            tf1.transform.rotation.y,
                            tf1.transform.rotation.z);

    Eigen::Quaterniond rot_2(tf2.transform.rotation.w,
                            tf2.transform.rotation.x,
                            tf2.transform.rotation.y,
                            tf2.transform.rotation.z);

    // Perform quaternion subtraction
    Eigen::Quaterniond relative_rotation = rot_2 * rot_1.inverse();

    // Subtract translation components
    double off_x = tf2.transform.translation.x - tf1.transform.translation.x;
    double off_y = tf2.transform.translation.y - tf1.transform.translation.y;
    double off_z = tf2.transform.translation.z - tf1.transform.translation.z;

    // relative_transform represents the difference between the two original transforms
    relative_transform.header = tf2.header;
    relative_transform.transform.rotation.w = relative_rotation.w();
    relative_transform.transform.rotation.x = relative_rotation.x();
    relative_transform.transform.rotation.y = relative_rotation.y();
    relative_transform.transform.rotation.z = relative_rotation.z();
    relative_transform.transform.translation.x = off_x;
    relative_transform.transform.translation.y = off_y;
    relative_transform.transform.translation.z = off_z;
}


void shift_cloud_from_tf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::msg::TransformStamped transf){
    //transf need to be the transformation from new_frame to old_frame
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Pass_matrixes_new_old = get_4Dmatrix_from_transform(transf);
    //transform each points
    for(int i=0; i<cloud->size(); i++){
        Eigen::MatrixXd Point_old = Eigen::MatrixXd::Identity(4, 1);
        Point_old(0,0) = cloud->points[i].x;
        Point_old(1,0) = cloud->points[i].y;
        Point_old(2,0) = cloud->points[i].z;
        Point_old(3,0) = 1.0;
        
        Eigen::MatrixXd Point_new = std::get<2>(Pass_matrixes_new_old)*Point_old;

        cloud->points[i].x = Point_new(0,0);
        cloud->points[i].y = Point_new(1,0);
        cloud->points[i].z = Point_new(2,0);
    }
}

*/