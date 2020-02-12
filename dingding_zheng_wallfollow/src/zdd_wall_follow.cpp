#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Time.h>
#include <math.h>
#include <array>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <unistd.h>



// PID control params:
double kp = 4;
double ki = 0.00005;
double kd = 0.0001;
double servo_offset;
double prev_error;
double error = 0.0;
double integral = 0.0;

// Define constants:
#define ANGLE_RANGE 270
#define DESIRED_DISTANCE_RIGHT 0.9
#define DESIRED_DISTANCE_LEFT 1.25
#define VELOCITY 2.0
#define CAR_LENGTH 0.5

// math definitions
#define _USE_MATH_DEFINES

// wall follow class
class Wall_Follow {
private:
    ros::NodeHandle nd;
    double velo;
    ros::Subscriber scan;
    ros::Publisher input;

public:
    Wall_Follow(){
        nd = ros::NodeHandle();
        // input = nd.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
        input = nd.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
        scan = nd.subscribe("/scan", 100, &Wall_Follow::scan_callback, this);
    }

    double follow_left(std::vector<float> dis_range){
        double alpha,theta,a,b;
        int a_pt = 2*ANGLE_RANGE - 2;
        a = dis_range.at(a_pt);
        b = dis_range.at(673);
        theta = M_PI/4;
        alpha = atan((cos(theta)*a - b)/(a*sin(theta)));
        double distance2wall = b * cos(alpha);
        double L = 0.15;// predictive length
        double new_distance2wall = distance2wall + L * sin(theta);
        //error = Dt - DESIRED_DISTANCE_LEFT;
        prev_error = error;
        error = new_distance2wall - DESIRED_DISTANCE_LEFT;
        double str_ang, velo;
        integral += error;
        double derivative = (error - prev_error)/0.01;
        str_ang = kp * error + ki * integral + kd * derivative;
        ROS_INFO("Steering Angle is [%f]",str_ang);
        return str_ang;

    }

    double choose_velo(double ang){
        double velo;
        double degree_ang = 180 * ang/M_PI;
        if(abs(degree_ang)<10){
            velo = 1.5;
        }else if(degree_ang>=10&&degree_ang<20){
            velo = 1.0;
        }else{
            velo = 0.5;
        }
        return velo;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
        double max_ang = scan_msg->angle_max;
        double min_ang = scan_msg->angle_min;
        double delta_ang = scan_msg->angle_increment;
        int sample_range = int((max_ang - min_ang)/delta_ang);
        std::vector<float> msg_range = scan_msg->ranges;

        double str_ang, speed;
        str_ang = Wall_Follow::follow_left(msg_range);
        if(str_ang >= 0.43){
            str_ang = 0.43;
        }else if(str_ang <= -0.43){
            str_ang = -0.43;
        }
        speed = choose_velo(str_ang);
        ROS_INFO("Now, speed is [%f]", speed);
        ackermann_msgs::AckermannDriveStamped drv_msgs;
        drv_msgs.header.frame_id = "laser";
        drv_msgs.drive.steering_angle = str_ang;
        drv_msgs.drive.speed = speed;
        input.publish(drv_msgs);

    }
};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "zdd_wall_follow");
    Wall_Follow obj;
    ros::spin();
    return 0;
}




