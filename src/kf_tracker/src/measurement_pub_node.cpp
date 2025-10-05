#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <random>

/*
Generate noisy measurements to be fed into kalman filter.
*/

int main(int argc, char **argv){
    ros::init(argc, argv, "measurement_pub_node");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher gt_pub = nh.advertise<geometry_msgs::Point>("/gt_pose", 10);
    ros::Publisher measurement_pub = nh.advertise<geometry_msgs::Point>("/measurements_pose", 10);

    // define GT motion model
    double dt = 0.1;
    int i=0;
    ros::Rate rate(10);

    // for measurement noise
    std::mt19937 rng(42);
    double noise_std = 0.1;
    std::normal_distribution<double> N(0.0, noise_std);

    while(ros::ok()){
        geometry_msgs::Point gt{};
        gt.x = dt * i;
        gt.y = 1;
        gt_pub.publish(gt);
        i++;

        geometry_msgs::Point meas{};
        meas.x = gt.x + N(rng);
        meas.y = gt.y + N(rng);
        measurement_pub.publish(meas);

        ros::spinOnce();
        rate.sleep();

    }

    // add noise
    
    // publish measurement

}
