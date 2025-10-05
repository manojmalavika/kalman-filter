#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"


void measurement_callback(const geometry_msgs::Point::ConstPtr& msg){
    ROS_INFO("[Tracker] Received measurement: x:%.2f, y=%.2f", msg->x, msg->y);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;

    ros::Publisher estimate_pub = nh.advertise<geometry_msgs::Point>("/pose_estimate", 10);

    ros::Subscriber measurement_sub = nh.subscribe("/measurement", 10, measurement_callback);
    ros::Rate rate(1);

    while(ros::ok()){
        geometry_msgs::Point est_msg;
        estimate_pub.publish(est_msg);
        ROS_INFO("[Tracker] Publishing estimate: (%.2f, %.2f)", est_msg.x, est_msg.y);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;

}
