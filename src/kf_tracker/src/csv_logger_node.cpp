#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <iomanip>   // optional, for nicer formatting

// latest samples (simple, single-thread: callbacks + loop share one thread)
geometry_msgs::Point last_gt{};
geometry_msgs::Point last_meas{};
bool have_gt = false, have_meas = false;

// 1) Callbacks: store the most recent messages
void gt_cb(const geometry_msgs::Point::ConstPtr& msg) {
  last_gt = *msg;
  have_gt = true;
}

void meas_cb(const geometry_msgs::Point::ConstPtr& msg) {
  last_meas = *msg;
  have_meas = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "csv_logger_node");
  ros::NodeHandle nh;

  // 2) Open CSV file (host path: ~/ros1_ws/logs/gt_meas.csv)
  std::ofstream log("/root/catkin_ws/logs/gt_meas.csv", std::ios::out);
  if (!log.is_open()) {
    ROS_ERROR("Failed to open /root/catkin_ws/logs/gt_meas.csv");
    return 1;
  }
  // header
  log << "t_sec,gt_x,gt_y,meas_x,meas_y\n";

  // 3) Subscriptions
  auto gt_sub   = nh.subscribe("/gt_pose", 10, gt_cb);
  auto meas_sub = nh.subscribe("/measurements_pose", 10, meas_cb);

  ros::Rate rate(10);  // log ~10 Hz
  while (ros::ok()) {
    ros::spinOnce();

    // // 4) Write a row each tick (even if one stream hasn’t arrived yet)
    // double t = ros::Time::now().toSec();
    // log << std::fixed << std::setprecision(6)
    //     << t << ","
    //     << last_gt.x << "," << last_gt.y << ","
    //     << last_meas.x << "," << last_meas.y << "\n";

    // after ros::spinOnce();
    if (have_gt && have_meas) {
    double t = ros::Time::now().toSec();
    log << std::fixed << std::setprecision(6)
        << t << ","
        << last_gt.x << "," << last_gt.y << ","
        << last_meas.x << "," << last_meas.y << "\n";
    }

    // (optional) flush occasionally for safety:
    // if (static int n=0; (++n % 20) == 0) log.flush();

    rate.sleep();
  }
  log.close();
  return 0;
}
