/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Eigen>
#include <mutex>

#include <mrs_lib/ParamLoader.h>

#include <compton_camera_filter/trajectory_plannerConfig.h>

#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerPoint.h>
#include <compton_camera_filter/Swarm.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>

#include <tf/transform_datatypes.h>

//}

#define STRING_EQUAL 0

namespace compton_camera_filter
{

/* TrajectoryPlanner //{ */
class TrajectoryPlanner : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;
  std::string     uav_name_;

  ros::Publisher publisher_trajectory;
  ros::Publisher publisher_swarm_control;

  double validateHeading(const double yaw_in);

private:
  ros::Subscriber subscriber_pose;
  void            callbackPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  bool            got_pose = false;
  std::mutex      mutex_radiation_pose;

  geometry_msgs::PoseWithCovarianceStamped radiation_pose;

private:
  ros::Subscriber subscriber_optimizer;
  void            callbackOptimizer(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  bool            got_optimizer = false;
  std::mutex      mutex_optimizer;

  ros::ServiceServer service_server_search;

private:
  void               callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  std::mutex         mutex_odometry;
  nav_msgs::Odometry odometry;
  ros::Subscriber    subscriber_odometry;
  bool               got_odometry = false;
  double             odometry_yaw;
  double             odometry_roll;
  double             odometry_pitch;

  geometry_msgs::PoseWithCovarianceStamped optimizer;

private:
  ros::Subscriber subscriber_swarm_control;
  void            callbackSwarmControl(const compton_camera_filter::SwarmConstPtr &msg);
  bool            got_swarm = false;

  std::mutex                                mutex_swarm_uavs;
  std::vector<compton_camera_filter::Swarm> swarm_uavs_list;
  std::map<std::string, int>                swarm_uavs_map;

private:
  ros::Timer main_timer;
  int        main_timer_rate_;
  void       mainTimer(const ros::TimerEvent &event);

private:
  ros::Timer swarm_timer;
  int        swarm_timer_rate;
  void       swarmTimer(const ros::TimerEvent &event);

private:
  mrs_msgs::TrackerTrajectory trackingTrajectory(void);
  mrs_msgs::TrackerTrajectory searchingTrajectory(void);

private:
  double searching_x, searching_y;

private:
  bool callbackSearch([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

private:
  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  double     tracking_radius_, tracking_speed_, tracking_height_, tracking_trajectory_steps_;
  double     searching_radius_, searching_speed_, searching_height_, searching_trajectory_steps_, searching_yaw_rate_;
  std::mutex mutex_params;

  boost::recursive_mutex                                  config_mutex_;
  typedef compton_camera_filter::trajectory_plannerConfig Config;
  typedef dynamic_reconfigure::Server<Config>             ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                    reconfigure_server_;
  void                                                    drs_callback(compton_camera_filter::trajectory_plannerConfig &config, uint32_t level);
  compton_camera_filter::trajectory_plannerConfig         drs_trajectory_planner;

  void       dynamicReconfigureCallback(compton_camera_filter::trajectory_plannerConfig &config, uint32_t level);
  std::mutex mutex_drs;
};
//}

/* inInit() //{ */

void TrajectoryPlanner::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[TrajectoryPlanner]: initializing");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "TrajectoryPlanner");

  param_loader.load_param("uav_name", uav_name_);

  param_loader.load_param("main_timer_rate", main_timer_rate_);
  param_loader.load_param("swarm_timer_rate", swarm_timer_rate);

  param_loader.load_param("tracking/radius", tracking_radius_);
  param_loader.load_param("tracking/speed", tracking_speed_);
  param_loader.load_param("tracking/height", tracking_height_);
  param_loader.load_param("tracking/trajectory_steps", tracking_trajectory_steps_);

  param_loader.load_param("searching/initial_position/x", searching_x);
  param_loader.load_param("searching/initial_position/y", searching_y);
  param_loader.load_param("searching/radius", searching_radius_);
  param_loader.load_param("searching/speed", searching_speed_);
  param_loader.load_param("searching/height", searching_height_);
  param_loader.load_param("searching/yaw_rate", searching_yaw_rate_);
  param_loader.load_param("searching/trajectory_steps", searching_trajectory_steps_);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_pose          = nh_.subscribe("pose_in", 1, &TrajectoryPlanner::callbackPose, this, ros::TransportHints().tcpNoDelay());
  subscriber_optimizer     = nh_.subscribe("optimizer_in", 1, &TrajectoryPlanner::callbackOptimizer, this, ros::TransportHints().tcpNoDelay());
  subscriber_odometry      = nh_.subscribe("odom_in", 1, &TrajectoryPlanner::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_swarm_control = nh_.subscribe("swarm_in", 1, &TrajectoryPlanner::callbackSwarmControl, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_trajectory    = nh_.advertise<mrs_msgs::TrackerTrajectory>("trajectory_out", 1);
  publisher_swarm_control = nh_.advertise<compton_camera_filter::Swarm>("swarm_out", 1);

  // --------------------------------------------------------------
  // |                       service servers                      |
  // --------------------------------------------------------------

  service_server_search = nh_.advertiseService("search_in", &TrajectoryPlanner::callbackSearch, this);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer  = nh_.createTimer(ros::Rate(main_timer_rate_), &TrajectoryPlanner::mainTimer, this);
  swarm_timer = nh_.createTimer(ros::Rate(swarm_timer_rate), &TrajectoryPlanner::swarmTimer, this);

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  drs_trajectory_planner.tracking_speed  = tracking_speed_;
  drs_trajectory_planner.tracking_radius = tracking_radius_;
  drs_trajectory_planner.tracking_height = tracking_height_;

  drs_trajectory_planner.searching_speed  = searching_speed_;
  drs_trajectory_planner.searching_radius = searching_radius_;
  drs_trajectory_planner.searching_height = searching_height_;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_trajectory_planner);
  ReconfigureServer::CallbackType f = boost::bind(&TrajectoryPlanner::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[TrajectoryPlanner]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[TrajectoryPlanner]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackPose() //{ */

void TrajectoryPlanner::callbackPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized)
    return;

  ROS_INFO_ONCE("[TrajectoryPlanner]: getting pose");

  std::scoped_lock lock(mutex_radiation_pose);

  got_pose = true;

  radiation_pose = *msg;
}

//}

/* callbackOdometry() //{ */

void TrajectoryPlanner::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  ROS_INFO_ONCE("[TrajectoryPlanner]: getting odometry");

  std::scoped_lock lock(mutex_odometry);

  got_odometry = true;

  odometry = *msg;

  // calculate the euler angles
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(odometry.pose.pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(odometry_roll, odometry_pitch, odometry_yaw);
}

//}

/* callbackSwarmControl() //{ */

void TrajectoryPlanner::callbackSwarmControl(const compton_camera_filter::SwarmConstPtr &msg) {

  if (!is_initialized)
    return;

  if (msg->uav_name.compare(uav_name_) == STRING_EQUAL) {
    return;
  }

  std::scoped_lock lock(mutex_swarm_uavs);

  ROS_INFO_ONCE("[TrajectoryPlanner]: getting other uavs");

  // get the idx of the uav_name
  auto uav_in_map = swarm_uavs_map.find(msg->uav_name);

  if (uav_in_map == swarm_uavs_map.end()) {

    ROS_INFO("[TrajectoryPlanner]: registering new uav: %s", msg->uav_name.c_str());

    swarm_uavs_list.push_back(*msg);
    swarm_uavs_map.insert(std::pair(msg->uav_name, swarm_uavs_list.size() - 1));

  } else {

    // update the uav in the list
    swarm_uavs_list.at(uav_in_map->second) = *msg;
  }
}

//}

/* callbackOptimizer() //{ */

void TrajectoryPlanner::callbackOptimizer(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_optimizer);

  got_optimizer = true;

  optimizer = *msg;
}

//}

/* dynamicReconfigureCallback() //{ */

void TrajectoryPlanner::dynamicReconfigureCallback(compton_camera_filter::trajectory_plannerConfig &config, [[maybe_unused]] uint32_t level) {

  std::scoped_lock lock(mutex_params);

  tracking_radius_ = config.tracking_radius;
  tracking_speed_  = config.tracking_speed;
  tracking_height_ = config.tracking_height;

  searching_radius_ = config.searching_radius;
  searching_speed_  = config.searching_speed;
  searching_height_ = config.searching_height;

  ROS_INFO("[TrajectoryPlanner]: drs updated params");
}

//}

/* callbackSearch() //{ */

bool TrajectoryPlanner::callbackSearch([[maybe_unused]] std_srvs::Trigger::Request &req, [[maybe_unused]] std_srvs::Trigger::Response &res) {

  got_pose = false;

  ROS_INFO("[TrajectoryPlanner]: switching back to search");

  res.message = "";
  res.success = true;

  return true;
}

//}

// --------------------------------------------------------------
// |                       custom routines                      |
// --------------------------------------------------------------

/* validateHeading() //{ */

double TrajectoryPlanner::validateHeading(const double yaw_in) {

  double yaw_out = yaw_in;

  if (!std::isfinite(yaw_out)) {

    yaw_out = 0;
    ROS_ERROR("[validateYawSetpoint]: Desired YAW is not finite number!");
  }

  if (fabs(yaw_out) > 1000) {

    ROS_WARN("[validateYawSetpoint]: Desired YAW is > 1000");
  }
  // if desired yaw_out is grater then 2*PI mod it
  if (fabs(yaw_out) > 2 * M_PI) {
    yaw_out = fmod(yaw_out, 2 * M_PI);
  }

  // move it to its place
  if (yaw_out > M_PI) {
    yaw_out -= 2 * M_PI;
  } else if (yaw_out < -M_PI) {
    yaw_out += 2 * M_PI;
  }

  return yaw_out;
}

//}

/* trackingTrajectory() //{ */

mrs_msgs::TrackerTrajectory TrajectoryPlanner::trackingTrajectory(void) {

  // get current angle
  double current_angle =
      atan2(odometry.pose.pose.position.y - radiation_pose.pose.pose.position.y, odometry.pose.pose.position.x - radiation_pose.pose.pose.position.x);

  // calculate the angle bias
  double closest_dist = 2 * M_PI;
  double angle_bias   = 0;

  {
    std::scoped_lock lock(mutex_swarm_uavs);

    for (std::vector<compton_camera_filter::Swarm>::iterator it = swarm_uavs_list.begin(); it != swarm_uavs_list.end(); it++) {

      double dist = fabs(validateHeading(it->orbit_angle) - validateHeading(current_angle));

      if (dist < closest_dist) {

        closest_dist = dist;
        angle_bias   = (it->orbit_angle - current_angle > 0) ? -0.10 : 0.10;
      }
    }
  }

  ROS_INFO_THROTTLE(1.0, "[TrajectoryPlanner]: angle_bias: %2.2f", angle_bias);

  current_angle += angle_bias;

  // calculate the angular velocity
  double angular_step = (tracking_speed_ / tracking_radius_) / 5.0;

  std::scoped_lock lock(mutex_radiation_pose, mutex_odometry);

  // create the trajectory
  mrs_msgs::TrackerTrajectory new_trajectory;

  new_trajectory.fly_now         = true;
  new_trajectory.use_yaw         = true;
  new_trajectory.header.stamp    = ros::Time::now();
  new_trajectory.header.frame_id = "local_origin";

  for (int i = 0; i < tracking_trajectory_steps_; i++) {

    mrs_msgs::TrackerPoint new_point;

    new_point.x   = radiation_pose.pose.pose.position.x + tracking_radius_ * cos(current_angle);
    new_point.y   = radiation_pose.pose.pose.position.y + tracking_radius_ * sin(current_angle);
    new_point.z   = tracking_height_;
    new_point.yaw = atan2(radiation_pose.pose.pose.position.y - new_point.y, radiation_pose.pose.pose.position.x - new_point.x);

    current_angle += angular_step;

    new_trajectory.points.push_back(new_point);
  }

  // update the searching coordinates
  searching_x = radiation_pose.pose.pose.position.x;
  searching_y = radiation_pose.pose.pose.position.y;

  return new_trajectory;
}

//}

/* searchingTrajectory() //{ */

mrs_msgs::TrackerTrajectory TrajectoryPlanner::searchingTrajectory(void) {

  // get current angle
  double current_angle = atan2(odometry.pose.pose.position.y - searching_y, odometry.pose.pose.position.x - searching_x);

  // calculate the angle bias
  double closest_dist = 2 * M_PI;
  double angle_bias   = 0;

  {
    std::scoped_lock lock(mutex_swarm_uavs);

    for (std::vector<compton_camera_filter::Swarm>::iterator it = swarm_uavs_list.begin(); it != swarm_uavs_list.end(); it++) {

      double dist = fabs(validateHeading(it->orbit_angle) - validateHeading(current_angle));

      if (dist < closest_dist) {

        closest_dist = dist;
        angle_bias   = (it->orbit_angle - current_angle > 0) ? -0.10 : 0.10;
      }
    }
  }

  current_angle += angle_bias;

  ROS_INFO_THROTTLE(1.0, "[TrajectoryPlanner]: angle_bias: %2.2f", angle_bias);

  // calculate the angular velocity
  double angular_step = (searching_speed_ / searching_radius_) / 5.0;

  std::scoped_lock lock(mutex_odometry);

  // create the trajectory
  mrs_msgs::TrackerTrajectory new_trajectory;

  new_trajectory.fly_now         = true;
  new_trajectory.use_yaw         = true;
  new_trajectory.header.stamp    = ros::Time::now();
  new_trajectory.header.frame_id = "local_origin";

  double current_yaw = odometry_yaw;

  for (int i = 0; i < searching_trajectory_steps_; i++) {

    mrs_msgs::TrackerPoint new_point;

    current_yaw += searching_yaw_rate_ / 5.0;

    new_point.x   = searching_x + searching_radius_ * cos(current_angle);
    new_point.y   = searching_y + searching_radius_ * sin(current_angle);
    new_point.z   = searching_height_;
    new_point.yaw = current_yaw;
    /* new_point.yaw = atan2(new_point.y - searching_y, new_point.x - searching_x); */

    current_angle += angular_step;

    new_trajectory.points.push_back(new_point);
  }

  return new_trajectory;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void TrajectoryPlanner::mainTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized) {
    return;
  }

  if (!got_odometry) {
    return;
  }

  mrs_msgs::TrackerTrajectory new_trajectory;

  if (got_pose) {

    ROS_INFO("[TrajectoryPlanner]: publishing tracking trajectory");

    new_trajectory = trackingTrajectory();

  } else {

    ROS_INFO("[TrajectoryPlanner]: publishing searching trajectory");

    new_trajectory = searchingTrajectory();
  }

  try {
    publisher_trajectory.publish(new_trajectory);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_trajectory.getTopic().c_str());
  }
}

//}

/* swarmTimer() //{ */

void TrajectoryPlanner::swarmTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized) {
    return;
  }

  compton_camera_filter::Swarm swarm_out;

  swarm_out.orbit_angle =
      atan2(odometry.pose.pose.position.y - radiation_pose.pose.pose.position.y, odometry.pose.pose.position.x - radiation_pose.pose.pose.position.x);
  swarm_out.header.stamp = ros::Time::now();
  swarm_out.uav_name     = uav_name_;

  try {
    publisher_swarm_control.publish(swarm_out);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_swarm_control.getTopic().c_str());
  }
}

//}

}  // namespace compton_camera_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_camera_filter::TrajectoryPlanner, nodelet::Nodelet)
