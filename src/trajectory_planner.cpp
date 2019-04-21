#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Eigen>
#include <mutex>

#include <mrs_lib/ParamLoader.h>

#include <compton_camera_filter/trajectory_plannerConfig.h>

#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerPoint.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>

namespace compton_camera_filter
{

  /* TrajectoryPlanner //{ */
  class TrajectoryPlanner : public nodelet::Nodelet {

  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh_;
    bool            is_initialized = false;

    ros::Publisher publisher_trajectory;

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

    geometry_msgs::PoseWithCovarianceStamped optimizer;

  private:
    ros::Timer main_timer;
    int        main_timer_rate_;
    void       mainTimer(const ros::TimerEvent &event);

  private:
    mrs_msgs::TrackerTrajectory trackingTrajectory(void);
    mrs_msgs::TrackerTrajectory searchingTrajectory(void);

  private:
    bool callbackSearch([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  private:
    // --------------------------------------------------------------
    // |                     dynamic reconfigure                    |
    // --------------------------------------------------------------

    double     tracking_radius_, tracking_speed_, tracking_height_, tracking_trajectory_steps_;
    double     searching_radius_, searching_speed_, searching_height_, searching_trajectory_steps_;
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

    param_loader.load_param("main_timer_rate", main_timer_rate_);

    param_loader.load_param("tracking/radius", tracking_radius_);
    param_loader.load_param("tracking/speed", tracking_speed_);
    param_loader.load_param("tracking/height", tracking_height_);
    param_loader.load_param("tracking/trajectory_steps", tracking_trajectory_steps_);

    param_loader.load_param("searching/radius", searching_radius_);
    param_loader.load_param("searching/speed", searching_speed_);
    param_loader.load_param("searching/height", searching_height_);
    param_loader.load_param("searching/trajectory_steps", searching_trajectory_steps_);

    // --------------------------------------------------------------
    // |                         subscribers                        |
    // --------------------------------------------------------------

    subscriber_pose      = nh_.subscribe("pose_in", 1, &TrajectoryPlanner::callbackPose, this, ros::TransportHints().tcpNoDelay());
    subscriber_optimizer = nh_.subscribe("optimizer_in", 1, &TrajectoryPlanner::callbackOptimizer, this, ros::TransportHints().tcpNoDelay());
    subscriber_odometry  = nh_.subscribe("odom_in", 1, &TrajectoryPlanner::callbackOdometry, this, ros::TransportHints().tcpNoDelay());

    // --------------------------------------------------------------
    // |                         publishers                         |
    // --------------------------------------------------------------

    publisher_trajectory = nh_.advertise<mrs_msgs::TrackerTrajectory>("trajectory_out", 1);

    // --------------------------------------------------------------
    // |                       service servers                      |
    // --------------------------------------------------------------

    service_server_search = nh_.advertiseService("search_in", &TrajectoryPlanner::callbackSearch, this);

    // --------------------------------------------------------------
    // |                           timers                           |
    // --------------------------------------------------------------

    main_timer = nh_.createTimer(ros::Rate(main_timer_rate_), &TrajectoryPlanner::mainTimer, this);

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

  /* trackingTrajectory() //{ */

  mrs_msgs::TrackerTrajectory TrajectoryPlanner::trackingTrajectory(void) {

    // get current angle
    double current_angle =
        atan2(odometry.pose.pose.position.y - radiation_pose.pose.pose.position.y, odometry.pose.pose.position.x - radiation_pose.pose.pose.position.x);

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

    return new_trajectory;
  }

  //}

  /* searchingTrajectory() //{ */

  mrs_msgs::TrackerTrajectory TrajectoryPlanner::searchingTrajectory(void) {

    // get current angle
    double current_angle = atan2(odometry.pose.pose.position.y - 0, odometry.pose.pose.position.x - 0);

    // calculate the angular velocity
    double angular_step = (searching_speed_ / searching_radius_) / 5.0;

    std::scoped_lock lock(mutex_odometry);

    // create the trajectory
    mrs_msgs::TrackerTrajectory new_trajectory;

    new_trajectory.fly_now         = true;
    new_trajectory.use_yaw         = true;
    new_trajectory.header.stamp    = ros::Time::now();
    new_trajectory.header.frame_id = "local_origin";

    for (int i = 0; i < searching_trajectory_steps_; i++) {

      mrs_msgs::TrackerPoint new_point;

      new_point.x   = 0 + searching_radius_ * cos(current_angle);
      new_point.y   = 0 + searching_radius_ * sin(current_angle);
      new_point.z   = searching_height_;
      new_point.yaw = atan2(new_point.y - 0, new_point.x - 0) + M_PI/2;

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

}  // namespace compton_camera_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_camera_filter::TrajectoryPlanner, nodelet::Nodelet)
