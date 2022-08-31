/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <rad_msgs/Cone.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Eigen>
#include <mutex>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/lkf.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/attitude_converter.h>

#include <compton_camera_filter/compton_filterConfig.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <dynamic_reconfigure/server.h>

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

namespace compton_camera_filter
{

/* LKF helpers //{ */

// Define the LKF we will be using
const int _2d_n_states_       = 2;
const int _2d_n_inputs_       = 0;
const int _2d_n_measurements_ = 2;

using lkf_2d_t = mrs_lib::LKF<_2d_n_states_, _2d_n_inputs_, _2d_n_measurements_>;

using A_2d_t        = lkf_2d_t::A_t;
using B_2d_t        = lkf_2d_t::B_t;
using H_2d_t        = lkf_2d_t::H_t;
using Q_2d_t        = lkf_2d_t::Q_t;
using x_2d_t        = lkf_2d_t::x_t;
using P_2d_t        = lkf_2d_t::P_t;
using R_2d_t        = lkf_2d_t::R_t;
using statecov_2d_t = lkf_2d_t::statecov_t;

// Define the LKF we will be using
const int _3d_n_states_       = 3;
const int _3d_n_inputs_       = 0;
const int _3d_n_measurements_ = 3;

using lkf_3d_t = mrs_lib::LKF<_3d_n_states_, _3d_n_inputs_, _3d_n_measurements_>;

using A_3d_t        = lkf_3d_t::A_t;
using B_3d_t        = lkf_3d_t::B_t;
using H_3d_t        = lkf_3d_t::H_t;
using Q_3d_t        = lkf_3d_t::Q_t;
using x_3d_t        = lkf_3d_t::x_t;
using P_3d_t        = lkf_3d_t::P_t;
using R_3d_t        = lkf_3d_t::R_t;
using statecov_3d_t = lkf_3d_t::statecov_t;

//}

/* ComptonFilter //{ */
class ComptonFilter : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;

  std::string uav_name_;
  std::string _world_frame_;

  // | -------------------------- libs -------------------------- |

  mrs_lib::BatchVisualizer batch_visualizer_;
  mrs_lib::BatchVisualizer batch_visualizer_2d_hypo_;
  mrs_lib::BatchVisualizer batch_visualizer_3d_hypo_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher publisher_2d_;
  ros::Publisher publisher_3d_;
  ros::Publisher publisher_2d_correction_;
  ros::Publisher publisher_3d_correction_;

  // | --------------------- service cleints -------------------- |

  ros::ServiceClient service_client_localization_;
  ros::ServiceClient service_client_sweeper_;
  ros::ServiceClient service_client_optimizer_reset_;

  // | ----------------------- subscribers ---------------------- |

  ros::Subscriber subscriber_cone;
  void            callbackCone(const rad_msgs::ConeConstPtr& msg);
  ros::Time       cone_last_time_;
  std::mutex      mutex_cone_last_time;
  double          no_cone_timeout_;

  ros::Subscriber subscriber_optimizer;
  void            callbackOptimizer(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  bool            got_optimizer = false;
  std::mutex      mutex_optimizer;

  geometry_msgs::PoseWithCovarianceStamped optimizer;

  // | ------------------------- timers ------------------------- |

  ros::Timer main_timer;
  double     _main_timer_rate_;
  void       mainTimer(const ros::TimerEvent& event);

  bool kalman_initialized = false;

  // | --------------------- service clients -------------------- |

  bool resetOptimizer();
  bool zigzaggerActivation();
  bool localizerActivation(const bool in);

  // | ------------------------ 2d kalman ----------------------- |

  Eigen::MatrixXd initial_covariance_2D_;

  statecov_2d_t statecov_2d_;

  A_2d_t A_2d_;
  R_2d_t R_2d_;
  Q_2d_t Q_2d_;
  H_2d_t H_2d_;
  B_2d_t B_2d_;

  std::unique_ptr<lkf_2d_t> lkf_2d_;

  // | ------------------------ 3d kalman ----------------------- |

  Eigen::MatrixXd initial_state_3D_;

  Eigen::MatrixXd initial_covariance_3D_;

  statecov_3d_t statecov_3d_;

  A_3d_t A_3d_;
  R_3d_t R_3d_;
  Q_3d_t Q_3d_;
  H_3d_t H_3d_;
  B_3d_t B_3d_;

  std::unique_ptr<lkf_3d_t> lkf_3d_;

  double _max_projection_error_;
  int    _n_projection_error_;
  int    projection_errored_ = 0;

  // | ------------------- dynamic reconfigure ------------------ |

  double q_2D_, r_2D_;
  double q_3D_, r_3D_;

  boost::recursive_mutex                              config_mutex_;
  typedef compton_camera_filter::compton_filterConfig Config;
  typedef dynamic_reconfigure::Server<Config>         ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                reconfigure_server_;
  void                                                drs_callback(compton_camera_filter::compton_filterConfig& config, uint32_t level);
  compton_camera_filter::compton_filterConfig         drs_compton_filter;

  void       callbackDrs(compton_camera_filter::compton_filterConfig& config, uint32_t level);
  std::mutex mutex_drs;

  std::atomic<bool> sweeping_ = true;

  // | ----------------------- roroutines ----------------------- |

  void                           initializeKalmans(const double x, const double y, const double z);
  std::optional<Eigen::Vector3d> projectPointOnCone(mrs_lib::geometry::Cone& cone, const Eigen::Vector3d& point);
};
//}

/* inInit() //{ */

void ComptonFilter::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ComptonFilter]: initializing");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "ComptonFilter");

  param_loader.loadParam("uav_name", uav_name_);
  param_loader.loadParam("world_frame", _world_frame_);

  param_loader.loadParam("main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("no_cone_timeout", no_cone_timeout_);

  param_loader.loadParam("kalman_2D/r", r_2D_);
  param_loader.loadParam("kalman_2D/q", q_2D_);

  param_loader.loadMatrixStatic("kalman_2D/A", A_2d_);
  param_loader.loadMatrixStatic("kalman_2D/B", B_2d_);
  param_loader.loadMatrixStatic("kalman_2D/R", R_2d_);
  param_loader.loadMatrixStatic("kalman_2D/H", H_2d_);
  param_loader.loadMatrixStatic("kalman_2D/Q", Q_2d_);

  param_loader.loadMatrixDynamic("kalman_2D/initial_covariance", initial_covariance_2D_, _2d_n_states_, _2d_n_states_);

  param_loader.loadMatrixStatic("kalman_3D/A", A_3d_);
  param_loader.loadMatrixStatic("kalman_3D/B", B_3d_);
  param_loader.loadMatrixStatic("kalman_3D/R", R_3d_);
  param_loader.loadMatrixStatic("kalman_3D/H", H_3d_);
  param_loader.loadMatrixStatic("kalman_3D/Q", Q_3d_);

  param_loader.loadMatrixDynamic("kalman_3D/initial_states", initial_state_3D_, _3d_n_states_, 1);

  param_loader.loadParam("kalman_3D/r", r_3D_);
  param_loader.loadParam("kalman_3D/q", q_3D_);

  param_loader.loadParam("kalman_3D/max_projection_error", _max_projection_error_);
  param_loader.loadParam("kalman_3D/n_projection_error", _n_projection_error_);

  param_loader.loadMatrixDynamic("kalman_3D/initial_covariance", initial_covariance_3D_, _3d_n_states_, _3d_n_states_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ComptonFilter]: Could not load all parameters!");
    ros::shutdown();
  }

  // | -------------------- batch visualizer -------------------- |

  batch_visualizer_ = mrs_lib::BatchVisualizer(nh_, "compton_filter", _world_frame_);

  batch_visualizer_.clearBuffers();
  batch_visualizer_.clearVisuals();

  batch_visualizer_2d_hypo_ = mrs_lib::BatchVisualizer(nh_, "hypothesis_2d", _world_frame_);

  batch_visualizer_2d_hypo_.clearBuffers();
  batch_visualizer_2d_hypo_.clearVisuals();

  batch_visualizer_3d_hypo_ = mrs_lib::BatchVisualizer(nh_, "hypothesis_3d", _world_frame_);

  batch_visualizer_3d_hypo_.clearBuffers();
  batch_visualizer_3d_hypo_.clearVisuals();

  // | ----------------------- subscribers ---------------------- |

  subscriber_cone      = nh_.subscribe("cone_in", 1, &ComptonFilter::callbackCone, this, ros::TransportHints().tcpNoDelay());
  subscriber_optimizer = nh_.subscribe("optimizer_in", 1, &ComptonFilter::callbackOptimizer, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_2d_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_2D_out", 1);
  publisher_3d_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_3D_out", 1);

  publisher_2d_correction_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("correction_2d_out", 1);
  publisher_3d_correction_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("correction_3d_out", 1);

  // | --------------------- service clients -------------------- |

  service_client_sweeper_      = nh_.serviceClient<std_srvs::SetBool>("sweeper_out");
  service_client_localization_ = nh_.serviceClient<std_srvs::SetBool>("localization_out");

  service_client_optimizer_reset_ = nh_.serviceClient<std_srvs::Trigger>("reset_out");

  // | ------------------------- timers ------------------------- |

  main_timer = nh_.createTimer(ros::Rate(_main_timer_rate_), &ComptonFilter::mainTimer, this);

  // | --------------------- kalman filters --------------------- |

  lkf_2d_ = std::make_unique<lkf_2d_t>(A_2d_, B_2d_, H_2d_);
  lkf_3d_ = std::make_unique<lkf_3d_t>(A_3d_, B_3d_, H_3d_);

  Eigen::Vector3d ground_point;
  ground_point << 0, 0, 0;

  Eigen::Vector3d ground_normal;
  ground_normal << 0, 0, 1;

  // | ------------------- dynamic reconfigure ------------------ |

  drs_compton_filter.q_2D = q_2D_;
  drs_compton_filter.r_2D = r_2D_;
  drs_compton_filter.q_3D = q_3D_;
  drs_compton_filter.r_3D = r_3D_;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_compton_filter);
  ReconfigureServer::CallbackType f = boost::bind(&ComptonFilter::callbackDrs, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  /* initializeKalmans(5, -5, 5); */

  is_initialized = true;

  ROS_INFO("[ComptonFilter]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackCone() //{ */

void ComptonFilter::callbackCone(const rad_msgs::ConeConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[ComptonFilter]: getting cones");

  if (!kalman_initialized) {
    return;
  }

  ROS_INFO("[ComptonFilter]: cone received");

  Eigen::Vector3d cone_position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Vector3d cone_direction(msg->direction.x, msg->direction.y, msg->direction.z);
  cone_direction.normalize();

  mrs_lib::geometry::Cone cone = mrs_lib::geometry::Cone(cone_position, msg->angle, 50, cone_direction);

  // | --------------------------- 3D --------------------------- |
  Eigen::Vector3d state_3D(statecov_3d_.x[0], statecov_3d_.x[1], statecov_3d_.x[2]);
  ROS_INFO_STREAM("[ComptonFilter]: state_3D = " << state_3D);

  batch_visualizer_.clearBuffers();
  batch_visualizer_.clearVisuals();

  auto            result3d = projectPointOnCone(cone, state_3D);
  Eigen::Vector3d projection;

  Eigen::Vector3d e1(1, 0, 0);

  if (result3d.has_value()) {

    projection = result3d.value();

    ROS_INFO_STREAM("[ComptonFilter]: projection = " << projection);
    ROS_INFO_STREAM("[ComptonFilter]: cone_position = " << cone_position);

    /* Eigen::Vector3d dir_to_proj = projection - cone_position; */
    Eigen::Vector3d dir_to_proj = projection - state_3D;
    ROS_INFO_STREAM("[ComptonFilter]: dir_to_proj = " << dir_to_proj);

    // calculate the angular size of the projection distance
    double proj_ang_size = mrs_lib::geometry::angleBetween(Eigen::Vector3d(state_3D - cone_position), Eigen::Vector3d(projection - cone_position));

    ROS_INFO("[ComptonFilter]: proj_ang_size %.2f deg", (proj_ang_size / M_PI) * 180.0);

    if (fabs(proj_ang_size) >= _max_projection_error_) {
      projection_errored_++;
      ROS_WARN("[ComptonFilter]: projection_errorred_++ = %d", projection_errored_);
      return;
    } else {
      projection_errored_ = 0;
      ROS_INFO("[ComptonFilter]: projection_errorred_ reset");
    }

    if (projection_errored_ >= _n_projection_error_) {

      ROS_WARN("[ComptonFilter]: angular error too large for more than #%d times", projection_errored_);

      std::scoped_lock lock(mutex_optimizer);

      Eigen::MatrixXd new_cov3 = Eigen::MatrixXd::Zero(_3d_n_states_, _3d_n_states_);
      new_cov3 << optimizer.pose.covariance[0] + 1.0, 0, 0, 0, optimizer.pose.covariance[7] + 1.0, 0, 0, 0, optimizer.pose.covariance[14] + 1.0;

      statecov_3d_.P = new_cov3;

      ROS_INFO("[ComptonFilter]: starting sweeping");

      resetOptimizer();
      localizerActivation(false);
      ros::Duration(1.0).sleep();
      zigzaggerActivation();

      kalman_initialized = false;
    }

    // check of the cone points to the sky
    {
      tf2::Vector3 cone_axis(msg->direction.x, msg->direction.y, msg->direction.z);

      double cone_axis_tilt = acos(cone_axis.dot(tf2::Vector3(0, 0, 1)));

      double cone_lowest_tilt = cone_axis_tilt + msg->angle;

      if (cone_lowest_tilt < M_PI / 2.0) {
        ROS_WARN("[ComptonFilter]: rejecting cone, it points to the sky");
        return;
      }
    }

    {
      std::scoped_lock lock(mutex_cone_last_time);

      cone_last_time_ = ros::Time::now();
    }

    // construct the covariance rotation
    double          angle = mrs_lib::geometry::angleBetween(dir_to_proj, e1);
    Eigen::Vector3d axis  = e1.cross(dir_to_proj);
    Eigen::Matrix3d rot   = mrs_lib::AttitudeConverter(Eigen::AngleAxis<double>(angle, axis));

    // rotate the covariance
    Eigen::Matrix3d R_3d_rotated_ = rot * R_3d_ * rot.transpose();

    {  // visualize

      geometry_msgs::PoseWithCovarianceStamped pose_out;

      pose_out.header.stamp         = ros::Time::now();
      pose_out.header.frame_id      = _world_frame_;
      pose_out.pose.pose.position.x = projection[0];
      pose_out.pose.pose.position.y = projection[1];
      pose_out.pose.pose.position.z = projection[2];

      pose_out.pose.covariance[0]  = R_3d_rotated_(0, 0);
      pose_out.pose.covariance[1]  = R_3d_rotated_(0, 1);
      pose_out.pose.covariance[2]  = R_3d_rotated_(0, 2);
      pose_out.pose.covariance[6]  = R_3d_rotated_(1, 0);
      pose_out.pose.covariance[7]  = R_3d_rotated_(1, 1);
      pose_out.pose.covariance[8]  = R_3d_rotated_(1, 2);
      pose_out.pose.covariance[12] = R_3d_rotated_(2, 0);
      pose_out.pose.covariance[13] = R_3d_rotated_(2, 1);
      pose_out.pose.covariance[14] = R_3d_rotated_(2, 2);

      try {
        publisher_3d_correction_.publish(pose_out);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_3d_correction_.getTopic().c_str());
      }
    }

    ROS_INFO_STREAM("[ComptonFilter]: r_3d_rotated: " << R_3d_rotated_);

    ROS_INFO_STREAM("[ComptonFilter]: 3d before correction = " << statecov_3d_.x.transpose() << ", measurement = " << projection.transpose());

    try {
      statecov_3d_ = lkf_3d_->correct(statecov_3d_, projection, R_3d_rotated_);
    }
    catch (...) {
      ROS_ERROR("exception caught during 3d correction");
    }

    /* statecov_3d_.P << 1, 0, 0, 0, 1, 0, 0, 0, 1; */

    ROS_INFO_STREAM("[ComptonFilter]: 3d after correction = " << statecov_3d_.x.transpose());

  } else {
    ROS_ERROR("[ComptonFilter]: could not do 3D projection!");
  }

  // | --------------------------- 2D --------------------------- |

  Eigen::Vector3d state_2D(statecov_2d_.x[0], statecov_2d_.x[1], 0);
  auto            result2d = projectPointOnCone(cone, state_2D);

  Eigen::Vector3d projection_2D;

  if (result2d) {

    projection_2D = result2d.value();

    // project it down to the ground
    projection_2D(2) = 0;

    Eigen::Vector3d dir_to_proj = projection_2D - state_2D;

    // construct the covariance rotation
    double          angle = mrs_lib::geometry::angleBetween(dir_to_proj, e1);
    Eigen::Vector3d axis  = e1.cross(dir_to_proj);
    Eigen::Matrix3d rot   = mrs_lib::AttitudeConverter(Eigen::AngleAxis<double>(angle, axis));

    // rotate the covariance
    Eigen::Matrix3d R_2D_in_3D   = Eigen::MatrixXd::Zero(3, 3);
    R_2D_in_3D.block(0, 0, 2, 2) = R_2d_;
    R_2D_in_3D(2, 2)             = R_2D_in_3D(1, 1);

    /* ROS_INFO_STREAM("[ComptonFilter]: Q_2D_in_3D:" << Q_2D_in_3D); */
    Eigen::Matrix3d R_3d_rotated_ = rot * R_2D_in_3D * rot.transpose();
    Eigen::Matrix2d cov_2D        = R_3d_rotated_.block(0, 0, 2, 2);

    /* cov_2D << q_2D_, 0, */
    /*           0, q_2D_; */

    Eigen::Vector2d measurement(projection_2D(0), projection_2D(1));

    /* ROS_INFO("[ComptonFilter]: state %f %f", state_2D(0), state_2D(1)); */
    /* ROS_INFO("[ComptonFilter]: measurement %f %f", measurement(0), measurement(1)); */

    {  // visualize
      geometry_msgs::PoseWithCovarianceStamped pose_out;

      pose_out.header.stamp         = ros::Time::now();
      pose_out.header.frame_id      = _world_frame_;
      pose_out.pose.pose.position.x = measurement[0];
      pose_out.pose.pose.position.y = measurement[1];
      pose_out.pose.pose.position.z = 0;

      pose_out.pose.covariance[0] = cov_2D(0, 0);
      pose_out.pose.covariance[1] = cov_2D(0, 1);
      pose_out.pose.covariance[2] = 0;
      pose_out.pose.covariance[6] = cov_2D(1, 0);
      pose_out.pose.covariance[7] = cov_2D(1, 1);
      pose_out.pose.covariance[8] = 0;

      try {
        publisher_2d_correction_.publish(pose_out);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_2d_correction_.getTopic().c_str());
      }
    }

    try {
      statecov_2d_ = lkf_2d_->correct(statecov_2d_, measurement, cov_2D);
    }
    catch (...) {
      ROS_ERROR("exception caught during 2d correction");
    }
  }

  batch_visualizer_.publish();
}

//}

/* callbackOptimizer() //{ */

void ComptonFilter::callbackOptimizer(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_optimizer);

  got_optimizer = true;

  optimizer = *msg;

  if (!kalman_initialized) {

    ROS_INFO("[ComptonFilter]: initializing KF");

    Eigen::Vector2d new_state2;
    new_state2 << msg->pose.pose.position.x, msg->pose.pose.position.y;

    Eigen::MatrixXd new_cov2 = Eigen::MatrixXd::Zero(_2d_n_states_, _2d_n_states_);
    new_cov2 << msg->pose.covariance[0] + 1.0, 0, 0, msg->pose.covariance[7] + 1.0;
    ROS_INFO_STREAM("[ComptonFilter]: new_cov2 = " << new_cov2);

    statecov_2d_.x = new_state2;
    statecov_2d_.P = new_cov2;

    Eigen::Vector3d new_state3;
    new_state3 << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

    Eigen::MatrixXd new_cov3 = Eigen::MatrixXd::Zero(_3d_n_states_, _3d_n_states_);
    new_cov3 << msg->pose.covariance[0] + 1.0, 0, 0, 0, msg->pose.covariance[7] + 1.0, 0, 0, 0, msg->pose.covariance[14] + 1.0;

    ROS_INFO_STREAM("[ComptonFilter]: new_cov3 = " << new_cov3);

    statecov_3d_.x = new_state3;
    statecov_3d_.P = new_cov3;

    kalman_initialized = true;

    localizerActivation(true);
  }
}

//}

/* callbackDrs() //{ */

void ComptonFilter::callbackDrs(compton_camera_filter::compton_filterConfig& config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs);

    q_2D_ = config.q_2D;
    r_2D_ = config.r_2D;

    q_3D_ = config.q_3D;
    r_3D_ = config.r_3D;
  }

  Q_3d_(0, 0) = q_3D_;
  Q_3d_(1, 1) = q_3D_;
  Q_3d_(2, 2) = q_3D_;
  R_3d_(0, 0) = r_3D_;

  Q_2d_(0, 0) = q_2D_;
  Q_2d_(1, 1) = q_2D_;
  R_2d_(0, 0) = r_2D_;

  ROS_INFO("[ComptonFilter]: updated covariances");
}

//}

// | ------------------------- timers ------------------------- |

/* mainTimer() //{ */

void ComptonFilter::mainTimer([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized)
    return;

  if (!kalman_initialized) {
    return;
  }

  ROS_INFO_THROTTLE(1.0, "[ComptonFilter]: main timer spinning");

  statecov_2d_ = lkf_2d_->predict(statecov_2d_, Eigen::VectorXd::Zero(_2d_n_inputs_), Q_2d_, 1.0 / _main_timer_rate_);
  statecov_3d_ = lkf_3d_->predict(statecov_3d_, Eigen::VectorXd::Zero(_3d_n_inputs_), Q_3d_, 1.0 / _main_timer_rate_);

  batch_visualizer_2d_hypo_.clearBuffers();
  batch_visualizer_2d_hypo_.clearVisuals();

  batch_visualizer_3d_hypo_.clearBuffers();
  batch_visualizer_3d_hypo_.clearVisuals();

  // | ------------------------ 2D kalman ----------------------- |

  {

    geometry_msgs::PoseWithCovarianceStamped pose_out;

    pose_out.header.stamp         = ros::Time::now();
    pose_out.header.frame_id      = _world_frame_;
    pose_out.pose.pose.position.x = statecov_2d_.x[0];
    pose_out.pose.pose.position.y = statecov_2d_.x[1];
    pose_out.pose.pose.position.z = 0;

    Eigen::MatrixXd covariance = statecov_2d_.P;

    pose_out.pose.covariance[0] = covariance(0, 0);
    pose_out.pose.covariance[1] = covariance(0, 1);
    pose_out.pose.covariance[2] = 0;
    pose_out.pose.covariance[6] = covariance(1, 0);
    pose_out.pose.covariance[7] = covariance(1, 1);
    pose_out.pose.covariance[8] = 0;

    try {
      publisher_2d_.publish(pose_out);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_2d_.getTopic().c_str());
    }

    Eigen::Vector3d cuboid_center(statecov_2d_.x[0], statecov_2d_.x[1], 0);
    Eigen::Vector3d cuboid_size(1.0, 1.0, 1.0);

    mrs_lib::geometry::Cuboid cuboid(cuboid_center, cuboid_size, mrs_lib::AttitudeConverter(0, 0, 0));

    batch_visualizer_2d_hypo_.addCuboid(cuboid, 1.0, 0.0, 0.0, 1.0, true);
    batch_visualizer_2d_hypo_.addCuboid(cuboid, 0.0, 0.0, 0.0, 1.0, false);
  }

  // | ------------------------ 3D kalman ----------------------- |

  {

    geometry_msgs::PoseWithCovarianceStamped pose_out;

    pose_out.header.stamp         = ros::Time::now();
    pose_out.header.frame_id      = _world_frame_;
    pose_out.pose.pose.position.x = statecov_3d_.x[0];
    pose_out.pose.pose.position.y = statecov_3d_.x[1];
    pose_out.pose.pose.position.z = statecov_3d_.x[2];

    Eigen::MatrixXd covariance = statecov_3d_.P;

    pose_out.pose.covariance[0]  = covariance(0, 0);
    pose_out.pose.covariance[1]  = covariance(0, 1);
    pose_out.pose.covariance[2]  = covariance(0, 2);
    pose_out.pose.covariance[6]  = covariance(1, 0);
    pose_out.pose.covariance[7]  = covariance(1, 1);
    pose_out.pose.covariance[8]  = covariance(1, 2);
    pose_out.pose.covariance[12] = covariance(2, 0);
    pose_out.pose.covariance[13] = covariance(2, 1);
    pose_out.pose.covariance[14] = covariance(2, 2);

    try {
      publisher_3d_.publish(pose_out);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_3d_.getTopic().c_str());
    }

    Eigen::Vector3d cuboid_center(statecov_3d_.x[0], statecov_3d_.x[1], statecov_3d_.x[2]);
    Eigen::Vector3d cuboid_size(1.0, 1.0, 1.0);

    mrs_lib::geometry::Cuboid cuboid(cuboid_center, cuboid_size, mrs_lib::AttitudeConverter(0, 0, 0));

    batch_visualizer_3d_hypo_.addCuboid(cuboid, 1.0, 0.0, 0.0, 1.0, true);
    batch_visualizer_3d_hypo_.addCuboid(cuboid, 0.0, 0.0, 0.0, 1.0, false);
  }

  batch_visualizer_2d_hypo_.publish();
  batch_visualizer_3d_hypo_.publish();

  {
    std::scoped_lock lock(mutex_cone_last_time);

    if (!sweeping_ && (ros::Time::now() - cone_last_time_).toSec() > no_cone_timeout_) {

      ROS_INFO("[ComptonFilter]: no cones arrived for more than %.2f s, last time %f s", no_cone_timeout_, cone_last_time_.toSec());

      std_srvs::Trigger search_out;
      service_client_optimizer_reset_.call(search_out);

      ROS_INFO("[ComptonFilter]: starting sweeping");

      resetOptimizer();
      localizerActivation(false);
      ros::Duration(1.0).sleep();
      zigzaggerActivation();

      kalman_initialized = false;
    }
  }
}

//}

// | ------------------------ routines ------------------------ |

/* initializeKalmans() //{ */

void ComptonFilter::initializeKalmans(const double x, const double y, const double z) {

  Eigen::Vector2d new_state2;
  new_state2 << x, y;

  Eigen::MatrixXd new_cov2 = Eigen::MatrixXd::Zero(_2d_n_states_, _2d_n_states_);
  new_cov2 << 100.0, 0, 0, 100.0;
  ROS_INFO_STREAM("[ComptonFilter]: new_cov2 = " << new_cov2);

  statecov_2d_.x = new_state2;
  statecov_2d_.P = new_cov2;

  Eigen::Vector3d new_state3;
  new_state3 << x, y, z;

  Eigen::MatrixXd new_cov3 = Eigen::MatrixXd::Zero(_3d_n_states_, _3d_n_states_);
  new_cov3 << 100.0, 0, 0, 0, 100.0, 0, 0, 0, 100.0;

  ROS_INFO_STREAM("[ComptonFilter]: new_cov3 = " << new_cov3);

  statecov_3d_.x = new_state3;
  statecov_3d_.P = new_cov3;

  localizerActivation(true);

  kalman_initialized = true;
}

//}

/* projectPointOnCone() //{ */

std::optional<Eigen::Vector3d> ComptonFilter::projectPointOnCone(mrs_lib::geometry::Cone& cone, const Eigen::Vector3d& point) {

  Eigen::Vector3d point_vec        = point - cone.origin();
  double          point_axis_angle = mrs_lib::geometry::angleBetween(point_vec, cone.direction());

  /* Eigen::Vector3d axis_projection = this->cone_axis_projector * point_vec + cone.origin(); */

  Eigen::Vector3d axis_rot = cone.direction().cross(point_vec);
  axis_rot.normalize();

  Eigen::AngleAxis<double> my_quat(cone.theta() - point_axis_angle, axis_rot);

  Eigen::Vector3d point_on_cone = my_quat * point_vec + cone.origin();

  {  // visualize

    batch_visualizer_.addPoint(point_on_cone, 1.0, 0.0, 0.0, 1.0);

    mrs_lib::geometry::Ray ray2(cone.origin(), point);
    batch_visualizer_.addRay(ray2, 0.0, 1.0, 0.0, 1.0);
  }

  Eigen::Vector3d vec_point_on_cone = point_on_cone - cone.origin();
  vec_point_on_cone.normalize();

  double beta = cone.theta() - point_axis_angle;

  if (point_axis_angle < cone.theta()) {

    Eigen::Vector3d projection = cone.origin() + vec_point_on_cone * cos(beta) * point_vec.norm();

    mrs_lib::geometry::Ray ray(point, projection);
    batch_visualizer_.addRay(ray, 1.0, 0.0, 0.0, 1.0);

    mrs_lib::geometry::Cuboid cuboid2(projection + vec_point_on_cone * 0.0, Eigen::Vector3d(0.3, 0.3, 0.3), mrs_lib::AttitudeConverter(0, 0, 0));
    batch_visualizer_.addCuboid(cuboid2, 1.0, 0.0, 0.0, 1.0);

    mrs_lib::geometry::Cuboid cuboid(projection, Eigen::Vector3d(0.3, 0.3, 0.3), mrs_lib::AttitudeConverter(0, 0, 0));
    batch_visualizer_.addCuboid(cuboid, 0.0, 0.0, 1.0, 1.0);

    /* return projection; */
    return projection + vec_point_on_cone * 0.0;

  } else if ((point_axis_angle >= cone.theta()) && (point_axis_angle - cone.theta()) <= M_PI / 2.0) {  // TODO: is this condition correct?

    Eigen::Vector3d projection = cone.origin() + vec_point_on_cone * cos(point_axis_angle - cone.theta()) * point_vec.norm();

    mrs_lib::geometry::Ray ray(point, projection);
    batch_visualizer_.addRay(ray, 1.0, 0.0, 0.0, 1.0);

    mrs_lib::geometry::Cuboid cuboid(projection, Eigen::Vector3d(0.3, 0.3, 0.3), mrs_lib::AttitudeConverter(0, 0, 0));
    batch_visualizer_.addCuboid(cuboid, 0.0, 0.0, 1.0, 1.0);

    mrs_lib::geometry::Cuboid cuboid2(projection + vec_point_on_cone * 0.0, Eigen::Vector3d(0.3, 0.3, 0.3), mrs_lib::AttitudeConverter(0, 0, 0));
    batch_visualizer_.addCuboid(cuboid2, 1.0, 0.0, 0.0, 1.0);

    /* return projection; */
    return projection + vec_point_on_cone * 0.0;

  } else {

    return cone.origin();
  }
}

//}

/* sweeperActivation() //{ */

bool ComptonFilter::zigzaggerActivation() {

  sweeping_ = true;

  std_srvs::SetBool srv;
  srv.request.data = 1;

  bool res = service_client_sweeper_.call(srv);

  if (!res) {
    ROS_ERROR("[ComptonFilter]: failed to call service to zigzagger");
    return false;
  } else {
    if (!srv.response.success) {
      ROS_ERROR("[ComptonFilter]: failed to call service to zigzagger: '%s'", srv.response.message.c_str());
      return false;
    }
  }

  return true;
}

//}

/* localizerActivation() //{ */

bool ComptonFilter::localizerActivation(const bool in) {

  if (in) {
    {
      std::scoped_lock lock(mutex_cone_last_time);

      cone_last_time_ = ros::Time::now();
    }
    sweeping_       = false;
  }

  std_srvs::SetBool srv;
  srv.request.data = in;

  bool res = service_client_localization_.call(srv);

  if (!res) {
    ROS_ERROR("[ComptonFilter]: failed to call service to localization");
    return false;
  } else {
    if (!srv.response.success) {
      ROS_ERROR("[ComptonFilter]: failed to call service to localization: '%s'", srv.response.message.c_str());
      return false;
    }
  }

  return true;
}

//}

/* resetOptimizer() //{ */

bool ComptonFilter::resetOptimizer() {

  std_srvs::Trigger srv;

  bool res = service_client_optimizer_reset_.call(srv);

  if (!res) {
    ROS_ERROR("[ComptonFilter]: failed to call service to optimizer");
    return false;
  } else {
    if (!srv.response.success) {
      ROS_ERROR("[ComptonFilter]: failed to call service to optimizer: '%s'", srv.response.message.c_str());
      return false;
    }
  }

  return true;
}

//}

}  // namespace compton_camera_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_camera_filter::ComptonFilter, nodelet::Nodelet)
