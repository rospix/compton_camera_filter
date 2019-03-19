#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <gazebo_rad_msgs/Cone.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Eigen>
#include <mutex>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Lkf.h>

#include <mrs_lib/GeometryUtils.h>

#include <compton_camera_filter/compton_filterConfig.h>

#include <std_srvs/Trigger.h>

namespace compton_camera_filter
{

  /* ComptonFilter //{ */
  class ComptonFilter : public nodelet::Nodelet {

  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh_;
    bool            is_initialized = false;

    ros::Publisher publisher_pose_2D;
    ros::Publisher publisher_pose_3D;

    ros::ServiceClient service_client_search;
    ros::ServiceClient service_client_reset;

  private:
    ros::Subscriber subscriber_cone;
    void            callbackCone(const gazebo_rad_msgs::ConeConstPtr &msg);

  private:
    ros::Subscriber subscriber_optimizer;
    void            callbackOptimizer(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    bool            got_optimizer = false;
    std::mutex      mutex_optimizer;

    geometry_msgs::PoseWithCovarianceStamped optimizer;

  private:
    ros::Timer main_timer;
    int        main_timer_rate_;
    void       mainTimer(const ros::TimerEvent &event);

  private:
    bool kalman_initialized = false;

    Eigen::MatrixXd A_2D_, B_2D_, P_2D_, Q_2D_, R_2D_;
    double          n_states_2D_, n_inputs_2D_, n_measurements_2D_;
    mrs_lib::Lkf *  lkf_2D;
    std::mutex      mutex_lkf_2D;

    Eigen::MatrixXd initial_covariance_2D_;

  private:
    Eigen::MatrixXd A_3D_, B_3D_, P_3D_, Q_3D_, R_3D_;
    double          n_states_3D_, n_inputs_3D_, n_measurements_3D_;
    mrs_lib::Lkf *  lkf_3D;
    std::mutex      mutex_lkf_3D;
    Eigen::MatrixXd initial_state_3D_;

    Eigen::MatrixXd initial_covariance_3D_;

  private:
    // --------------------------------------------------------------
    // |                     dynamic reconfigure                    |
    // --------------------------------------------------------------

    double q_2D_, r_2D_;
    double q_3D_, r_3D_;

    boost::recursive_mutex                              config_mutex_;
    typedef compton_camera_filter::compton_filterConfig Config;
    typedef dynamic_reconfigure::Server<Config>         ReconfigureServer;
    boost::shared_ptr<ReconfigureServer>                reconfigure_server_;
    void                                                drs_callback(compton_camera_filter::compton_filterConfig &config, uint32_t level);
    compton_camera_filter::compton_filterConfig         drs_compton_filter;

    void       dynamicReconfigureCallback(compton_camera_filter::compton_filterConfig &config, uint32_t level);
    std::mutex mutex_drs;

  private:
    mrs_lib::Plane ground_plane;
  };
  //}

  /* inInit() //{ */

  void ComptonFilter::onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ROS_INFO("[ComptonFilter]: initializing");

    ros::Time::waitForValid();

    mrs_lib::ParamLoader param_loader(nh_, "ComptonFilter");

    param_loader.load_param("main_timer_rate", main_timer_rate_);

    param_loader.load_param("kalman_2D/n_states", n_states_2D_);
    param_loader.load_param("kalman_2D/n_inputs", n_inputs_2D_);
    param_loader.load_param("kalman_2D/n_measurements", n_measurements_2D_);

    param_loader.load_param("kalman_2D/r", r_2D_);
    param_loader.load_param("kalman_2D/q", q_2D_);

    param_loader.load_matrix_dynamic("kalman_2D/A", A_2D_, n_states_2D_, n_states_2D_);
    param_loader.load_matrix_dynamic("kalman_2D/B", B_2D_, n_states_2D_, n_inputs_2D_);
    param_loader.load_matrix_dynamic("kalman_2D/R", R_2D_, n_states_2D_, n_states_2D_);
    param_loader.load_matrix_dynamic("kalman_2D/P", P_2D_, n_measurements_2D_, n_states_2D_);
    param_loader.load_matrix_dynamic("kalman_2D/Q", Q_2D_, n_measurements_2D_, n_measurements_2D_);

    param_loader.load_matrix_dynamic("kalman_2D/initial_covariance", initial_covariance_2D_, n_states_2D_, n_states_2D_);

    param_loader.load_param("kalman_3D/n_states", n_states_3D_);
    param_loader.load_param("kalman_3D/n_inputs", n_inputs_3D_);
    param_loader.load_param("kalman_3D/n_measurements", n_measurements_3D_);

    param_loader.load_matrix_dynamic("kalman_3D/A", A_3D_, n_states_3D_, n_states_3D_);
    param_loader.load_matrix_dynamic("kalman_3D/B", B_3D_, n_states_3D_, n_inputs_3D_);
    param_loader.load_matrix_dynamic("kalman_3D/R", R_3D_, n_states_3D_, n_states_3D_);
    param_loader.load_matrix_dynamic("kalman_3D/P", P_3D_, n_measurements_3D_, n_states_3D_);
    param_loader.load_matrix_dynamic("kalman_3D/Q", Q_3D_, n_measurements_3D_, n_measurements_3D_);

    param_loader.load_matrix_dynamic("kalman_3D/initial_states", initial_state_3D_, n_states_3D_, 1);

    param_loader.load_param("kalman_3D/r", r_3D_);
    param_loader.load_param("kalman_3D/q", q_3D_);

    param_loader.load_matrix_dynamic("kalman_3D/initial_covariance", initial_covariance_3D_, n_states_3D_, n_states_3D_);

    // --------------------------------------------------------------
    // |                         subscribers                        |
    // --------------------------------------------------------------

    subscriber_cone      = nh_.subscribe("cone_in", 1, &ComptonFilter::callbackCone, this, ros::TransportHints().tcpNoDelay());
    subscriber_optimizer = nh_.subscribe("optimizer_in", 1, &ComptonFilter::callbackOptimizer, this, ros::TransportHints().tcpNoDelay());

    // --------------------------------------------------------------
    // |                         publishers                         |
    // --------------------------------------------------------------

    publisher_pose_2D = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_2D_out", 1);
    publisher_pose_3D = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_3D_out", 1);

    // --------------------------------------------------------------
    // |                       service clients                      |
    // --------------------------------------------------------------

    service_client_search = nh_.serviceClient<std_srvs::Trigger>("search_out");
    service_client_reset = nh_.serviceClient<std_srvs::Trigger>("reset_out");

    // --------------------------------------------------------------
    // |                           timers                           |
    // --------------------------------------------------------------

    main_timer = nh_.createTimer(ros::Rate(main_timer_rate_), &ComptonFilter::mainTimer, this);

    // --------------------------------------------------------------
    // |                        Kalman filter                       |
    // --------------------------------------------------------------

    lkf_2D = new mrs_lib::Lkf(n_states_2D_, n_inputs_2D_, n_measurements_2D_, A_2D_, B_2D_, R_2D_, Q_2D_, P_2D_);
    lkf_2D->setCovariance(initial_covariance_2D_);

    lkf_3D = new mrs_lib::Lkf(n_states_3D_, n_inputs_3D_, n_measurements_3D_, A_3D_, B_3D_, R_3D_, Q_3D_, P_3D_);
    lkf_3D->setCovariance(initial_covariance_3D_);
    lkf_3D->setStates(initial_state_3D_);

    Eigen::Vector3d ground_point;
    ground_point << 0, 0, 0;

    Eigen::Vector3d ground_normal;
    ground_normal << 0, 0, 1;

    ground_plane = mrs_lib::Plane(ground_point, ground_normal);

    // --------------------------------------------------------------
    // |                     dynamic reconfigure                    |
    // --------------------------------------------------------------

    drs_compton_filter.q_2D = q_2D_;
    drs_compton_filter.r_2D = r_2D_;
    drs_compton_filter.q_3D = q_3D_;
    drs_compton_filter.r_3D = r_3D_;

    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
    reconfigure_server_->updateConfig(drs_compton_filter);
    ReconfigureServer::CallbackType f = boost::bind(&ComptonFilter::dynamicReconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    // | ----------------------- finish init ---------------------- |

    if (!param_loader.loaded_successfully()) {
      ROS_ERROR("[ComptonFilter]: Could not load all parameters!");
      ros::shutdown();
    }

    is_initialized = true;

    ROS_INFO("[ComptonFilter]: initialized");
  }

  //}

  // --------------------------------------------------------------
  // |                          callbacks                         |
  // --------------------------------------------------------------

  /* callbackCone() //{ */

  void ComptonFilter::callbackCone(const gazebo_rad_msgs::ConeConstPtr &msg) {

    if (!is_initialized)
      return;

    if (!kalman_initialized) {
      return;
    }

    ROS_INFO_ONCE("[ComptonFilter]: getting cones");

    std::scoped_lock lock(mutex_lkf_3D);

    Eigen::Vector3d cone_position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Vector3d cone_direction(msg->direction.x, msg->direction.y, msg->direction.z);
    cone_direction.normalize();

    mrs_lib::Cone cone = mrs_lib::Cone(cone_position, cone_direction, msg->angle);

    // | --------------------------- 3D --------------------------- |
    Eigen::Vector3d state_3D(lkf_3D->getState(0), lkf_3D->getState(1), lkf_3D->getState(2));
    Eigen::Vector3d projection = cone.ProjectPoint(state_3D);

    Eigen::Vector3d unit(1, 0, 0);
    Eigen::Vector3d dir_to_proj = projection - state_3D;

    if (dir_to_proj.norm() > 15.0) {

      std::scoped_lock lock(mutex_optimizer);

      Eigen::MatrixXd new_cov3 = Eigen::MatrixXd::Zero(n_states_3D_, n_states_3D_);
      new_cov3 << optimizer.pose.covariance[0], 0, 0, 0, optimizer.pose.covariance[7], 0, 0, 0, optimizer.pose.covariance[14];

      lkf_3D->setCovariance(new_cov3);

      std_srvs::Trigger search_out;
      service_client_reset.call(search_out);
      service_client_search.call(search_out);

      kalman_initialized = false;

      ROS_INFO("[ComptonFilter]: calling service for searching");
    }

    // construct the covariance rotation
    double                   angle = acos((dir_to_proj.dot(unit)) / (dir_to_proj.norm() * unit.norm()));
    Eigen::Vector3d          axis  = unit.cross(dir_to_proj);
    Eigen::AngleAxis<double> my_quat(angle, axis);
    Eigen::Matrix3d          rot = my_quat.toRotationMatrix();

    // rotate the covariance
    Eigen::Matrix3d rot_cov = rot * Q_3D_ * rot.transpose();

    lkf_3D->setMeasurement(projection, rot_cov);
    lkf_3D->iterate();

    // | --------------------------- 2D --------------------------- |

    Eigen::Vector3d state_2D(lkf_2D->getState(0), lkf_2D->getState(1), 0);
    Eigen::Vector3d projection_2D = cone.ProjectPoint(state_2D);

    // project it down to the ground
    projection_2D(2) = 0;

    dir_to_proj = projection_2D - state_2D;

    // construct the covariance rotation
    angle = acos((dir_to_proj.dot(unit)) / (dir_to_proj.norm() * unit.norm()));
    axis  = unit.cross(dir_to_proj);
    Eigen::AngleAxis<double> my_quat2(angle, axis);
    rot = my_quat2.toRotationMatrix();

    // rotate the covariance
    Eigen::Matrix3d Q_2D_in_3D   = Eigen::MatrixXd::Zero(3, 3);
    Q_2D_in_3D.block(0, 0, 2, 2) = Q_2D_;
    Q_2D_in_3D(2, 2)             = Q_2D_in_3D(1, 1);

    rot_cov                = rot * Q_2D_in_3D * rot.transpose();
    Eigen::Matrix2d cov_2D = rot_cov.block(0, 0, 2, 2);

    /* cov_2D << q_2D_, 0, */
    /*           0, q_2D_; */

    Eigen::Vector2d measurement(projection_2D(0), projection_2D(1));

    ROS_INFO("[ComptonFilter]: state %f %f", state_2D(0), state_2D(1));
    ROS_INFO("[ComptonFilter]: measurement %f %f", measurement(0), measurement(1));

    lkf_2D->setMeasurement(measurement, cov_2D);
    lkf_2D->iterate();
  }

  //}

  /* callbackOptimizer() //{ */

  void ComptonFilter::callbackOptimizer(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

    if (!is_initialized) {
      return;
    }

    std::scoped_lock lock(mutex_optimizer);

    got_optimizer = true;

    optimizer = *msg;

    if (!kalman_initialized) {

      ROS_INFO("[ComptonFilter]: initializing KF");

      Eigen::Vector2d new_state2;
      new_state2 << msg->pose.pose.position.x, msg->pose.pose.position.z;

      Eigen::MatrixXd new_cov2 = Eigen::MatrixXd::Zero(n_states_2D_, n_states_2D_);
      new_cov2 << msg->pose.covariance[0], 0, 0, msg->pose.covariance[7];

      lkf_2D->setStates(new_state2);
      lkf_2D->setCovariance(new_cov2);

      Eigen::Vector3d new_state3;
      new_state3 << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

      Eigen::MatrixXd new_cov3 = Eigen::MatrixXd::Zero(n_states_3D_, n_states_3D_);
      new_cov3 << msg->pose.covariance[0], 0, 0, 0, msg->pose.covariance[7], 0, 0, 0, msg->pose.covariance[14];

      lkf_3D->setStates(new_state3);
      lkf_3D->setCovariance(new_cov3);

      kalman_initialized = true;
    }
  }

  //}

  /* mainTimer() //{ */

  void ComptonFilter::mainTimer([[maybe_unused]] const ros::TimerEvent &event) {

    if (!is_initialized)
      return;

    if (!kalman_initialized) {
      return;
    }

    // | ------------------------ 2D kalman ----------------------- |

    {

      std::scoped_lock lock(mutex_lkf_2D);

      geometry_msgs::PoseWithCovarianceStamped pose_out;

      pose_out.header.stamp         = ros::Time::now();
      pose_out.header.frame_id      = "local_origin";
      pose_out.pose.pose.position.x = lkf_2D->getState(0);
      pose_out.pose.pose.position.y = lkf_2D->getState(1);
      pose_out.pose.pose.position.z = 0;

      Eigen::MatrixXd covariance = lkf_2D->getCovariance();

      pose_out.pose.covariance[0] = covariance(0, 0);
      pose_out.pose.covariance[1] = covariance(0, 1);
      pose_out.pose.covariance[2] = 0;
      pose_out.pose.covariance[6] = covariance(1, 0);
      pose_out.pose.covariance[7] = covariance(1, 1);
      pose_out.pose.covariance[8] = 0;

      try {
        publisher_pose_2D.publish(pose_out);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_pose_2D.getTopic().c_str());
      }
    }

    // | ------------------------ 3D kalman ----------------------- |

    {

      std::scoped_lock lock(mutex_lkf_3D);

      geometry_msgs::PoseWithCovarianceStamped pose_out;

      pose_out.header.stamp         = ros::Time::now();
      pose_out.header.frame_id      = "local_origin";
      pose_out.pose.pose.position.x = lkf_3D->getState(0);
      pose_out.pose.pose.position.y = lkf_3D->getState(1);
      pose_out.pose.pose.position.z = lkf_3D->getState(2);

      Eigen::MatrixXd covariance = lkf_3D->getCovariance();

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
        publisher_pose_3D.publish(pose_out);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_pose_3D.getTopic().c_str());
      }
    }
  }

  //}

  /* dynamicReconfigureCallback() //{ */

  void ComptonFilter::dynamicReconfigureCallback(compton_camera_filter::compton_filterConfig &config, [[maybe_unused]] uint32_t level) {

    {
      std::scoped_lock lock(mutex_drs);

      q_2D_ = config.q_2D;
      r_2D_ = config.r_2D;

      q_3D_ = config.q_3D;
      r_3D_ = config.r_3D;
    }

    {
      std::scoped_lock lock(mutex_lkf_3D);

      R_3D_(0, 0) = r_3D_;
      R_3D_(1, 1) = r_3D_;
      R_3D_(2, 2) = r_3D_;
      lkf_3D->setR(R_3D_);

      Q_3D_(0, 0) = q_3D_;
    }

    {
      std::scoped_lock lock(mutex_lkf_2D);

      R_2D_(0, 0) = r_2D_;
      R_2D_(1, 1) = r_2D_;
      lkf_2D->setR(R_2D_);

      Q_2D_(0, 0) = q_2D_;
    }

    ROS_INFO("[ComptonFilter]: updated covariances");
  }

  //}

}  // namespace compton_camera_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_camera_filter::ComptonFilter, nodelet::Nodelet)
