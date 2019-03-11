#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <gazebo_rad_msgs/Cone.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Eigen>
#include <mutex>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Lkf.h>

#include <mrs_lib/GeometryUtils.h>

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

  private:
    ros::Subscriber subscriber_cone;
    void            callbackCone(const gazebo_rad_msgs::ConeConstPtr &msg);

  private:
    ros::Timer main_timer;
    int        main_timer_rate_;
    void       mainTimer(const ros::TimerEvent &event);

  private:
    Eigen::MatrixXd A_2D_, B_2D_, P_2D_, Q_2D_, R_2D_;
    double n_states_2D_, n_inputs_2D_, n_measurements_2D_;
    mrs_lib::Lkf *lkf_2D;
    std::mutex    mutex_lkf_2D;

    Eigen::MatrixXd initial_covariance_2D_;

  private:
    Eigen::MatrixXd A_3D_, B_3D_, P_3D_, Q_3D_, R_3D_;
    double n_states_3D_, n_inputs_3D_, n_measurements_3D_;
    mrs_lib::Lkf *lkf_3D;
    std::mutex    mutex_lkf_3D;

    Eigen::MatrixXd initial_covariance_3D_;

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

    param_loader.load_matrix_dynamic("kalman_3D/initial_covariance", initial_covariance_3D_, n_states_3D_, n_states_3D_);

    // --------------------------------------------------------------
    // |                         subscribers                        |
    // --------------------------------------------------------------

    subscriber_cone = nh_.subscribe("cone_in", 1, &ComptonFilter::callbackCone, this, ros::TransportHints().tcpNoDelay());

    // --------------------------------------------------------------
    // |                         publishers                         |
    // --------------------------------------------------------------

    publisher_pose_2D = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_2D_out", 1);
    publisher_pose_3D = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_3D_out", 1);

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

    Eigen::Vector3d ground_point;
    ground_point << 0, 0, 0;

    Eigen::Vector3d ground_normal;
    ground_normal << 0, 0, 1;

    ground_plane = mrs_lib::Plane(ground_point, ground_normal);

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

    ROS_INFO_ONCE("[ComptonFilter]: getting cones");

    std::scoped_lock lock(mutex_lkf_3D);

    Eigen::Vector3d position;
    position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    Eigen::Vector3d direction;
    direction << msg->direction.x, msg->direction.y, msg->direction.z;
    direction.normalize();

    mrs_lib::Cone cone = mrs_lib::Cone(position, direction, msg->angle);

    Eigen::Vector3d state;
    state << lkf_3D->getState(0), lkf_3D->getState(1), lkf_3D->getState(2);

    // 3D
    Eigen::Vector3d projection = cone.ProjectPoint(state);

    Eigen::Vector3d unit;
    unit << 1, 0, 0;
    Eigen::Vector3d dir_to_proj = projection - state;
    double proj_len = dir_to_proj.norm();

    double angle = acos((dir_to_proj.dot(unit))/(dir_to_proj.norm()*unit.norm()));
    Eigen::Vector3d axis = unit.cross(dir_to_proj);
    Eigen::AngleAxis<double> my_quat(angle, axis);

    Eigen::Matrix3d rot = my_quat.toRotationMatrix();

    Q_3D_ << proj_len*proj_len, 0, 0,
             0, 1e10, 0,
             0, 0, 1e10;

    Eigen::Matrix3d rot_cov = rot*Q_3D_*rot.transpose();

    ROS_INFO_STREAM("[ComptonFilter]: rotated_cov: " << rot_cov);

    /* lkf_3D->setP(rot); */
    lkf_3D->setMeasurement(projection, rot_cov);
    lkf_3D->doCorrection();

    // 2D

    /* state << lkf_2D->getState(0), lkf_2D->getState(1), lkf_2D->getState(2); */

    /* boost::optional<Eigen::Vector3d> projection_2d = cone.ProjectPointOnPlane(ground_plane, state); */

    /* if (!projection_2d) { */

    /*   ROS_INFO("[ComptonFilter]: not projection"); */
    /* } else { */

    /*   ROS_INFO("[ComptonFilter]: fusing 2D"); */
    /*   lkf_2D->setMeasurement(projection_2d.get(), Q_2D_); */
    /*   lkf_2D->doCorrection(); */
    /* } */

  }

  //}

  /* mainTimer() //{ */

  void ComptonFilter::mainTimer([[maybe_unused]] const ros::TimerEvent &event) {

    if (!is_initialized)
      return;

    // | ------------------------ 2D kalman ----------------------- |

    {

      std::scoped_lock lock(mutex_lkf_2D);

      lkf_2D->iterateWithoutCorrection();

      geometry_msgs::PoseWithCovarianceStamped pose_out;

      pose_out.header.stamp         = ros::Time::now();
      pose_out.header.frame_id      = "local_origin";
      pose_out.pose.pose.position.x = lkf_2D->getState(0);
      pose_out.pose.pose.position.y = lkf_2D->getState(1);
      pose_out.pose.pose.position.z = 0;

      Eigen::MatrixXd covariance = lkf_2D->getCovariance();

      pose_out.pose.covariance[0]  = covariance(0, 0);
      pose_out.pose.covariance[1]  = covariance(0, 1);
      pose_out.pose.covariance[2]  = 0;
      pose_out.pose.covariance[6]  = covariance(1, 0);
      pose_out.pose.covariance[7]  = covariance(1, 1);
      pose_out.pose.covariance[8]  = 0;

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

      lkf_3D->iterateWithoutCorrection();

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

}  // namespace compton_camera_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_camera_filter::ComptonFilter, nodelet::Nodelet)
