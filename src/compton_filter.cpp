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

    ros::Publisher publisher_pose;

  private:
    ros::Subscriber subscriber_cone;
    void            callbackCone(const gazebo_rad_msgs::ConeConstPtr &msg);

  private:
    ros::Timer main_timer;
    int        main_timer_rate_;
    void       mainTimer(const ros::TimerEvent &event);

  private:
    Eigen::MatrixXd A_, B_, P_, Q_, R_;
    double          n_states_, n_inputs_, n_measurements_;
    mrs_lib::Lkf *  lkf;
    std::mutex      mutex_lkf;

    Eigen::MatrixXd initial_covariance_;

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

    param_loader.load_param("n_states", n_states_);
    param_loader.load_param("n_inputs", n_inputs_);
    param_loader.load_param("n_measurements", n_measurements_);

    param_loader.load_matrix_dynamic("A", A_, n_states_, n_states_);
    param_loader.load_matrix_dynamic("B", B_, n_states_, n_inputs_);
    param_loader.load_matrix_dynamic("R", R_, n_states_, n_states_);
    param_loader.load_matrix_dynamic("P", P_, n_measurements_, n_states_);
    param_loader.load_matrix_dynamic("Q", Q_, n_measurements_, n_measurements_);

    param_loader.load_matrix_dynamic("initial_covariance", initial_covariance_, n_states_, n_states_);

    // --------------------------------------------------------------
    // |                         subscribers                        |
    // --------------------------------------------------------------

    subscriber_cone = nh_.subscribe("cone_in", 1, &ComptonFilter::callbackCone, this, ros::TransportHints().tcpNoDelay());

    // --------------------------------------------------------------
    // |                         publishers                         |
    // --------------------------------------------------------------

    publisher_pose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_out", 1);

    // --------------------------------------------------------------
    // |                           timers                           |
    // --------------------------------------------------------------

    main_timer = nh_.createTimer(ros::Rate(main_timer_rate_), &ComptonFilter::mainTimer, this);

    // --------------------------------------------------------------
    // |                        Kalman filter                       |
    // --------------------------------------------------------------

    lkf = new mrs_lib::Lkf(n_states_, n_inputs_, n_measurements_, A_, B_, R_, Q_, P_);

    /* lkf->setState(0, 0); */
    /* lkf->setState(1, 0); */

    ground_plane = mrs_lib::Plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));

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

    std::scoped_lock lock(mutex_lkf);

    Eigen::Vector3d position;
    position << msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z;

    Eigen::Vector3d direction;
    direction << msg->direction.x,
                msg->direction.y,
                msg->direction.z;
    direction.normalize();

    mrs_lib::Cone cone = mrs_lib::Cone(position, direction, msg->angle);

    Eigen::Vector3d state;
    state << lkf->getState(0), lkf->getState(1), lkf->getState(2);

    /* boost::optional<Eigen::Vector3d> projection = cone.ProjectPointOnPlane(ground_plane, state); */
    Eigen::Vector3d projection = cone.ProjectPoint(state);

    /* if (!projection) { */

    /*   ROS_INFO("[ComptonFilter]: not projection"); */
    /*   return; */
    /* } */

    lkf->setMeasurement(projection, Q_);
    lkf->doCorrection();
  }

  //}

  /* mainTimer() //{ */

  void ComptonFilter::mainTimer([[maybe_unused]] const ros::TimerEvent &event) {

    if (!is_initialized)
      return;

    lkf->iterateWithoutCorrection();

    std::scoped_lock lock(mutex_lkf);

    geometry_msgs::PoseWithCovarianceStamped pose_out;

    pose_out.header.stamp         = ros::Time::now();
    pose_out.header.frame_id      = "local_origin";
    pose_out.pose.pose.position.x = lkf->getState(0);
    pose_out.pose.pose.position.y = lkf->getState(1);
    pose_out.pose.pose.position.z = lkf->getState(2);

    Eigen::MatrixXd covariance = lkf->getCovariance();

    pose_out.pose.covariance[0] = covariance(0, 0);
    pose_out.pose.covariance[1] = covariance(0, 1);
    pose_out.pose.covariance[2] = covariance(0, 2);
    pose_out.pose.covariance[6] = covariance(1, 0);
    pose_out.pose.covariance[7] = covariance(1, 1);
    pose_out.pose.covariance[8] = covariance(1, 2);
    pose_out.pose.covariance[12] = covariance(2, 0);
    pose_out.pose.covariance[13] = covariance(2, 1);
    pose_out.pose.covariance[14] = covariance(2, 2);

    try {
      publisher_pose.publish(pose_out);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_pose.getTopic().c_str());
    }
  }

  //}

}  // namespace compton_camera_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_camera_filter::ComptonFilter, nodelet::Nodelet)
