#pragma once
#include <pronto_core/sensing_module.hpp>
#include <pronto_core/visual_odometry_module.hpp>
#include <pronto_msgs/VisualOdometryUpdate.h>
#include <ros/node_handle.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Dense>

namespace pronto {

class VisualOdometryHandlerROS : public SensingModule<geometry_msgs::PoseWithCovarianceStamped>{
public:
  VisualOdometryHandlerROS(ros::NodeHandle& nh);

//  RBISUpdateInterface * processMessage(const pronto_msgs::VisualOdometryUpdate* msg,
//                                       StateEstimator* state_estimator);
  RBISUpdateInterface * processMessage(const geometry_msgs::PoseWithCovarianceStamped* msg,
                                       StateEstimator* state_estimator);

//  bool processMessageInit(const pronto_msgs::VisualOdometryUpdate *msg,
//                          const std::map<std::string, bool> &sensor_initialized,
//                          const RBIS &default_state,
//                          const RBIM &default_cov,
//                          RBIS &init_state,
//                          RBIM &init_cov);
  bool processMessageInit(const geometry_msgs::PoseWithCovarianceStamped *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov);
private:
  std::shared_ptr<VisualOdometryModule> vo_module_;
  VisualOdometryUpdate vo_update_;
  ros::Duration msg_time_offset_;
  PoseMeasurement pose_meas_;
  tf::Quaternion tf_q;
};

}
