#include <ros/node_handle.h>
#include <pronto_quadruped_ros/stance_estimator_ros.hpp>
#include <pronto_quadruped_ros/leg_odometer_ros.hpp>
#include <sensor_msgs/JointState.h>

#include <simpledog_feet_contact_forces.hpp>
#include <simpledog_feet_jacobians.hpp>
#include <simpledog_forward_kinematics.hpp>

#include <pronto_quadruped_ros/bias_lock_handler_ros.hpp>
#include <pronto_quadruped_ros/legodo_handler_ros.hpp>

#include <pronto_ros/pronto_node.hpp>
#include "ros/ros.h"
#include <vector>
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64MultiArray.h>

class stateestimator{
public:
    stateestimator(const ros::NodeHandle& node_handle):node_handle_(node_handle){

    }

private:
    ros::NodeHandle node_handle_;
};

int main(int argc,char **argv){

}
