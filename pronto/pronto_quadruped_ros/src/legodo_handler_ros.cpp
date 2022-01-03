﻿/* Copyright (c) 2018-2019 University of Oxford
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
 *
 * This file is part of pronto_quadruped,
 * a library for leg odometry on quadruped robots.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "pronto_quadruped_ros/legodo_handler_ros.hpp"
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace pronto {
namespace quadruped {

LegodoHandlerBase::LegodoHandlerBase(ros::NodeHandle &nh,
                                     StanceEstimatorBase& stance_est,
                                     LegOdometerBase& legodo) :
    nh_(nh),
    stance_estimator_(stance_est),
    leg_odometer_(legodo)
{
    std::cout<<"!!!!!!!!!!!!!!!!!!"<<std::endl;
    std::cout<<"legodohanler"<<std::endl;
    std::cout<<"!!!!!!!!!!!!!!!!!!"<<std::endl;
    std::string prefix = "/state_estimator_pronto/legodo/";

    nh_.getParam(prefix + "downsample_factor", (int&)downsample_factor_);
    nh_.getParam(prefix + "utime_offset", (int&)utime_offset_);

    double r_kse_vx;
    double r_kse_vy;
    double r_kse_vz;
    nh_.getParam(prefix + "r_vx", r_kse_vx);
    nh_.getParam(prefix + "r_vy", r_kse_vy);
    nh_.getParam(prefix + "r_vz", r_kse_vz);

    r_legodo_init << r_kse_vx, r_kse_vy,r_kse_vz;
    R_legodo = r_legodo_init.array().square().matrix();
    R_legodo_init = R_legodo;
    cov_legodo = R_legodo.asDiagonal();

    leg_odometer_.setInitVelocityCov(cov_legodo);
    leg_odometer_.setInitVelocityStd(r_legodo_init);

    // not subscribing to IMU messages to get omega and stuff for now
    // if(!nh_.getParam(prefix + "imu_topic", imu_topic_)){
    //     ROS_WARN_STREAM("Couldn't get the IMU topic. Using default: " << imu_topic_);
    // }
    // imu_sub_ = nh_.subscribe(imu_topic_, 100, &LegodoHandlerROS::imuCallback, this);

    std::vector<std::string> leg_names = {"lf", "rf", "rh", "lh"};
    if(debug_){
        for(int i=0; i<4; i++){
            vel_debug_.push_back(nh.advertise<geometry_msgs::TwistStamped>(leg_names[i] + "_veldebug",10));
            grf_debug_.push_back(nh.advertise<geometry_msgs::WrenchStamped>(leg_names[i]+ "_grf", 10));
        }

        vel_raw_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("vel_raw", 10);
        stance_pub_ = nh.advertise<pronto_msgs::QuadrupedStance>("stance", 10);

        dl_pose_.reset(new pronto::DataLogger("prontopos.txt"));
        dl_pose_->setStartFromZero(false);
	dl_vel_.reset(new pronto::DataLogger("prontovel.txt"));
	dl_vel_->setStartFromZero(false);

    dl_vel_sigma_.reset(new pronto::DataLogger("velsigma.txt"));
    dl_vel_->setStartFromZero(false);

        wrench_msg_.wrench.torque.x = 0;
        wrench_msg_.wrench.torque.y = 0;
        wrench_msg_.wrench.torque.z = 0;
    }
}

void LegodoHandlerBase::getPreviousState(const StateEstimator *est)
{
    // TODO try not to use the state estimator object

    est->getHeadState(head_state_,head_cov_);
    //eigen_dump(head_state_);

    // take the acceleration, rotational rate and orientation from the current
    // state of the filter
    xd_ = head_state_.velocity();
    xdd_ = head_state_.acceleration();
    omega_ = head_state_.angularVelocity();
    omegad_ = Eigen::Vector3d::Zero(); // TODO retrieve angular acceleration
    orientation_ = head_state_.orientation();

    std::cout<<"+++++++++++++++++++"<<std::endl;
    std::cout<<"citting LegodoHandlerBase::getPreviousState............."<<std::endl;
    std::cout<<"+++++++++++++++++++"<<std::endl;

    if(debug_){
      double time = ((double)head_state_.utime)*1e-6;
        dl_pose_->addSampleCSV(time, head_state_.position(), orientation_);
        dl_vel_->addSample(time, head_state_.velocity(), head_state_.angularVelocity());

        Matrix3d vel_cov = head_cov_.block<3,3>(RBIS::velocity_ind, RBIS::velocity_ind);
        Matrix3d omega_cov = head_cov_.block<3,3>(RBIS::angular_velocity_ind, RBIS::angular_velocity_ind);

        Vector3d vel_sigma = vel_cov.diagonal().array().sqrt().matrix();
        Vector3d omega_sigma = omega_cov.diagonal().array().sqrt().matrix();

        dl_vel_sigma_->addSample(time, vel_sigma, omega_sigma);

        // std::cout<<"+++++++++++++++++++"<<std::endl;
        // std::cout<<vel_sigma<<std::endl;
        // std::cout<<"+++++++++++++++++++"<<std::endl;
    }
}

LegodoHandlerBase::Update * LegodoHandlerBase::computeVelocity ()
{
  std::cout<<"+++++++++++++++++++"<<std::endl;
  std::cout<<"citting LegodoHandlerBase::computeVelocity ()............."<<std::endl;
  std::cout<<"+++++++++++++++++++"<<std::endl;
    stance_estimator_.getStance(utime_,
                                q_,
                                qd_,
                                tau_,
                                orientation_,
                                stance_,
                                stance_prob_,
                                qdd_, // optional arguments starts from here
                                xd_, // note passing previous value for velocity
                                xdd_,
                                omega_,
                                omegad_);

    if(debug_){
        StanceEstimatorBase::LegVectorMap grf = stance_estimator_.getGRF();
        wrench_msg_.header.stamp = ros::Time().fromNSec(utime_*1e3);
        stance_msg_.header.stamp = wrench_msg_.header.stamp;
        std::cout<<"接触力计算:"<<std::endl;

        for(int i = 0; i<4; i++){
            wrench_msg_.wrench.force.x = grf[pronto::quadruped::LegID(i)](0);
            wrench_msg_.wrench.force.y = grf[pronto::quadruped::LegID(i)](1);
            wrench_msg_.wrench.force.z = grf[pronto::quadruped::LegID(i)](2);
            //MXR::just for debug
            //LegID{LF=0, RF, LH, RH};
            //but in gazebo/foot_contact_state is {LF,RF,LH,RH}
            //NOTE!!!!!!!!!!!!
            std::cout<<pronto::quadruped::LegID(i)<<" "<<grf[pronto::quadruped::LegID(i)](0)<<" "<<grf[pronto::quadruped::LegID(i)](1)<<" "<<grf[pronto::quadruped::LegID(i)](2)<<std::endl;
            grf_debug_[i].publish(wrench_msg_);
        }
        stance_msg_.lf = stance_[pronto::quadruped::LegID::LF] * 0.4;
        stance_msg_.rf = stance_[pronto::quadruped::LegID::RF] * 0.3;
        stance_msg_.lh = stance_[pronto::quadruped::LegID::LH] * 0.2;
        stance_msg_.rh = stance_[pronto::quadruped::LegID::RH] * 0.1;
        stance_pub_.publish(stance_msg_);
    }

    omega_ = head_state_.angularVelocity();
    // TODO add support for the dynamic stance estimator

    if(leg_odometer_.estimateVelocity(utime_,
                                      q_,
                                      qd_,
                                      omega_,
                                      stance_,
                                      stance_prob_,
                                      xd_,
                                      cov_legodo))
    {
        // save the diagonal
        R_legodo = cov_legodo.diagonal();

//        ROS_WARN("noting!!!!!!!!!!!!!!!!!!!!!");
//        std::cout<<"qd_ equal to "<<qd_<<std::endl;

        if(debug_){
            LegOdometerBase::LegVectorMap veldebug;
            leg_odometer_.getVelocitiesFromLegs(veldebug);
            geometry_msgs::TwistWithCovarianceStamped twist;
            twist.header.stamp = ros::Time().fromNSec(utime_*1000);
            twist.twist.twist.angular.x = 0;
            twist.twist.twist.angular.y = 0;
            twist.twist.twist.angular.z = 0;

            // publish the estimated velocity for each individual leg
            for(int i=0; i<4; i++){
                //veldebug is gotten from legodom
                twist.twist.twist.linear.x = veldebug[pronto::quadruped::LegID(i)](0);
                twist.twist.twist.linear.y = veldebug[pronto::quadruped::LegID(i)](1);
                twist.twist.twist.linear.z = veldebug[pronto::quadruped::LegID(i)](2);
                vel_debug_[i].publish(twist);
            }
            // publish the estimated velocity from the leg odometer
            // before it gets passed to the filter
            twist.twist.twist.linear.x = xd_(0);
            twist.twist.twist.linear.y = xd_(1);
            twist.twist.twist.linear.z = xd_(2);





            //MXR::just for debug
            //std::cout<<twist.twist.linear.x<<" "<<twist.twist.linear.y<<" "<<twist.twist.linear.z<<std::endl;

            vel_raw_.publish(twist);
        }
        return new pronto::RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
                                                       xd_,
                                                       cov_legodo,
                                                       Update::legodo,
                                                       utime_);

    }
    return NULL;
}


LegodoHandlerROS::LegodoHandlerROS(ros::NodeHandle &nh,
                                   StanceEstimatorBase& stance_est,
                                   LegOdometerBase& legodo) :
    LegodoHandlerBase(nh, stance_est, legodo)
{

}

bool LegodoHandlerROS::jointStateFromROS(const sensor_msgs::JointState& msg,
                                                         uint64_t& utime,
                                                         JointState& q,
                                                         JointState& qd,
                                                         JointState& qdd,
                                                         JointState& tau)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    if(static_cast<const sensor_msgs::JointState&>(msg).position.size() != q.rows()*q.cols()){
        ROS_WARN_STREAM_THROTTLE(1, "Joint State is expected " << \
                                 q.rows()*q.cols() << " joints but "\
                                 << msg.position.size() << " are provided.");
        return false;
    }
    // store message time in microseconds
    utime = msg.header.stamp.toNSec() / 1000;
    //MXR::NOTE:The state should One to one correspondence
//    for(int i=0; i<12; i++){
//      q(i) = msg.position[i];
//      qd(i) = msg.velocity[i];
//      tau(i) = msg.effort[i];
//    }
    q<<msg.position[0],msg.position[1],msg.position[2],
            msg.position[6],msg.position[7],msg.position[8],
            msg.position[9],msg.position[10],msg.position[11],
            msg.position[3],msg.position[4],msg.position[5];
    tau<<msg.effort[0],msg.effort[1],msg.effort[2],
            msg.effort[6],msg.effort[7],msg.effort[8],
            msg.effort[9],msg.effort[10],msg.effort[11],
            msg.effort[3],msg.effort[4],msg.effort[5];
    qd<<msg.velocity[0],msg.velocity[1],msg.velocity[2],
            msg.velocity[6],msg.velocity[7],msg.velocity[8],
            msg.velocity[9],msg.velocity[10],msg.velocity[11],
            msg.velocity[3],msg.velocity[4],msg.velocity[5];
    //q = Eigen::Map<const JointState>(msg.position.data());
    //qd = Eigen::Map<const JointState>(msg.velocity.data());
    //tau = Eigen::Map<const JointState>(msg.effort.data());

    qdd = JointState::Zero(); // TODO compute the acceleration
    return true;
}

LegodoHandlerROS::Update* LegodoHandlerROS::processMessage(const sensor_msgs::JointState *msg,
                                                           StateEstimator *est)
{
//    std::cout<<"...................."<<std::endl;
//    std::cout<<"citing this............."<<std::endl;
//    std::cout<<"...................."<<std::endl;
//    ros::Duration(0.5).sleep();
    if(!jointStateFromROS(*msg, utime_, q_, qd_, qdd_, tau_)){
//      std::cout<<"...................."<<std::endl;
//      std::cout<<"cannot get the joint state from ros............."<<std::endl;
//      std::cout<<"...................."<<std::endl;
      return nullptr;
    }
    std::cout<<"LegodoHandlerROS::processMessage++++++++++++"<<std::endl;
    getPreviousState(est);

   return computeVelocity();
}

bool LegodoHandlerROS::processMessageInit(const sensor_msgs::JointState *msg,
                                          const std::map<std::string, bool> &sensor_initialized,
                                          const RBIS &default_state,
                                          const RBIM &default_cov,
                                          RBIS &init_state,
                                          RBIM &init_cov)
{
    // do nothing for now, we don't expect to initialize with the leg odometry
    return true;
}

} // namespace quadruped
} // namespace pronto
