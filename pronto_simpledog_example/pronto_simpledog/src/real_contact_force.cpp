#include <ros/node_handle.h>
#include <pronto_quadruped_ros/stance_estimator_ros.hpp>
#include <pronto_quadruped_ros/leg_odometer_ros.hpp>
#include <sensor_msgs/JointState.h>

#include <simpledog_feet_contact_forces.hpp>
#include <simpledog_feet_jacobians.hpp>
#include <simpledog_forward_kinematics.hpp>

#include <pronto_quadruped_ros/bias_lock_handler_ros.hpp>
#include <pronto_quadruped_ros/legodo_handler_ros.hpp>
#include <pronto_core/state_est.hpp>
#include <pronto_core/rbis.hpp>
#include <cmath>
#include <pronto_ros/pronto_node.hpp>
#include "ros/ros.h"
#include <vector>
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64MultiArray.h>

iit::simpledog::FeetContactForces feet_contact_force;
iit::simpledog::FeetJacobians feet_jacs;
iit::simpledog::ForwardKinematics fwd_kin;
pronto::quadruped::JointState q;     //joint position
pronto::quadruped::JointState qd;    //joint velocity
pronto::quadruped::JointState tau;   //jpint torque
Eigen::Quaterniond orient;           //base orientation
Eigen::Vector3d foot_grfLF,foot_grfRF,foot_grfLH,foot_grfRH;       //the out
pronto::quadruped::JointState qdd;   //joint acceleration
Eigen::Vector3d xd=Eigen::Vector3d::Zero();                  //base linear velocity
Eigen::Vector3d xdd=Eigen::Vector3d::Zero();                 //base acccleration from IMU
Eigen::Vector3d omega=Eigen::Vector3d::Zero();               //base angular velocity from IMU
Eigen::Vector3d omegad=Eigen::Vector3d::Zero();              //base angular acceleration from IMU
Eigen::Matrix3d LF_J,RF_J,LH_J,RH_J;
std_msgs::Float64MultiArray jointpos_;
std_msgs::Float64MultiArray footpos_;
std_msgs::Float64MultiArray footpos_delta;
std_msgs::Float64MultiArray foot_iscontact;
Eigen::Matrix3d rotation_matrix_trans;
Eigen::Vector3d LF_foot_Pos_b,RF_foot_Pos_b,LH_foot_Pos_b,RH_foot_Pos_b;
Eigen::Vector3d LF_foot_Pos_w,RF_foot_Pos_w,LH_foot_Pos_w,RH_foot_Pos_w;

//MXR::NOTE:足端接触力和足端接触判定顺序没问题
class ContactTest{

public:
    ContactTest(const ros::NodeHandle& node_handle): node_handle_(node_handle){
        jointstate_sub_ = node_handle_.subscribe("/joint_states", 1, &ContactTest::testjointposCallback,this);
        jointpos_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>("/test/joint_pos",1);
        jointvel_sub_ = node_handle_.subscribe("/joint_states",100,&ContactTest::testjointvelCallback,this);
        jointacc_sub_ = node_handle_.subscribe("/joint_states",1,&ContactTest::testjointeffCallback,this);
        iscontact_sub_ = node_handle_.subscribe("/gazebo/foot_contact_state",1,&ContactTest::testcontactCallback,this);
        base_orientation_sub_ = node_handle_.subscribe("/imu",1,&ContactTest::testorientCallback,this);
        lf_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/test/lf_contact_force",1);
        rf_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/test/rf_contact_force",1);
        lh_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/test/lh_contact_force",1);
        rh_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/test/rh_contact_force",1);

        jointpos_.data.resize(12);
        footpos_.data.resize(12);
        footpos_delta.data.resize(12);
        foot_iscontact.data.resize(4);
        wrenchPublishThread_ = boost::thread(boost::bind(&ContactTest::jointpospublish, this));

    }

    ~ContactTest(){};


    void testcontactCallback(const std_msgs::Float64MultiArray::ConstPtr& contact_states){
        //the foot order in this cpp is LF RF LH RH
        //while the order in the bumper cpp is LF RF RH LH
        //NOTE!!!!!!!!!!!!!!!
        foot_iscontact.data[0]=contact_states->data[0];
        foot_iscontact.data[1]=contact_states->data[1];
        foot_iscontact.data[2]=contact_states->data[3];
        foot_iscontact.data[3]=contact_states->data[2];
//        std::cout<<"LF CONTACT  "<<contact_states->data[0]<<std::endl;
//        std::cout<<"RF CONTACT  "<<contact_states->data[1]<<std::endl;
//        std::cout<<"RH CONTACT  "<<contact_states->data[2]<<std::endl;
//        std::cout<<"LH CONTACT  "<<contact_states->data[3]<<std::endl;
    }
    void testjointvelCallback(const sensor_msgs::JointStateConstPtr& joint_states){
        qd<<joint_states->velocity[0],joint_states->velocity[1],joint_states->velocity[2],
                joint_states->velocity[6],joint_states->velocity[7],joint_states->velocity[8],
                joint_states->velocity[9],joint_states->velocity[10],joint_states->velocity[11],
                joint_states->velocity[3],joint_states->velocity[4],joint_states->velocity[5];
    }
    void testorientCallback(const sensor_msgs::ImuConstPtr& imu_msg){
        orient.x()=imu_msg->orientation.x;
        orient.y()=imu_msg->orientation.y;
        orient.z()=imu_msg->orientation.z;
        orient.w()=imu_msg->orientation.w;
        rotation_matrix_trans = orient.normalized().toRotationMatrix();

        imu_msg_.header = imu_msg->header;
        imu_msg_.orientation = imu_msg->orientation;
        imu_msg_.orientation_covariance  = imu_msg->orientation_covariance;
        imu_msg_.angular_velocity = imu_msg->angular_velocity;
        imu_msg_.angular_velocity_covariance = imu_msg->angular_velocity_covariance;
        imu_msg_.linear_acceleration = imu_msg->linear_acceleration;
        imu_msg_.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;


    }
    void testjointeffCallback(const sensor_msgs::JointStateConstPtr& joint_states){
        tau<<joint_states->effort[0],joint_states->effort[1],joint_states->effort[2],
                joint_states->effort[6],joint_states->effort[7],joint_states->effort[8],
                joint_states->effort[9],joint_states->effort[10],joint_states->effort[11],
                joint_states->effort[3],joint_states->effort[4],joint_states->effort[5];
    }
    void testjointposCallback(const sensor_msgs::JointState::ConstPtr& joint_states){
        q<<joint_states->position[0],joint_states->position[1],joint_states->position[2],               
                joint_states->position[6],joint_states->position[7],joint_states->position[8],
                joint_states->position[9],joint_states->position[10],joint_states->position[11],
                joint_states->position[3],joint_states->position[4],joint_states->position[5];
            jointpos_.data[0]=joint_states->position[0];
            jointpos_.data[1]=joint_states->position[1];
            jointpos_.data[2]=joint_states->position[2];
            jointpos_.data[3]=joint_states->position[3];
            jointpos_.data[4]=joint_states->position[4];
            jointpos_.data[5]=joint_states->position[5];
            jointpos_.data[6]=joint_states->position[6];
            jointpos_.data[7]=joint_states->position[7];
            jointpos_.data[8]=joint_states->position[8];
            jointpos_.data[9]=joint_states->position[9];
            jointpos_.data[10]=joint_states->position[10];
            jointpos_.data[11]=joint_states->position[11];


            //全局消息
            joint_state_.header = joint_states->header;
            joint_state_.name = joint_states->name;
            joint_state_.position = joint_states->position;
            joint_state_.velocity = joint_states->velocity;
            joint_state_.effort = joint_states->effort;



            //compute jacabians
            LF_J=feet_jacs.getFootJacobianLF(q);
            RF_J=feet_jacs.getFootJacobianRF(q);
            LH_J=feet_jacs.getFootJacobianLH(q);
            RH_J=feet_jacs.getFootJacobianRH(q);

            footpos_.data[0]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::LF)[0];
            footpos_.data[1]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::LF)[1];
            footpos_.data[2]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::LF)[2];
            footpos_.data[3]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::RF)[0];
            footpos_.data[4]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::RF)[1];
            footpos_.data[5]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::RF)[2];
            footpos_.data[6]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::LH)[0];
            footpos_.data[7]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::LH)[1];
            footpos_.data[8]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::LH)[2];
            footpos_.data[9]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::RH)[0];
            footpos_.data[10]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::RH)[1];
            footpos_.data[11]=fwd_kin.getFootPos(q,pronto::quadruped::LegID::RH)[2];


    }
private:
    void jointpospublish(){
        qdd<<0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
        //feet_contact_force.getFeetGRF(q,qd,tau,orient,0,foot_grfLF,qdd,xd,xdd,omega,omegad);
//        lf_wrench_.wrench.force.x= foot_grfLF[0];
//        lf_wrench_.wrench.force.y= foot_grfLF[1];
//        lf_wrench_.wrench.force.z= foot_grfLF[2];
        ros::Rate rate(100);
        while (ros::ok()) {
            LF_foot_Pos_b<<footpos_.data[0],footpos_.data[1],footpos_.data[2];
            RF_foot_Pos_b<<footpos_.data[3],footpos_.data[4],footpos_.data[5];
            LH_foot_Pos_b<<footpos_.data[6],footpos_.data[7],footpos_.data[8];
            RH_foot_Pos_b<<footpos_.data[9],footpos_.data[10],footpos_.data[11];
//            std::cout<<"LF_foot_Pos_b     "<<LF_foot_Pos_b<<std::endl;
//            std::cout<<"RF_foot_Pos_b     "<<RF_foot_Pos_b<<std::endl;
//            std::cout<<"LH_foot_Pos_b     "<<LH_foot_Pos_b<<std::endl;
//            std::cout<<"RH_foot_Pos_b     "<<RH_foot_Pos_b<<std::endl;
//            std::cout<<"LF_J  "<<LF_J<<std::endl;
//            std::cout<<"RF_J  "<<RF_J<<std::endl;
//            std::cout<<"LH_J  "<<LH_J<<std::endl;
//            std::cout<<"RH_J  "<<RH_J<<std::endl;

            LF_foot_Pos_w = rotation_matrix_trans*LF_foot_Pos_b;
            RF_foot_Pos_w = rotation_matrix_trans*RF_foot_Pos_b;
            LH_foot_Pos_w = rotation_matrix_trans*LH_foot_Pos_b;
            RH_foot_Pos_w = rotation_matrix_trans*RH_foot_Pos_b;
//            test = std::abs(RF_foot_Pos_w[1]-RF_foot_Pos_b[1]);
            footpos_delta.data[0]=std::abs(LF_foot_Pos_w[0]-LF_foot_Pos_b[0]);
            footpos_delta.data[1]=std::abs(LF_foot_Pos_w[1]-LF_foot_Pos_b[1]);
            footpos_delta.data[2]=std::abs(LF_foot_Pos_w[2]-LF_foot_Pos_b[2]);
            footpos_delta.data[3]=std::abs(RF_foot_Pos_w[0]-RF_foot_Pos_b[0]);
            footpos_delta.data[4]=std::abs(RF_foot_Pos_w[1]-RF_foot_Pos_b[1]);
            footpos_delta.data[5]=std::abs(RF_foot_Pos_w[2]-RF_foot_Pos_b[2]);
            footpos_delta.data[6]=std::abs(LH_foot_Pos_w[0]-LH_foot_Pos_b[0]);
            footpos_delta.data[7]=std::abs(LH_foot_Pos_w[1]-LH_foot_Pos_b[1]);
            footpos_delta.data[8]=std::abs(LH_foot_Pos_w[2]-LH_foot_Pos_b[2]);
            footpos_delta.data[9]=std::abs(RH_foot_Pos_w[0]-RH_foot_Pos_b[0]);
            footpos_delta.data[10]=std::abs(RH_foot_Pos_w[1]-RH_foot_Pos_b[1]);
            footpos_delta.data[11]=std::abs(RH_foot_Pos_w[2]-RH_foot_Pos_b[2]);

            //compute the grf(ground react force)
            foot_grfLF=feet_contact_force.getFootGRF(q,qd,tau,orient,pronto::quadruped::LegID::LF,qdd,xd,xdd,omega,omegad);
            foot_grfRF=feet_contact_force.getFootGRF(q,qd,tau,orient,pronto::quadruped::LegID::RF,qdd,xd,xdd,omega,omegad);
            foot_grfLH=feet_contact_force.getFootGRF(q,qd,tau,orient,pronto::quadruped::LegID::LH,qdd,xd,xdd,omega,omegad);
            foot_grfRH=feet_contact_force.getFootGRF(q,qd,tau,orient,pronto::quadruped::LegID::RH,qdd,xd,xdd,omega,omegad);
            lf_wrench_.wrench.force.x= foot_grfLF[0];
            lf_wrench_.wrench.force.y= foot_grfLF[1];
            lf_wrench_.wrench.force.z= std::abs(foot_grfLF[2]);
            lh_wrench_.wrench.force.x= foot_grfLH[0];
            lh_wrench_.wrench.force.y= foot_grfLH[1];
            lh_wrench_.wrench.force.z= std::abs(foot_grfLH[2]);
            rf_wrench_.wrench.force.x= foot_grfRF[0];
            rf_wrench_.wrench.force.y= foot_grfRF[1];
            rf_wrench_.wrench.force.z= std::abs(foot_grfRF[2]);
            rh_wrench_.wrench.force.x= foot_grfRH[0];
            rh_wrench_.wrench.force.y= foot_grfRH[1];
            rh_wrench_.wrench.force.z= std::abs(foot_grfRH[2]);

            // lock thread for data safe
            boost::recursive_mutex::scoped_lock lock(r_mutex_);
            //joint postition publisher
            jointpos_pub_.publish(jointpos_);
            //foot contact force publisher
            lf_foot_contact_force_pub_.publish(lf_wrench_);
            rf_foot_contact_force_pub_.publish(rf_wrench_);
            lh_foot_contact_force_pub_.publish(lh_wrench_);
            rh_foot_contact_force_pub_.publish(rh_wrench_);
//            std::cout<<"lf:     "<<lf_wrench_.wrench.force.z<<std::endl;
//            std::cout<<"rf:     "<<rf_wrench_.wrench.force.z<<std::endl;
//            std::cout<<"lh:     "<<lh_wrench_.wrench.force.z<<std::endl;
//            std::cout<<"rh:     "<<rh_wrench_.wrench.force.z<<std::endl;
            //foot position publisher
            footpos_pub_.publish(footpos_);
            footpos_delta_pub_.publish(footpos_delta);

            lock.unlock();
            rate.sleep();

          }
    }





    ros::NodeHandle node_handle_;
    ros::Subscriber jointstate_sub_,jointvel_sub_,jointacc_sub_,base_orientation_sub_,iscontact_sub_;
    ros::Publisher jointpos_pub_,footpos_pub_,lf_foot_contact_force_pub_,rf_foot_contact_force_pub_,lh_foot_contact_force_pub_,rh_foot_contact_force_pub_,footpos_delta_pub_;
    boost::thread wrenchPublishThread_;
    boost::recursive_mutex r_mutex_;
    geometry_msgs::WrenchStamped lf_wrench_, rf_wrench_, rh_wrench_, lh_wrench_;
    sensor_msgs::JointState joint_state_;
    sensor_msgs::Imu imu_msg_;
    ros::Time contact_time=ros::Time::now();

};

int main(int argc,char **argv){
    ros::init(argc,argv,"real_contact_force");
    ros::NodeHandle nh_("~");
    int64_t utimet= 0;
    //pronto::RBIS default_state,initial_state;
    //pronto::RBIM default_cov,initial_cov;
    //how to define the object of enum class
    //pronto::RBISUpdateInterface::sensor_enum sensor_id = pronto::RBISUpdateInterface::ins;
    //sensor_id = ins;

    ContactTest contacttest(nh_);
    //pronto::RBISResetUpdate init_state(default_state,default_cov,pronto::RBISUpdateInterface::sensor_enum::ins,utimet);
    //pronto::StateEstimator estimator(&init_state,history_span);
    //pronto::quadruped::StanceEstimatorROS stance_estimator(nh_,feet_contact_force);
    //pronto::quadruped::LegOdometerROS leg_odometer_(nh_,feet_jacs,fwd_kin);
    //pronto::quadruped::ImuBiasLockROS bias(nh_);
    //pronto::quadruped::LegodoHandlerROS leg_handler_(nh_,stance_estimator,leg_odometer_);
    //pronto::ProntoNode<sensor_msgs::JointState> node(nh_, leg_handler_, bias);
    //node.run();
    while (ros::ok()) {
        ros::spin();
      }
    return 0;



}
