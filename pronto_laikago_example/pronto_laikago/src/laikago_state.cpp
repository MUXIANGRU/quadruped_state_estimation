#include <ros/node_handle.h>
#include <pronto_quadruped_ros/stance_estimator_ros.hpp>
#include <pronto_quadruped_ros/leg_odometer_ros.hpp>
#include <sensor_msgs/JointState.h>

#include <laikago_feet_contact_forces.hpp>
#include <laikago_feet_jacobians.hpp>
#include <laikago_forward_kinematics.hpp>

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
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>


//MXR:NOTE: for test
#include <inekf_msgs/Contact.h>
#include <inekf_msgs/ContactArray.h>
#include <inekf_msgs/Kinematics.h>
#include <inekf_msgs/KinematicsArray.h>

uint64_t utime;
uint64_t history_span = 60000000.0;

//ros::NodeHandle np;
pronto::quadruped::JointState q;     //joint position
pronto::quadruped::JointState qd;    //joint velocity
pronto::quadruped::JointState tau;   //jpint torque
pronto::quadruped::LegBoolMap stance;
pronto::quadruped::LegDataMap<double> stance_probability;
pronto::LegOdometer::LegVector3Map v_base_from_leg;


iit::laikago::FeetContactForces feet_contact_force;
iit::laikago::FeetJacobians feet_jacs;
iit::laikago::ForwardKinematics fwd_kin;
iit::rbd::Matrix33d R_lf,R_rf,R_rh,R_lh;
Eigen::Matrix3d Rotation_lf,Rotation_rf,Rotation_rh,Rotation_lh;

Eigen::Quaterniond orient;           //base orientation
Eigen::Vector3d foot_grfLF,foot_grfRF,foot_grfLH,foot_grfRH;       //the out
pronto::quadruped::JointState qdd;   //joint acceleration
Eigen::Vector3d xd=Eigen::Vector3d::Zero();                  //base linear velocity
Eigen::Vector3d xdd=Eigen::Vector3d::Zero();                 //base acccleration from IMU
Eigen::Vector3d omega=Eigen::Vector3d::Zero();               //base angular velocity from IMU
Eigen::Vector3d omegad=Eigen::Vector3d::Zero();              //base angular acceleration from IMU
Eigen::Matrix3d LF_J,RF_J,LH_J,RH_J;
std_msgs::Float64MultiArray jointpos_;
std_msgs::Float64MultiArray footpos_,footpos_in_world;
std_msgs::Float64MultiArray footpos_delta;
std_msgs::Float64MultiArray foot_iscontact,real_foot_iscontact;
geometry_msgs::PoseWithCovarianceStamped foot_odom;

Eigen::Matrix3d rotation_matrix_trans;
Eigen::Vector3d LF_foot_Pos_b,RF_foot_Pos_b,LH_foot_Pos_b,RH_foot_Pos_b;
Eigen::Vector3d LF_foot_Pos_w,RF_foot_Pos_w,LH_foot_Pos_w,RH_foot_Pos_w;
Eigen::Vector3d pre_LF_foot_Pos_w,pre_RF_foot_Pos_w,pre_LH_foot_Pos_w,pre_RH_foot_Pos_w;
Eigen::Vector3d LF_foot_Pos_w_real,RF_foot_Pos_w_real,LH_foot_Pos_w_real,RH_foot_Pos_w_real;
Eigen::Vector3d LF_foot_Pos_delta,RF_foot_Pos_delta,RH_foot_Pos_delta,LH_foot_Pos_delta;
Eigen::Vector3d base_position,velocity,vel_std;
std::vector<double> beta={3.7252,-0.018077,2.8191,-0.015947,3.2241,-0.018661,3.8836,-0.019669};  //param used for logic regression
Eigen::Matrix3d vel_cov,pos_cov,velocity_covariance;
double N_contact,N_real_contact,pre_pos,z_delta,x_delta,y_delta;

class testmaster{
public:
    testmaster(const ros::NodeHandle& node_handle): node_handle_(node_handle){
        //注意参数的命名空间,加不加/
        node_handle_.param("/use_gazebo_time",use_gazebo_time,bool(false));
        node_handle_.param("/imu_topic_name", imu_topic_name_, std::string("/imu/data"));
        node_handle_.param("/pronto_real", pronto_real, bool(false));
        node_handle_.param("/hang_test",hang_test,bool(false));
        //node_handle_.param("/publish_real_tf",publish_real_tf,bool(false));
       gazebopose_sub = node_handle_.subscribe("/pose_pub_node/base_pose",1,&testmaster::testposeCallback,this);
       jointstate_sub_ = node_handle_.subscribe("/joint_states", 1, &testmaster::testjointposCallback,this);
       jointvel_sub_ = node_handle_.subscribe("/joint_states",1,&testmaster::testjointvelCallback,this);
       jointacc_sub_ = node_handle_.subscribe("/joint_states",1,&testmaster::testjointeffCallback,this);
       iscontact_sub_ = node_handle_.subscribe("/gazebo/foot_contact_state",1,&testmaster::testcontactCallback,this);
       base_orientation_sub_ = node_handle_.subscribe(imu_topic_name_,1,&testmaster::testorientCallback,this);
       real_contact_sub_ = node_handle_.subscribe("/contact_pro",1,&testmaster::realCallback,this);
       real_vel_sub_ = node_handle_.subscribe("/laikago_state/vel_raw",1,&testmaster::realVelCB,this);
 //      quaternion_sub = node_handle_.subscribe("/filter/quaternion",1,&testmaster::quaternionCB,this);
//       jointpos_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>("/test/joint_pos",1);
//       lf_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/laikago_pronto/lf_contact_force",1);
//       rf_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/laikago_pronto/rf_contact_force",1);
//       rh_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/laikago_pronto/rh_contact_force",1);
//       lh_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/laikago_pronto/lh_contact_force",1);
       footpos_pub_=node_handle_.advertise<std_msgs::Float64MultiArray>("/laikago_pronto/foot_position_in_base",1);
       foot_pose_in_world_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>("/laikago_pronto/foot_position_in_world",1);
       footpos_delta_pub_=node_handle_.advertise<std_msgs::Float64MultiArray>("/laikago_pronto/foot_position_delta",1);
       LegOdom_pub = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/laikago_pronto/foot_odom",1);
       raw_pose_pub_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/laikago_pronto/rawpose",1);
       euler_pub = node_handle_.advertise<geometry_msgs::Vector3>("/laikago_pronto/euler",1);
       euler_filter_pub = node_handle_.advertise<geometry_msgs::Vector3>("/laikago_pronto/euler_filter",1);
//       inekf_kinpub = node_handle_.advertise<inekf_msgs::KinematicsArray>("/kinematics",1);
//       inekf_contactpub = node_handle_.advertise<inekf_msgs::ContactArray>("/contacts",1);


       jointpos_.data.resize(12);
       footpos_.data.resize(12);
       footpos_delta.data.resize(12);
       footpos_in_world.data.resize(12);
       foot_iscontact.data.resize(4);
       real_foot_iscontact.data.resize(4);
       kinmsg_.frames.resize(4);
       conmsg_.contacts.resize(4);

       Initparams();
       wrenchPublishThread_ = boost::thread(boost::bind(&testmaster::jointpospublish, this));
//       pronto::quadruped::StanceEstimatorROS stance_estimator_(node_handle_,feet_contact_force);
//       pronto::quadruped::LegOdometerROS leg_odometer_(node_handle_, feet_jacs, fwd_kin);
//       pronto::quadruped::LegodoHandlerROS legodo_handler(node_handle_, stance_estimator_, leg_odometer_);
//       pronto::ProntoNode<sensor_msgs::JointState> node(node_handle_, legodo_handler, bias_lock);

    }
    ~testmaster(){};
    void testposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
       pose_in_world.position.x = pose->pose.pose.position.x;
       pose_in_world.position.y = pose->pose.pose.position.y;
       pose_in_world.position.z = pose->pose.pose.position.z;
    }
//    void quaternionCB(const geometry_msgs::QuaternionStamped::ConstPtr& qua){

//        double roll,pitch,yaw;
//        tf::Quaternion q;
//        q.setW(qua->quaternion.w);
//        q.setX(qua->quaternion.x);
//        q.setY(qua->quaternion.y);
//        q.setZ(qua->quaternion.z);
//        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
//        euler_filter.x = roll;
//        euler_filter.y = pitch;
//        euler_filter.z = yaw;

//    }
    void realVelCB(const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg){
        xd.x()=msg->twist.twist.linear.x;
        xd.y()=msg->twist.twist.linear.y;
        xd.z()=msg->twist.twist.linear.z;
    }
    void Initparams(){
           foot_odom.pose.pose.position.x = 0.0;
           foot_odom.pose.pose.position.y = 0.0;
           foot_odom.pose.pose.position.z = 0.084;
    }
    void realCallback(const std_msgs::Float64MultiArray::ConstPtr& contact_states){
                real_foot_iscontact.data[0]=contact_states->data[0];
                real_foot_iscontact.data[1]=contact_states->data[1];
                real_foot_iscontact.data[2]=contact_states->data[2];
                real_foot_iscontact.data[3]=contact_states->data[3];
    }
    void testcontactCallback(const std_msgs::Float64MultiArray::ConstPtr& contact_states){
        //the foot order in this cpp is LF RF LH RH
        //while the order in the bumper cpp is LF RF RH LH
        //NOTE!!!!!!!!!!!!!!!
        foot_iscontact.data[0]=contact_states->data[0];
        foot_iscontact.data[1]=contact_states->data[1];
        foot_iscontact.data[2]=contact_states->data[3];
        foot_iscontact.data[3]=contact_states->data[2];

//        conmsg_.contacts[0].id=0;
//        conmsg_.contacts[1].id=1;
//        conmsg_.contacts[2].id=2;
//        conmsg_.contacts[3].id=3;
//        conmsg_.contacts[0].indicator=contact_states->data[0];
//        conmsg_.contacts[1].indicator=contact_states->data[1];
//        conmsg_.contacts[2].indicator=contact_states->data[2];
//        conmsg_.contacts[3].indicator=contact_states->data[3];


//        conmsg_.contacts[0].id=1;
//        conmsg_.contacts[1].id=0;
//        conmsg_.contacts[2].id=2;
//        conmsg_.contacts[3].id=3;
//        conmsg_.contacts[0].indicator=contact_states->data[1];
//        conmsg_.contacts[1].indicator=contact_states->data[0];
//        conmsg_.contacts[2].indicator=contact_states->data[3];
//        conmsg_.contacts[3].indicator=contact_states->data[2];
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
        omega.x()=imu_msg->angular_velocity.x;
        omega.y()=imu_msg->angular_velocity.y;
        omega.z()=imu_msg->angular_velocity.z;
        xdd.x()=imu_msg->linear_acceleration.x;
        xdd.y()=imu_msg->linear_acceleration.y;
        xdd.z()=imu_msg->linear_acceleration.z-9.8;

        double roll,pitch,yaw;
        tf::Quaternion q;
        q.setW(orient.w());
        q.setX(orient.x());
        q.setY(orient.y());
        q.setZ(orient.z());
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        euler.x = roll;
        euler.y = pitch;
        euler.z = yaw;

        rotation_matrix_trans = orient.normalized().toRotationMatrix();

        imu_msg_.header = imu_msg->header;
        imu_msg_.orientation = imu_msg->orientation;
        imu_msg_.orientation_covariance  = imu_msg->orientation_covariance;
        imu_msg_.angular_velocity = imu_msg->angular_velocity;
        imu_msg_.angular_velocity_covariance = imu_msg->angular_velocity_covariance;
        imu_msg_.linear_acceleration = imu_msg->linear_acceleration;
        imu_msg_.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;



       // orient(imu_msg->orientation.w,imu_msg->orientation.x,imu_msg->orientation.y,imu_msg->orientation.z);


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

            //get foot orientation
            R_lf=fwd_kin.getFootOrientation(q,pronto::quadruped::LegID::LF);
            R_rf=fwd_kin.getFootOrientation(q,pronto::quadruped::LegID::RF);
            R_lh=fwd_kin.getFootOrientation(q,pronto::quadruped::LegID::LH);
            R_rh=fwd_kin.getFootOrientation(q,pronto::quadruped::LegID::RH);

            Rotation_lf=R_lf;
            Rotation_lh=R_lh;
            Rotation_rf=R_rf;
            Rotation_rh=R_rh;
            Eigen::Quaterniond q_lf(Rotation_lf);
            Eigen::Quaterniond q_rf(Rotation_rf);
            Eigen::Quaterniond q_lh(Rotation_lh);
            Eigen::Quaterniond q_rh(Rotation_rh);

            kinmsg_.frames[0].id=0;
            kinmsg_.frames[1].id=1;
            kinmsg_.frames[2].id=2;
            kinmsg_.frames[3].id=3;
            kinmsg_.frames[0].pose.pose.position.x=footpos_.data[0];
            kinmsg_.frames[0].pose.pose.position.y=footpos_.data[1];
            kinmsg_.frames[0].pose.pose.position.z=footpos_.data[2];
            kinmsg_.frames[0].pose.pose.orientation.w=q_lf.w();
            kinmsg_.frames[0].pose.pose.orientation.x=q_lf.x();
            kinmsg_.frames[0].pose.pose.orientation.y=q_lf.y();
            kinmsg_.frames[0].pose.pose.orientation.z=q_lf.z();

            kinmsg_.frames[1].pose.pose.position.x=footpos_.data[3];
            kinmsg_.frames[1].pose.pose.position.y=footpos_.data[4];
            kinmsg_.frames[1].pose.pose.position.z=footpos_.data[5];
            kinmsg_.frames[1].pose.pose.orientation.w=q_rf.w();
            kinmsg_.frames[1].pose.pose.orientation.x=q_rf.x();
            kinmsg_.frames[1].pose.pose.orientation.y=q_rf.y();
            kinmsg_.frames[1].pose.pose.orientation.z=q_rf.z();

            kinmsg_.frames[2].pose.pose.position.x=footpos_.data[9];
            kinmsg_.frames[2].pose.pose.position.y=footpos_.data[10];
            kinmsg_.frames[2].pose.pose.position.z=footpos_.data[11];
            kinmsg_.frames[2].pose.pose.orientation.w=q_rh.w();
            kinmsg_.frames[2].pose.pose.orientation.x=q_rh.x();
            kinmsg_.frames[2].pose.pose.orientation.y=q_rh.y();
            kinmsg_.frames[2].pose.pose.orientation.z=q_rh.z();

            kinmsg_.frames[3].pose.pose.position.x=footpos_.data[6];
            kinmsg_.frames[3].pose.pose.position.y=footpos_.data[7];
            kinmsg_.frames[3].pose.pose.position.z=footpos_.data[8];
            kinmsg_.frames[3].pose.pose.orientation.w=q_lh.w();
            kinmsg_.frames[3].pose.pose.orientation.x=q_lh.x();
            kinmsg_.frames[3].pose.pose.orientation.y=q_lh.y();
            kinmsg_.frames[3].pose.pose.orientation.z=q_lh.z();


            gazebo_time = joint_state_.header.stamp;



    //    q(0,0)=joint_states->position[0];
    //    q(1,0)=joint_states->position[1];
    //    q(2,0)=joint_states->position[2];
    //    q(3,0)=joint_states->position[3];
    //    q(4,0)=joint_states->position[4];
    //    q(5,0)=joint_states->position[5];
    //    q(6,0)=joint_states->position[6];
    //    q(7,0)=joint_states->position[7];
    //    q(8,0)=joint_states->position[8];
    //    q(9,0)=joint_states->position[9];
    //    q(10,0)=joint_states->position[10];
    //    q(11,0)=joint_states->position[11];
        //std::cout<<q<<std::endl;

    }
//    void testcontactforce(){
//        foot_grfLF=feet_contact_force.getFootGRF(q,qd,tau,orient,pronto::quadruped::LegID::LF,qdd,xd,xdd,omega,omegad);
//        foot_grfRF=feet_contact_force.getFootGRF(q,qd,tau,orient,pronto::quadruped::LegID::RF,qdd,xd,xdd,omega,omegad);
//        foot_grfLH=feet_contact_force.getFootGRF(q,qd,tau,orient,pronto::quadruped::LegID::LH,qdd,xd,xdd,omega,omegad);
//        foot_grfRH=feet_contact_force.getFootGRF(q,qd,tau,orient,pronto::quadruped::LegID::RH,qdd,xd,xdd,omega,omegad);
//        lf_wrench_.wrench.force.x= foot_grfLF[0];
//        lf_wrench_.wrench.force.y= foot_grfLF[1];
//        lf_wrench_.wrench.force.z= foot_grfLF[2];
//    }
   Eigen::Vector3d getposDelta(Eigen::Matrix3d& orient,Eigen::Vector3d& footpos){
        Eigen::Vector3d pos_delta;
        pos_delta = orient*footpos-footpos;
        return pos_delta;
    }
   sensor_msgs::JointState getJointState(){
        return joint_state_;
    }
   sensor_msgs::Imu getIMUState(){
        return imu_msg_;
    }
    Eigen::Vector3d computebasePositionFromContactMessage(pronto::quadruped::JointState& q,std_msgs::Float64MultiArray& foot_contact){
        ros::Time contact_time;//= ros::Time::now();
        Eigen::Vector3d lf_delta,rf_delta,lh_delta,rh_delta;
        Eigen::Vector3d lf_sum,rf_sum,lh_sum,rh_sum;
        pronto::quadruped::JointState q_pre0,q_pre1,q_pre2,q_pre3;

        if(foot_contact.data[0]==1){
            //contact_time=ros::Time::now();
           lf_delta<<std::abs(lf_delta[0]),std::abs(lf_delta[1]),std::abs(lf_delta[2]);

            lf_sum+=lf_delta;
            q_pre0 = q;
            lf_delta<<0,0,0;
        }
        lf_delta=fwd_kin.getFootPos(q,pronto::quadruped::LegID::LF)-fwd_kin.getFootPos(q_pre0,pronto::quadruped::LegID::LF);
        if(foot_contact.data[1]==1){
            //contact_time=ros::Time::now();
            rf_delta<<std::abs(rf_delta[0]),std::abs(rf_delta[1]),std::abs(rf_delta[2]);

            rf_sum+=rf_delta;
            q_pre1 = q;
            rf_delta<<0,0,0;
        }
        rf_delta=fwd_kin.getFootPos(q,pronto::quadruped::LegID::RF)-fwd_kin.getFootPos(q_pre1,pronto::quadruped::LegID::RF);
        if(foot_contact.data[3]==1){
            //contact_time=ros::Time::now();
            lh_delta<<std::abs(lh_delta[0]),std::abs(lh_delta[1]),std::abs(lh_delta[2]);

            lh_sum+=lh_delta;
            q_pre2 = q;
            lh_delta<<0,0,0;
        }
        lh_delta=fwd_kin.getFootPos(q,pronto::quadruped::LegID::LH)-fwd_kin.getFootPos(q_pre2,pronto::quadruped::LegID::LH);
        if(foot_contact.data[2]==1){
            //contact_time=ros::Time::now();
            rh_delta<<std::abs(rh_delta[0]),std::abs(rh_delta[1]),std::abs(rh_delta[2]);

            rh_sum+=rh_delta;
            q_pre3 = q;
            rh_delta<<0,0,0;
        }
        rh_delta=fwd_kin.getFootPos(q,pronto::quadruped::LegID::RH)-fwd_kin.getFootPos(q_pre3,pronto::quadruped::LegID::RH);
        base_position = (foot_contact.data[0]*lf_sum+foot_contact.data[1]*rf_sum+foot_contact.data[3]*lh_sum+foot_contact.data[2]*rh_sum)/2;
        return base_position;
    }

private:
    void jointpospublish(){
        qdd<<0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
        //feet_contact_force.getFeetGRF(q,qd,tau,orient,0,foot_grfLF,qdd,xd,xdd,omega,omegad);
//        lf_wrench_.wrench.force.x= foot_grfLF[0];
//        lf_wrench_.wrench.force.y= foot_grfLF[1];
//        lf_wrench_.wrench.force.z= foot_grfLF[2];
        ros::Rate rate(200);
        while (ros::ok()) {
            LF_foot_Pos_b<<footpos_.data[0],footpos_.data[1],footpos_.data[2];
            RF_foot_Pos_b<<footpos_.data[3],footpos_.data[4],footpos_.data[5];
            LH_foot_Pos_b<<footpos_.data[6],footpos_.data[7],footpos_.data[8];
            RH_foot_Pos_b<<footpos_.data[9],footpos_.data[10],footpos_.data[11];

//            std::cout<<"LF_foot_Pos_b        "<<LF_foot_Pos_b<<std::endl;
//            std::cout<<"RF_foot_Pos_b        "<<RF_foot_Pos_b<<std::endl;
//            std::cout<<"LH_foot_Pos_b        "<<LH_foot_Pos_b<<std::endl;
//            std::cout<<"RH_foot_Pos_b        "<<RH_foot_Pos_b<<std::endl;

            LF_foot_Pos_w = rotation_matrix_trans*LF_foot_Pos_b;
            RF_foot_Pos_w = rotation_matrix_trans*RF_foot_Pos_b;
            LH_foot_Pos_w = rotation_matrix_trans*LH_foot_Pos_b;
            RH_foot_Pos_w = rotation_matrix_trans*RH_foot_Pos_b;

            //MXR::NOTE: get the foot position in world link
            // it doesn't concerned with the legodom computation
            if(!pronto_real){
                Eigen::Vector3d POSE_IN_WORLD;
                POSE_IN_WORLD<<pose_in_world.position.x,pose_in_world.position.y,pose_in_world.position.z;
                LF_foot_Pos_w_real = LF_foot_Pos_w+POSE_IN_WORLD;
                RF_foot_Pos_w_real = RF_foot_Pos_w+POSE_IN_WORLD;
                RH_foot_Pos_w_real = RH_foot_Pos_w+POSE_IN_WORLD;
                LH_foot_Pos_w_real = LH_foot_Pos_w+POSE_IN_WORLD;
            }else{
                Eigen::Vector3d POSE_IN_WORLD;
                POSE_IN_WORLD<<foot_odom.pose.pose.position.x,foot_odom.pose.pose.position.y,foot_odom.pose.pose.position.z;
                LF_foot_Pos_w_real = LF_foot_Pos_w+POSE_IN_WORLD;
                RF_foot_Pos_w_real = RF_foot_Pos_w+POSE_IN_WORLD;
                RH_foot_Pos_w_real = RH_foot_Pos_w+POSE_IN_WORLD;
                LH_foot_Pos_w_real = LH_foot_Pos_w+POSE_IN_WORLD;
            }


//            std::cout<<"rotation_matrix_trans        "<<rotation_matrix_trans<<std::endl;
//            std::cout<<"LF_foot_Pos_w        "<<LF_foot_Pos_w<<std::endl;
//            std::cout<<"pre_LF_foot_Pos_w        "<<pre_LF_foot_Pos_w<<std::endl;
//            std::cout<<"RF_foot_Pos_w        "<<RF_foot_Pos_w<<std::endl;
//            std::cout<<"LH_foot_Pos_w        "<<LH_foot_Pos_w<<std::endl;
//            std::cout<<"RH_foot_Pos_w        "<<RH_foot_Pos_w<<std::endl;

            //MXR::NOTE: just for initialization
            if(pre_LF_foot_Pos_w[0]==0.0&&pre_LF_foot_Pos_w[1]==0.0&&pre_LF_foot_Pos_w[2]==0.0){
               pre_LF_foot_Pos_w = LF_foot_Pos_w;
               pre_RF_foot_Pos_w = RF_foot_Pos_w;
               pre_RH_foot_Pos_w = RH_foot_Pos_w;
               pre_LH_foot_Pos_w = LH_foot_Pos_w;
            }
            //MXR::NOTE:  get the foot position delta in the world_link between [t_i,t_(i+1)]
            if(foot_iscontact.data[0]==1.0||real_foot_iscontact.data[0]==1.0){
                footpos_delta.data[0]=LF_foot_Pos_w[0]-pre_LF_foot_Pos_w[0];
                footpos_delta.data[1]=LF_foot_Pos_w[1]-pre_LF_foot_Pos_w[1];
                footpos_delta.data[2]=LF_foot_Pos_w[2]-pre_LF_foot_Pos_w[2];
            }
            if(foot_iscontact.data[1]==1.0||real_foot_iscontact.data[1]==1.0){
                footpos_delta.data[3]=RF_foot_Pos_w[0]-pre_RF_foot_Pos_w[0];
                footpos_delta.data[4]=RF_foot_Pos_w[1]-pre_RF_foot_Pos_w[1];
                footpos_delta.data[5]=RF_foot_Pos_w[2]-pre_RF_foot_Pos_w[2];
            }
            if(foot_iscontact.data[2]==1.0||real_foot_iscontact.data[2]==1.0){
                footpos_delta.data[6]=RH_foot_Pos_w[0]-pre_RH_foot_Pos_w[0];
                footpos_delta.data[7]=RH_foot_Pos_w[1]-pre_RH_foot_Pos_w[1];
                footpos_delta.data[8]=RH_foot_Pos_w[2]-pre_RH_foot_Pos_w[2];
            }
            if(foot_iscontact.data[3]==1.0||real_foot_iscontact.data[3]==1.0){
                footpos_delta.data[9]=LH_foot_Pos_w[0]-pre_LH_foot_Pos_w[0];
                footpos_delta.data[10]=LH_foot_Pos_w[1]-pre_LH_foot_Pos_w[1];
                footpos_delta.data[11]=LH_foot_Pos_w[2]-pre_LH_foot_Pos_w[2];
            }


            LF_foot_Pos_delta<<footpos_delta.data[0],footpos_delta.data[1],footpos_delta.data[2];
            RF_foot_Pos_delta<<footpos_delta.data[3],footpos_delta.data[4],footpos_delta.data[5];
            RH_foot_Pos_delta<<footpos_delta.data[6],footpos_delta.data[7],footpos_delta.data[8];
            LH_foot_Pos_delta<<footpos_delta.data[9],footpos_delta.data[10],footpos_delta.data[11];



            //the number of the contact foot
            N_contact = foot_iscontact.data[0]+foot_iscontact.data[1]+foot_iscontact.data[2]+foot_iscontact.data[3];
            N_real_contact = real_foot_iscontact.data[0]+real_foot_iscontact.data[1]+real_foot_iscontact.data[2]+real_foot_iscontact.data[3];
            double weight;
            if(N_contact==4.0||N_real_contact==4.0){
                weight = 0.25;
            }else if(N_contact==3.0||N_real_contact==3.0){
                weight=0.33;
            }else if(N_contact==2.0||N_real_contact==2.0){
                weight=0.5;
            }else if(N_contact==1.0||N_real_contact==1.0){
                weight=1.0;
            }else{
                weight=0.0;
            }

            if(!pronto_real){
                //gazebo
                x_delta = ((LF_foot_Pos_delta.x()*foot_iscontact.data[0]+
                                   RF_foot_Pos_delta.x()*foot_iscontact.data[1]+RH_foot_Pos_delta.x()*foot_iscontact.data[2]+LH_foot_Pos_delta.x()*foot_iscontact.data[3])*weight);
                y_delta = (weight)*(LF_foot_Pos_delta.y()*foot_iscontact.data[0]+
                        RF_foot_Pos_delta.y()*foot_iscontact.data[1]+RH_foot_Pos_delta.y()*foot_iscontact.data[2]+LH_foot_Pos_delta.y()*foot_iscontact.data[3]);
    //增量式更新在trot时候z方向有较大飘逸
                //            double z_delta = (weight)*(LF_foot_Pos_delta.z()*foot_iscontact.data[0]+
    //                    RF_foot_Pos_delta.z()*foot_iscontact.data[1]+RH_foot_Pos_delta.z()*foot_iscontact.data[2]+LH_foot_Pos_delta.z()*foot_iscontact.data[3]);

                z_delta = (0.25)*(LF_foot_Pos_w[2]+
                        RF_foot_Pos_w[2]+RH_foot_Pos_w[2]+LH_foot_Pos_w[2]);
            }else{
                //real_dog
                std::cout<<real_foot_iscontact.data[0]<<" "<<real_foot_iscontact.data[1]<<" "
                                                     <<real_foot_iscontact.data[2]<<" "<<real_foot_iscontact.data[3]<<std::endl;
                x_delta = ((LF_foot_Pos_delta.x()*real_foot_iscontact.data[0]+
                                   RF_foot_Pos_delta.x()*real_foot_iscontact.data[1]+RH_foot_Pos_delta.x()*real_foot_iscontact.data[2]+LH_foot_Pos_delta.x()*real_foot_iscontact.data[3])*weight);
                y_delta = (weight)*(LF_foot_Pos_delta.y()*real_foot_iscontact.data[0]+
                        RF_foot_Pos_delta.y()*real_foot_iscontact.data[1]+RH_foot_Pos_delta.y()*real_foot_iscontact.data[2]+LH_foot_Pos_delta.y()*real_foot_iscontact.data[3]);
                z_delta = (0.25)*(LF_foot_Pos_w[2]+
                        RF_foot_Pos_w[2]+RH_foot_Pos_w[2]+LH_foot_Pos_w[2]);


//                std::cout<<"x_delta   "<<x_delta<<std::endl;
//                std::cout<<"y_delta   "<<y_delta<<std::endl;
//                std::cout<<"z_delta   "<<z_delta<<std::endl;

            }

            if(!pronto_real){
                //just for gazebo fusion
                foot_odom.header.stamp  = gazebo_time;
                if((foot_iscontact.data[0]==0.0)||(foot_iscontact.data[1]==0.0)  //当足端不接触时,默认z方向位置不变
                        ||(foot_iscontact.data[2]==0.0)||(foot_iscontact.data[3]==0.0)){
                    z_delta = pre_pos;
                }
            }else{
                foot_odom.header.stamp  = ros::Time::now();
                if(!hang_test){
                    //ROS_ERROR("@@@@@@@@@@@@@@@@@@@@");
                    if((real_foot_iscontact.data[0]==0.0)||(real_foot_iscontact.data[1]==0.0)  //当足端不接触时,默认z方向位置不变
                            ||(real_foot_iscontact.data[2]==0.0)||(real_foot_iscontact.data[3]==0.0)){
                        z_delta = pre_pos;
                    }
                }

            }

            pre_pos = z_delta;

            foot_odom.pose.pose.position.x +=(-x_delta);
            foot_odom.pose.pose.position.y +=(-y_delta);
            foot_odom.pose.pose.position.z =(-z_delta+0.01);  //0.02 for foot radius???
            foot_odom.pose.covariance[0]=0.005;
            foot_odom.pose.covariance[7]=0.005;
            foot_odom.pose.covariance[14]=0.005;




//            std::cout<<"foot_odom.pose.pose.position.x   "<< foot_odom.pose.pose.position.x<<std::endl;
//            std::cout<<"foot_odom.pose.pose.position.y   "<< foot_odom.pose.pose.position.y<<std::endl;
//            std::cout<<"foot_odom.pose.pose.position.z   "<< foot_odom.pose.pose.position.z<<std::endl;



            pre_LF_foot_Pos_w[0] = LF_foot_Pos_w[0];
            pre_LF_foot_Pos_w[1] = LF_foot_Pos_w[1];
            pre_RF_foot_Pos_w[0] = RF_foot_Pos_w[0];
            pre_RF_foot_Pos_w[1] = RF_foot_Pos_w[1];
            pre_RH_foot_Pos_w[0] = RH_foot_Pos_w[0];
            pre_RH_foot_Pos_w[1] = RH_foot_Pos_w[1];
            pre_LH_foot_Pos_w[0] = LH_foot_Pos_w[0];
            pre_LH_foot_Pos_w[1] = LH_foot_Pos_w[1];
            pre_LF_foot_Pos_w[2] = LF_foot_Pos_w[2];
            pre_RF_foot_Pos_w[2] = RF_foot_Pos_w[2];
            pre_RH_foot_Pos_w[2] = RH_foot_Pos_w[2];
            pre_LH_foot_Pos_w[2] = LH_foot_Pos_w[2];

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
            footpos_pub_.publish(footpos_);
            footpos_delta_pub_.publish(footpos_delta);
            //compute the odometry from the contact
            //testpos =computebasePositionFromContactMessage(q,foot_iscontact);
            //contact_force publisher
            lf_foot_contact_force_pub_.publish(lf_wrench_);
            rf_foot_contact_force_pub_.publish(rf_wrench_);
            lh_foot_contact_force_pub_.publish(lh_wrench_);
            rh_foot_contact_force_pub_.publish(rh_wrench_);
            LegOdom_pub.publish(foot_odom);

            kinmsg_.header.stamp=gazebo_time;
            conmsg_.header.stamp=gazebo_time;
            inekf_kinpub.publish(kinmsg_);
            inekf_contactpub.publish(conmsg_);

            footpos_in_world.data[0]=LF_foot_Pos_w_real.x();
            footpos_in_world.data[1]=LF_foot_Pos_w_real.y();
            footpos_in_world.data[2]=LF_foot_Pos_w_real.z();
            footpos_in_world.data[3]=RF_foot_Pos_w_real.x();
            footpos_in_world.data[4]=RF_foot_Pos_w_real.y();
            footpos_in_world.data[5]=RF_foot_Pos_w_real.z();
            footpos_in_world.data[6]=RH_foot_Pos_w_real.x();
            footpos_in_world.data[7]=RH_foot_Pos_w_real.y();
            footpos_in_world.data[8]=RH_foot_Pos_w_real.z();
            footpos_in_world.data[9]=LH_foot_Pos_w_real.x();
            footpos_in_world.data[10]=LH_foot_Pos_w_real.y();
            footpos_in_world.data[11]=LH_foot_Pos_w_real.z();
            foot_pose_in_world_pub_.publish(footpos_in_world);

            rawPose.pose.pose.position.x = foot_odom.pose.pose.position.x;
            rawPose.pose.pose.position.y = foot_odom.pose.pose.position.y;
            rawPose.pose.pose.position.z = foot_odom.pose.pose.position.z;
//            rawPose.pose.covariance[0] = foot_odom.pose.covariance[0];
//            rawPose.pose.covariance[7] = foot_odom.pose.covariance[7];
//            rawPose.pose.covariance[14] = foot_odom.pose.covariance[14];
            rawPose.pose.pose.orientation.w = imu_msg_.orientation.w;
            rawPose.pose.pose.orientation.x = imu_msg_.orientation.x;
            rawPose.pose.pose.orientation.y = imu_msg_.orientation.y;
            rawPose.pose.pose.orientation.z = imu_msg_.orientation.z;
            raw_pose_pub_.publish(rawPose);
            euler_pub.publish(euler);
            //euler_filter_pub.publish(euler_filter);

            lock.unlock();
            rate.sleep();

          }
    }

    std::string imu_topic_name_;
    ros::NodeHandle node_handle_,np;
    ros::Subscriber jointstate_sub_,jointvel_sub_,jointacc_sub_,base_orientation_sub_,
    iscontact_sub_,real_contact_sub_,gazebopose_sub,quaternion_sub,real_vel_sub_;
    ros::Publisher jointpos_pub_,footpos_pub_,lf_foot_contact_force_pub_,rf_foot_contact_force_pub_,LegOdom_pub,
    rh_foot_contact_force_pub_,lh_foot_contact_force_pub_,footpos_delta_pub_,foot_pose_in_world_pub_,raw_pose_pub_,
    euler_pub,euler_filter_pub,inekf_kinpub,inekf_contactpub;
    boost::thread wrenchPublishThread_;
    boost::recursive_mutex r_mutex_;
    geometry_msgs::WrenchStamped lf_wrench_, rf_wrench_, rh_wrench_, lh_wrench_;
    sensor_msgs::JointState joint_state_;
    sensor_msgs::Imu imu_msg_;
    ros::Time contact_time=ros::Time::now();
    bool pronto_real,hang_test,use_gazebo_time;
    ros::Time gazebo_time;
    geometry_msgs::Pose pose_in_world;
    geometry_msgs::PoseWithCovarianceStamped rawPose;
    geometry_msgs::Vector3 euler,euler_filter;
    inekf_msgs::KinematicsArray kinmsg_;
    inekf_msgs::ContactArray conmsg_;


};
int main(int argc,char **argv){
    ros::init(argc,argv,"laikago_state");
    ros::NodeHandle nh_("~");
    int64_t utimet= 0;
    pronto::RBIS default_state,initial_state;
    pronto::RBIM default_cov,initial_cov;
    //how to define the object of enum class
    //pronto::RBISUpdateInterface::sensor_enum sensor_id = pronto::RBISUpdateInterface::ins;
    //sensor_id = ins;

    testmaster testmaster(nh_);
    pronto::RBISResetUpdate init_state(default_state,default_cov,pronto::RBISUpdateInterface::sensor_enum::ins,utimet);
    pronto::StateEstimator estimator(&init_state,history_span);
    pronto::quadruped::StanceEstimatorROS stance_estimator(nh_,feet_contact_force);
    pronto::quadruped::LegOdometerROS leg_odometer_(nh_,feet_jacs,fwd_kin);
    pronto::quadruped::ImuBiasLockROS bias(nh_);
    pronto::quadruped::LegodoHandlerROS leg_handler_(nh_,stance_estimator,leg_odometer_);
    pronto::ProntoNode<sensor_msgs::JointState> node(nh_, leg_handler_, bias);
    node.run();

    return 0;



}
