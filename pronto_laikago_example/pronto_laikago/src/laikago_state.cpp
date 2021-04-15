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

uint64_t utime;
uint64_t history_span = 60000000.0;

//ros::NodeHandle np;
pronto::quadruped::JointState q;     //joint position
pronto::quadruped::JointState qd;    //joint velocity
pronto::quadruped::JointState tau;   //jpint torque
pronto::quadruped::LegBoolMap stance;
pronto::quadruped::LegDataMap<double> stance_probability;
pronto::LegOdometer::LegVector3Map v_base_from_leg;

//pronto::quadruped::StanceEstimatorROS stance_estimator(nh,feet_forces);
//pronto::quadruped::LegOdometerROS leg_odometer_(nh,feet_jacs,fwd_kin);
//pronto::quadruped::ImuBiasLockROS imu_bias(nh);
//pronto::quadruped::LegodoHandlerROS leg_handler_(nh,stance_estimator,leg_odometer_);
//pronto::quadruped::ImuBiasLockROS bias_lock(np);

//uint64_t history_span;
//ros::NodeHandle nh_;

iit::laikago::FeetContactForces feet_contact_force;
iit::laikago::FeetJacobians feet_jacs;
iit::laikago::ForwardKinematics fwd_kin;

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
std_msgs::Float64MultiArray foot_iscontact,real_foot_iscontact;
geometry_msgs::PoseWithCovarianceStamped foot_odom;

Eigen::Matrix3d rotation_matrix_trans;
Eigen::Vector3d LF_foot_Pos_b,RF_foot_Pos_b,LH_foot_Pos_b,RH_foot_Pos_b;
Eigen::Vector3d LF_foot_Pos_w,RF_foot_Pos_w,LH_foot_Pos_w,RH_foot_Pos_w;
Eigen::Vector3d pre_LF_foot_Pos_w,pre_RF_foot_Pos_w,pre_LH_foot_Pos_w,pre_RH_foot_Pos_w;
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

       jointstate_sub_ = node_handle_.subscribe("/joint_states", 1, &testmaster::testjointposCallback,this);
       jointpos_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>("/test/joint_pos",1);
       jointvel_sub_ = node_handle_.subscribe("/joint_states",1,&testmaster::testjointvelCallback,this);
       jointacc_sub_ = node_handle_.subscribe("/joint_states",1,&testmaster::testjointeffCallback,this);
       iscontact_sub_ = node_handle_.subscribe("/gazebo/foot_contact_state",1,&testmaster::testcontactCallback,this);
       base_orientation_sub_ = node_handle_.subscribe(imu_topic_name_,1,&testmaster::testorientCallback,this);
       real_contact_sub_ = node_handle_.subscribe("/laikago_contact_state",1,&testmaster::realCallback,this);
       lf_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/laikago_pronto/lf_contact_force",1);
       rf_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/laikago_pronto/rf_contact_force",1);
       rh_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/laikago_pronto/rh_contact_force",1);
       lh_foot_contact_force_pub_=node_handle_.advertise<geometry_msgs::WrenchStamped>("/laikago_pronto/lh_contact_force",1);
       footpos_pub_=node_handle_.advertise<std_msgs::Float64MultiArray>("/laikago_pronto/foot_position_in_base",1);
       footpos_delta_pub_=node_handle_.advertise<std_msgs::Float64MultiArray>("/laikago_pronto/foot_position_delta",1);
       LegOdom_pub = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/laikago_pronto/foot_odom",1);
       jointpos_.data.resize(12);
       footpos_.data.resize(12);
       footpos_delta.data.resize(12);
       foot_iscontact.data.resize(4);
       real_foot_iscontact.data.resize(4);
       Initparams();
       wrenchPublishThread_ = boost::thread(boost::bind(&testmaster::jointpospublish, this));
//       pronto::quadruped::StanceEstimatorROS stance_estimator_(node_handle_,feet_contact_force);
//       pronto::quadruped::LegOdometerROS leg_odometer_(node_handle_, feet_jacs, fwd_kin);
//       pronto::quadruped::LegodoHandlerROS legodo_handler(node_handle_, stance_estimator_, leg_odometer_);
//       pronto::ProntoNode<sensor_msgs::JointState> node(node_handle_, legodo_handler, bias_lock);

    }
    ~testmaster(){};

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
        foot_iscontact.data[2]=contact_states->data[2];
        foot_iscontact.data[3]=contact_states->data[3];
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

   void testimu_bias_lock(pronto::quadruped::ImuBiasLockROS& bias,pronto::StateEstimator *est){
        bias.processMessage(&imu_msg_,est);
    }
   void testleg_odom(pronto::quadruped::LegodoHandlerROS& handler,pronto::StateEstimator *est){
        handler.processMessage(&joint_state_,est);
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
//    void processdata(){
//        pronto::quadruped::StanceEstimatorROS stance_estimator(nh,feet_forces);
//        pronto::quadruped::LegOdometerROS leg_odometer_(nh,feet_jacs,fwd_kin);
//        pronto::quadruped::ImuBiasLockROS imu_bias(nh);
//        //pronto::quadruped::ImuBiasLockBaseROS<pronto::JointState> bias_lock(nh);
//        pronto::quadruped::LegodoHandlerROS leg_handler_(nh,stance_estimator,leg_odometer_);
//        //pronto::quadruped::ImuBiasLock
//        //pronto::ProntoNode<sensor_msgs::JointState> node(nh, leg_handler_, imu_bias);
//        //node.run();
//    }

private:
    void jointpospublish(){
        qdd<<0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
        //feet_contact_force.getFeetGRF(q,qd,tau,orient,0,foot_grfLF,qdd,xd,xdd,omega,omegad);
//        lf_wrench_.wrench.force.x= foot_grfLF[0];
//        lf_wrench_.wrench.force.y= foot_grfLF[1];
//        lf_wrench_.wrench.force.z= foot_grfLF[2];
        ros::Rate rate(400);
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
//                std::cout<<"LF_foot_Pos_delta   "<<LF_foot_Pos_delta<<std::endl;
//                std::cout<<"RF_foot_Pos_delta   "<<RF_foot_Pos_delta<<std::endl;
//                std::cout<<"RH_foot_Pos_delta   "<<RH_foot_Pos_delta<<std::endl;
//                std::cout<<"LH_foot_Pos_delta   "<<LH_foot_Pos_delta<<std::endl;
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




            std::cout<<"foot_odom.pose.pose.position.x   "<< foot_odom.pose.pose.position.x<<std::endl;
            std::cout<<"foot_odom.pose.pose.position.y   "<< foot_odom.pose.pose.position.y<<std::endl;
            std::cout<<"foot_odom.pose.pose.position.z   "<< foot_odom.pose.pose.position.z<<std::endl;



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

            lock.unlock();
            rate.sleep();

          }
    }

    std::string imu_topic_name_;
    ros::NodeHandle node_handle_,np;
    ros::Subscriber jointstate_sub_,jointvel_sub_,jointacc_sub_,base_orientation_sub_,iscontact_sub_,real_contact_sub_;
    ros::Publisher jointpos_pub_,footpos_pub_,lf_foot_contact_force_pub_,rf_foot_contact_force_pub_,LegOdom_pub,
    rh_foot_contact_force_pub_,lh_foot_contact_force_pub_,footpos_delta_pub_;
    boost::thread wrenchPublishThread_;
    boost::recursive_mutex r_mutex_;
    geometry_msgs::WrenchStamped lf_wrench_, rf_wrench_, rh_wrench_, lh_wrench_;
    sensor_msgs::JointState joint_state_;
    sensor_msgs::Imu imu_msg_;
    ros::Time contact_time=ros::Time::now();
    bool pronto_real,hang_test,use_gazebo_time;
    ros::Time gazebo_time;


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
//       std::cout<<"*************"<<std::endl;
//       std::cout<<foot_grfLF<<std::endl;
//       std::cout<<"*************"<<std::endl;
//    std::cout<<"*************"<<std::endl;
//    std::cout<<testmaster.getIMUState()<<std::endl;
//    std::cout<<testmaster.getJointState()<<std::endl;

    //stance_estimator_.getGRF();




    //ros::Duration(0.5).sleep();
//    ros::Rate loop_rate(50);
//    while (ros::ok()) {

//       testmaster.testimu_bias_lock(bias,&estimator);
////        std::cout<<"*************"<<std::endl;
// //       std::cout<<testmaster.getIMUState()<<std::endl;
////        std::cout<<testmaster.getJointState()<<std::endl;
////       testmaster.testleg_odom(leg_handler_);

//       ros::spinOnce();
//       loop_rate.sleep();
//      }
    return 0;



}
