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
std_msgs::Float64MultiArray footpos_,footpos_in_world;
std_msgs::Float64MultiArray footpos_delta;
std_msgs::Float64MultiArray foot_iscontact,real_foot_iscontact;
geometry_msgs::PoseWithCovarianceStamped foot_odom;
geometry_msgs::PoseWithCovarianceStamped foot_odomdown;

Eigen::Matrix3d rotation_matrix_trans;
Eigen::Vector3d LF_foot_Pos_b,RF_foot_Pos_b,LH_foot_Pos_b,RH_foot_Pos_b;
Eigen::Vector3d LF_foot_Pos_w,RF_foot_Pos_w,LH_foot_Pos_w,RH_foot_Pos_w;
Eigen::Vector3d LF_hip_Pos_w,RF_hip_Pos_w,LH_hip_Pos_w,RH_hip_Pos_w;
Eigen::Vector3d pre_LF_hip_Pos_w,pre_RF_hip_Pos_w,pre_LH_hip_Pos_w,pre_RH_hip_Pos_w;
Eigen::Vector3d LF_hip_Pos_b,RF_hip_Pos_b,LH_hip_Pos_b,RH_hip_Pos_b;
Eigen::Vector3d pre_LF_foot_Pos_w,pre_RF_foot_Pos_w,pre_LH_foot_Pos_w,pre_RH_foot_Pos_w;
Eigen::Vector3d LF_foot_Pos_w_real,RF_foot_Pos_w_real,LH_foot_Pos_w_real,RH_foot_Pos_w_real;
Eigen::Vector3d LF_foot_Pos_delta,RF_foot_Pos_delta,RH_foot_Pos_delta,LH_foot_Pos_delta;
Eigen::Vector3d base_position,velocity,vel_std;  //param used for logic regression
double N_contact,N_real_contact,pre_pos,z_delta,x_delta,y_delta;

class ContactTest{

public:
    ContactTest(const ros::NodeHandle& node_handle): node_handle_(node_handle){
        node_handle_.param("/pronto_real", pronto_real, bool(false));
        jointstate_sub_ = node_handle_.subscribe("/joint_states", 1, &ContactTest::testjointposCallback,this);
        jointvel_sub_ = node_handle_.subscribe("/joint_states",100,&ContactTest::testjointvelCallback,this);
        jointacc_sub_ = node_handle_.subscribe("/joint_states",1,&ContactTest::testjointeffCallback,this);
        iscontact_sub_ = node_handle_.subscribe("/gazebo/foot_contact_state",1,&ContactTest::testcontactCallback,this);
        real_contact_sub_ = node_handle_.subscribe("/laikago_contact_state",1,&ContactTest::realCallback,this);
        base_orientation_sub_ = node_handle_.subscribe("/imu",1,&ContactTest::testorientCallback,this);
        gazebopose_sub = node_handle_.subscribe("/pose_pub_node/base_pose",1,&ContactTest::testposeCallback,this);
        LegOdom_pub = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/laikago/foot_odom",1);
        LegOdomDown_pub = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/laikago/foot_odomdown",1);

        jointpos_.data.resize(12);
        footpos_.data.resize(12);
        footpos_delta.data.resize(12);
        footpos_in_world.data.resize(12);
        foot_iscontact.data.resize(4);
        real_foot_iscontact.data.resize(4);
        wrenchPublishThread_ = boost::thread(boost::bind(&ContactTest::jointpospublish, this));
        wrenchPublishThread_2 = boost::thread(boost::bind(&ContactTest::jointpospublish2, this));

    }

    ~ContactTest(){};
    void testposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
       pose_in_world.position.x = pose->pose.pose.position.x;
       pose_in_world.position.y = pose->pose.pose.position.y;
       pose_in_world.position.z = pose->pose.pose.position.z;
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
        foot_iscontact.data[0]=contact_states->data[0]; //lf
        foot_iscontact.data[1]=contact_states->data[1]; //rf
        foot_iscontact.data[2]=contact_states->data[3]; //lh
        foot_iscontact.data[3]=contact_states->data[2]; //rh
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


            //Jacobian can be commputed
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




    }
private:
    void jointpospublish2(){
        ros::Rate rate(5);
        while(ros::ok()){
            foot_odomdown=foot_odom;
            LegOdomDown_pub.publish(foot_odomdown);
            boost::recursive_mutex::scoped_lock lock(r_mutex_);
            lock.unlock();
            rate.sleep();
        }
    }
    void jointpospublish(){
        qdd<<0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
        ros::Rate rate(400);
        while (ros::ok()) {
            LF_foot_Pos_b<<footpos_.data[0],footpos_.data[1],footpos_.data[2];
            RF_foot_Pos_b<<footpos_.data[3],footpos_.data[4],footpos_.data[5];
            LH_foot_Pos_b<<footpos_.data[6],footpos_.data[7],footpos_.data[8];
            RH_foot_Pos_b<<footpos_.data[9],footpos_.data[10],footpos_.data[11];
            LF_hip_Pos_b=fwd_kin.getHipLF(q);
            RF_hip_Pos_b=fwd_kin.getHipRF(q);
            RH_hip_Pos_b=fwd_kin.getHipRH(q);
            LH_hip_Pos_b=fwd_kin.getHipLH(q);
            LF_hip_Pos_w=rotation_matrix_trans*LF_hip_Pos_b;
            RF_hip_Pos_w=rotation_matrix_trans*RF_hip_Pos_b;
            RH_hip_Pos_w=rotation_matrix_trans*RH_hip_Pos_b;
            LH_hip_Pos_w=rotation_matrix_trans*LH_hip_Pos_b;

//            std::cout<<"LF_hip_Pos_w  "<<LF_hip_Pos_w<<std::endl;
//            std::cout<<"RF_hip_Pos_w  "<<RF_hip_Pos_w<<std::endl;
//            std::cout<<"RH_hip_Pos_w  "<<RH_hip_Pos_w<<std::endl;
//            std::cout<<"LH_hip_Pos_w  "<<LH_hip_Pos_w<<std::endl;

//            std::cout<<"LF_foot_Pos_b     "<<LF_foot_Pos_b<<std::endl;
//            std::cout<<"RF_foot_Pos_b     "<<RF_foot_Pos_b<<std::endl;
//            std::cout<<"LH_foot_Pos_b     "<<LH_foot_Pos_b<<std::endl;
//            std::cout<<"RH_foot_Pos_b     "<<RH_foot_Pos_b<<std::endl;
//            std::cout<<"LF_foot_Pos_w     "<<LF_foot_Pos_w<<std::endl;
//            std::cout<<"RF_foot_Pos_w     "<<RF_foot_Pos_w<<std::endl;
//            std::cout<<"LH_foot_Pos_w     "<<LH_foot_Pos_w<<std::endl;
//            std::cout<<"RH_foot_Pos_w     "<<RH_foot_Pos_w<<std::endl;

            LF_foot_Pos_w = rotation_matrix_trans*LF_foot_Pos_b;
            RF_foot_Pos_w = rotation_matrix_trans*RF_foot_Pos_b;
            LH_foot_Pos_w = rotation_matrix_trans*LH_foot_Pos_b;
            RH_foot_Pos_w = rotation_matrix_trans*RH_foot_Pos_b;

            //MXR::NOTE: get foot_localization in world(for gp)
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

            //MXR::NOTE: just for initialization
            if(pre_LF_foot_Pos_w[0]==0.0&&pre_LF_foot_Pos_w[1]==0.0&&pre_LF_foot_Pos_w[2]==0.0){
               pre_LF_foot_Pos_w = LF_foot_Pos_w;
               pre_RF_foot_Pos_w = RF_foot_Pos_w;
               pre_RH_foot_Pos_w = RH_foot_Pos_w;
               pre_LH_foot_Pos_w = LH_foot_Pos_w;
            }
            if(pre_LF_hip_Pos_w[0]==0.0&&pre_LF_hip_Pos_w[1]==0.0&&pre_LF_hip_Pos_w[2]==0.0){
               pre_LF_hip_Pos_w = LF_hip_Pos_w;
               pre_RF_hip_Pos_w = RF_hip_Pos_w;
               pre_RH_hip_Pos_w = RH_hip_Pos_w;
               pre_LH_hip_Pos_w = LH_hip_Pos_w;
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
//                        std::cout<<"LF_foot_Pos_delta     "<<LF_foot_Pos_delta<<std::endl;
//                        std::cout<<"RF_foot_Pos_delta     "<<RF_foot_Pos_delta<<std::endl;
//                        std::cout<<"RH_foot_Pos_delta     "<<RH_foot_Pos_delta<<std::endl;
//                        std::cout<<"LH_foot_Pos_delta     "<<LH_foot_Pos_delta<<std::endl;

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
                                   RF_foot_Pos_delta.x()*foot_iscontact.data[1]+RH_foot_Pos_delta.x()*foot_iscontact.data[3]+LH_foot_Pos_delta.x()*foot_iscontact.data[2])*weight);
                y_delta = (weight)*(LF_foot_Pos_delta.y()*foot_iscontact.data[0]+
                        RF_foot_Pos_delta.y()*foot_iscontact.data[1]+RH_foot_Pos_delta.y()*foot_iscontact.data[3]+LH_foot_Pos_delta.y()*foot_iscontact.data[2]);
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


            }

            if(!pronto_real){
                //just for gazebo fusion
                foot_odom.header.stamp  = gazebo_time;
                if((foot_iscontact.data[0]==0.0)||(foot_iscontact.data[1]==0.0)  //当足端不接触时,默认z方向位置不变
                        ||(foot_iscontact.data[2]==0.0)||(foot_iscontact.data[3]==0.0)){
                    z_delta = pre_pos;
                    z_delta = (LF_foot_Pos_w[2]*foot_iscontact.data[0]+RF_foot_Pos_w[2]*foot_iscontact.data[1]+
                            RH_foot_Pos_w[2]*foot_iscontact.data[3]+LH_foot_Pos_w[2]*foot_iscontact.data[2])*weight;
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
//                            std::cout<<"x_delta   "<<x_delta<<std::endl;
//                            std::cout<<"y_delta   "<<y_delta<<std::endl;
//                            std::cout<<"z_delta   "<<z_delta<<std::endl;
            pre_pos = z_delta;
            foot_odom.pose.pose.position.x +=(-x_delta);
            foot_odom.pose.pose.position.y +=(-y_delta);
            foot_odom.pose.pose.position.z =(-z_delta+0.01);  //0.02 for foot radius???
            foot_odom.pose.covariance[0]=0.005;
            foot_odom.pose.covariance[7]=0.005;
            foot_odom.pose.covariance[14]=0.005;
            foot_odom.pose.pose.orientation.w=orient.w();
            foot_odom.pose.pose.orientation.x=orient.x();
            foot_odom.pose.pose.orientation.y=orient.y();
            foot_odom.pose.pose.orientation.z=orient.z();


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

//            pre_LF_hip_Pos_w[0] = LF_hip_Pos_w[0];
//            pre_LF_hip_Pos_w[1] = LF_hip_Pos_w[1];
//            pre_RF_hip_Pos_w[0] = RF_hip_Pos_w[0];
//            pre_RF_hip_Pos_w[1] = RF_hip_Pos_w[1];
//            pre_RH_hip_Pos_w[0] = RH_hip_Pos_w[0];
//            pre_RH_hip_Pos_w[1] = RH_hip_Pos_w[1];
//            pre_LH_hip_Pos_w[0] = LH_hip_Pos_w[0];
//            pre_LH_hip_Pos_w[1] = LH_hip_Pos_w[1];
//            pre_LF_hip_Pos_w[2] = LF_hip_Pos_w[2];
//            pre_RF_hip_Pos_w[2] = RF_hip_Pos_w[2];
//            pre_RH_hip_Pos_w[2] = RH_foot_Pos_w[2];
//            pre_LH_hip_Pos_w[2] = LH_foot_Pos_w[2];

            LegOdom_pub.publish(foot_odom);

            // lock thread for data safe
            boost::recursive_mutex::scoped_lock lock(r_mutex_);
            //joint postition publisher
            jointpos_pub_.publish(jointpos_);

            lock.unlock();
            rate.sleep();

          }
    }





    ros::NodeHandle node_handle_;
    ros::Subscriber jointstate_sub_,jointvel_sub_,jointacc_sub_,
    base_orientation_sub_,iscontact_sub_,real_contact_sub_,gazebopose_sub;
    ros::Publisher jointpos_pub_,footpos_pub_,lf_foot_contact_force_pub_,LegOdom_pub,LegOdomDown_pub,
    rf_foot_contact_force_pub_,lh_foot_contact_force_pub_,rh_foot_contact_force_pub_,footpos_delta_pub_;
    boost::thread wrenchPublishThread_,wrenchPublishThread_2;
    boost::recursive_mutex r_mutex_;
    geometry_msgs::WrenchStamped lf_wrench_, rf_wrench_, rh_wrench_, lh_wrench_;
    sensor_msgs::JointState joint_state_;
    sensor_msgs::Imu imu_msg_;
    ros::Time contact_time=ros::Time::now();
    std::string imu_topic_name_;
    ros::Time gazebo_time;
    geometry_msgs::Pose pose_in_world;
    bool pronto_real,hang_test,use_gazebo_time;

};

int main(int argc,char **argv){
    ros::init(argc,argv,"laikago_test");
    ros::NodeHandle nh_("~");
    int64_t utimet= 0;

    ContactTest contacttest(nh_);

    while (ros::ok()) {
        ros::spin();
      }
    return 0;



}
