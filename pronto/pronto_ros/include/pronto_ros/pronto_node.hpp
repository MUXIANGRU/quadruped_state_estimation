#pragma once

#include <ros/node_handle.h>
#include "pronto_ros/ros_frontend.hpp"
#include "pronto_ros/ins_ros_handler.hpp"
#include "pronto_ros/vicon_ros_handler.hpp"
#include "pronto_ros/pose_msg_ros_handler.hpp"
#include "pronto_ros/visual_odometry_ros_handler.hpp"
#include "pronto_ros/lidar_odometry_ros_handler.hpp"
#include "pronto_ros/scan_matcher_ros_handler.hpp"

namespace pronto {

template <class MsgT>
class ProntoNode {
public:
    using SensorList = std::vector<std::string>;
    using SensorSet = std::set<std::string>;
public:
    //MXR::(1) get node (2)node.init() (3)node.run()
    ProntoNode(ros::NodeHandle& nh,
               SensingModule<MsgT>& legodo_handler,
               DualSensingModule<sensor_msgs::Imu, MsgT>& imu_bias_lock);
    virtual void init(bool subscribe = true);
    virtual void run();

protected:
    ros::NodeHandle& nh_;
    SensingModule<MsgT>& legodo_handler_;
    DualSensingModule<sensor_msgs::Imu, MsgT>& bias_lock_handler_;
    ROSFrontEnd front_end;
    SensorList init_sensors;
    SensorList active_sensors;
    SensorSet all_sensors; // sets have unique elements
    // pointers to the modules we might want to initialize
    std::shared_ptr<InsHandlerROS> ins_handler_;
    std::shared_ptr<PoseHandlerROS> pose_handler_;
    std::shared_ptr<ViconHandlerROS> vicon_handler_;
    std::shared_ptr<VisualOdometryHandlerROS> vo_handler_;
    std::shared_ptr<LidarOdometryHandlerROS> sm_handler_;
    std::shared_ptr<ScanMatcherHandler> sm2_handler_;
};

template <class MsgT>
ProntoNode<MsgT>::ProntoNode(ros::NodeHandle &nh,
                             SensingModule<MsgT>& legodo_handler,
                             DualSensingModule<sensor_msgs::Imu, MsgT>& imu_bias_lock) :
    nh_(nh), legodo_handler_(legodo_handler), bias_lock_handler_(imu_bias_lock), front_end(nh_)
{
    // get the list of active and init sensors from the param server
    if(!nh_.getParam("/state_estimator_pronto/init_sensors", init_sensors)){
        ROS_ERROR("Not able to get init_sensors param");
    }

    if(!nh_.getParam("/state_estimator_pronto/active_sensors", active_sensors)){
        ROS_ERROR("Not able to get active_sensors param");
    }
    bool publish_pose = false;
    if(!nh_.getParam("/state_estimator_pronto/publish_pose", publish_pose)){
        ROS_WARN("Not able to get publish_pose param. Not publishing pose.");
    }
}

template <class MsgT>
void ProntoNode<MsgT>::init(bool subscribe) {
    // parameters:
    // is the module used for init?
    // do we need to move forward the filter once computed the update?
    // do we want to publish the filter state after the message is received?
    // which topic are we listening to for this module?
    //参数：
    //模块是否用于初始化？
    //我们是否需要在计算更新后向前移动过滤器？
    //是否要在收到消息后发布筛选器状态？
    //本单元我们听哪个主题？

    bool init = false;
    bool active = false;
    bool roll_forward = false;
    bool publish_head = false;
    std::string topic;
    std::string secondary_topic;

   ROS_WARN("[pronto_node.hpp] pronto_node is running....");
   ROS_WARN("[pronto_node.hpp] pronto::Init()...............");

   //???
   //Is the order between the active and init important
    for(SensorList::iterator it = active_sensors.begin(); it != active_sensors.end(); ++it){
        all_sensors.insert(*it);
    }
    for(SensorList::iterator it = init_sensors.begin(); it != init_sensors.end(); ++it){
        all_sensors.insert(*it);
    }
    //MXR::for debug:
    //MXR::how to print std::set
   std::cout<<"[pronto_node.hpp] init sensors:初始化传感器模块"<<std::endl;
   for(SensorSet::iterator it = all_sensors.begin(); it != all_sensors.end(); ++it)
   {
           std::cout<<*it<<std::endl;
   };
   std::cout<<"_______________________"<<std::endl;

    // now all_sensors contain a list which is the union (without repetition)
    // of the init sensors and active sensors
    // iterate over the sensors
    //现在所有的传感器都包含一个列表，这个列表就是联合体（不重复）
    //初始化传感器和有源传感器
    //反复检查传感器
    for(SensorSet::iterator it = all_sensors.begin(); it != all_sensors.end(); ++it)
    {

        if(!nh_.getParam("/state_estimator_pronto/"+*it + "/roll_forward_on_receive", roll_forward)){
            ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
            ROS_WARN_STREAM ("Param \"roll_forward_on_receive\" not available.");
            continue;
        }
        if(!nh_.getParam("/state_estimator_pronto/"+*it + "/publish_head_on_message", publish_head)){
            ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
            ROS_WARN_STREAM ("Param \"publish_head_on_message\" not available.");
            continue;
        }
        if(!nh_.getParam("/state_estimator_pronto/"+*it + "/topic", topic)){
            ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
            ROS_WARN_STREAM ("Param \"topic\" not available.");
            continue;
        }
        // check if the sensor is also used to initialize
        init = (std::find(init_sensors.begin(), init_sensors.end(), *it) != init_sensors.end());
        active = (std::find(active_sensors.begin(), active_sensors.end(), *it) != active_sensors.end());
        // is the IMU module in the list? Typically yes.

        //MXR::for debug:
//         std::cout<<"_______________________"<<std::endl;
// //        for(SensorSet::iterator it = all_sensors.begin(); it != all_sensors.end(); ++it)
// //        {
//         std::cout<<*it<<"  "<<init<<"  "<<active<<std::endl;
// //        };
//         std::cout<<"_______________________"<<std::endl;

        ////compare函数用来进行字符串以及其子串的比较////
        //if it(a) = "ins"
        //it->compare("ins") == 0;
        if(it->compare("ins") == 0)
        {
            std::cout<<"对 ins 模块进行处理！！！"<<std::endl;
            ins_handler_ = std::make_shared<InsHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*ins_handler_, *it, roll_forward, publish_head, topic, subscribe);
                std::cout<<"ins addSensingMdodule............."<<std::endl;
            }
            if(init){
                front_end.addInitModule(*ins_handler_, *it, topic, subscribe);
                std::cout<<"ins addInitMdodule............."<<std::endl;
            }
        }

        // is the leg odometry module in the list?
        // TODO the leg odometry handler object is costructed anyway, because
        // it is passed to the constructor as a reference. It should be
        // constructed only if we want to use it, but it is hard do handle this
        // from here.
        //腿里程计模块在列表中吗？
        //不管怎样，leg odometry处理程序对象都是构造好的，因为
        //它作为引用传递给构造函数。应该是的
        //只有当我们想使用它时才构建它，但是很难处理这个问题
        //从这里开始。

        if(it->compare("legodo") == 0) {
            std::cout<<"对 腿部里程计 模块进行处理！！！"<<std::endl;
            if(active){
                front_end.addSensingModule(legodo_handler_, *it, roll_forward, publish_head, topic, subscribe);
                std::cout<<"legodo addSensingMdodule............."<<std::endl;
            }
            if(init){
                front_end.addInitModule(legodo_handler_, *it, topic, subscribe);
                std::cout<<"legodo addInitMdodule............."<<std::endl;
            }
        }
        if(it->compare("pose_meas") == 0){
            std::cout<<"对 位姿 模块进行处理！！！"<<std::endl;
            pose_handler_ = std::make_shared<PoseHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*pose_handler_, *it, roll_forward, publish_head, topic, subscribe);
                std::cout<<"pose_meas addSensingMdodule............."<<std::endl;
            }
            if(init){
                front_end.addInitModule(*pose_handler_, *it, topic, subscribe);
                std::cout<<"pose_meas addInitMdodule............."<<std::endl;
            }

        }
        if(it->compare("vicon") == 0 ){
            vicon_handler_ = std::make_shared<ViconHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*vicon_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
            if(init){
                front_end.addInitModule(*vicon_handler_, *it, topic, subscribe);
            }
        }
        if(it->compare("bias_lock") == 0){
          std::cout<<"对 imu偏差 模块进行处理！！！"<<std::endl;
          if(!nh_.getParam("/state_estimator_pronto/"+*it + "/secondary_topic", secondary_topic)){
              ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
              ROS_WARN_STREAM ("Param \"secondary_topic\" not available.");
              continue;
          }
          if(active){

            front_end.addSensingModule(bias_lock_handler_, *it, roll_forward, publish_head, topic, subscribe);
            front_end.addSecondarySensingModule(bias_lock_handler_, *it, secondary_topic, subscribe);
            std::cout<<"bias_lock addSensingMdodule............."<<std::endl;
          }
        }
        if(it->compare("fovis") == 0 ){
            std::cout<<"对 视觉里程计 模块进行处理！！！"<<std::endl;
            vo_handler_ = std::make_shared<VisualOdometryHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*vo_handler_, *it, roll_forward, publish_head, topic, subscribe);
                std::cout<<"visual_odometry addSensingMdodule............."<<std::endl;
            }
            if(init){
                front_end.addInitModule(*vo_handler_, *it, topic, subscribe);
                std::cout<<"visual_odometry addInitMdodule............."<<std::endl;
            }
        }
        if(it->compare("scan_matcher") == 0 ){
          std::cout<<"对 激光里程计 模块进行处理！！！"<<std::endl;
          bool use_relative_pose = true;
          nh_.getParam("/state_estimator_pronto/scan_matcher/relative_pose", use_relative_pose);
          ROS_WARN_STREAM("Scan matcher will use " << (use_relative_pose ? "relative " : "absolute ") << "pose");

          if(use_relative_pose){
            sm_handler_ = std::make_shared<LidarOdometryHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*sm_handler_, *it, roll_forward, publish_head, topic, subscribe);
                std::cout<<"LidarOdometry addSensingMdodule............."<<std::endl;
            }
            if(init){
                front_end.addInitModule(*sm_handler_, *it, topic, subscribe);
                std::cout<<"LidarOdometry addInitMdodule............."<<std::endl;
            }
          } else {
            sm2_handler_ = std::make_shared<ScanMatcherHandler>(nh_);
            if(active){
                front_end.addSensingModule(*sm2_handler_, *it, roll_forward, publish_head, topic, subscribe);
                std::cout<<"ScanMatcher addSensingMdodule............."<<std::endl;
            }
            if(init){
                front_end.addInitModule(*sm2_handler_, *it, topic, subscribe);
                std::cout<<"ScanMatcher addInitMdodule............."<<std::endl;
            }
          }
        }
    }
}

template <class MsgT>
void ProntoNode<MsgT>::run() {
    init(true);
    // you ...
    ros::spin();
    // ... me round (like a record)!
}
}
