#pragma once

#include <pronto_core/sensing_module.hpp>
#include <pronto_core/state_est.hpp>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <chrono>
#include <tf/transform_broadcaster.h>
#include <string>
#include <cstdlib>
#include <cxxabi.h>
#include <nav_msgs/Path.h>
#include <eigen_conversions/eigen_msg.h>

template<typename T>
std::string type_name()
{
    int status;
    std::string tname = typeid(T).name();
    char *demangled_name = abi::__cxa_demangle(tname.c_str(), NULL, NULL, &status);
    if(status == 0) {
        tname = demangled_name;
        std::free(demangled_name);
    }
    return tname;
}

namespace pronto {
class ROSFrontEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    using SensorId = std::string;

    ROSFrontEnd(ros::NodeHandle &nh, bool verbose = false);
    virtual ~ROSFrontEnd();

    template<class MsgT>
    void addSensingModule(SensingModule<MsgT>& module,
                          const SensorId& sensor_id,
                          bool roll_forward,
                          bool publish_head,
                          const std::string& topic,
                          bool subscribe = true);

    template<class MsgT, class SecondaryMsgT>
    inline void addSecondarySensingModule(DualSensingModule<MsgT, SecondaryMsgT>& module,
                                          const SensorId& sensor_id,
                                          const std::string& topic,
                                          bool subscribe)
    {
        if(!subscribe){
            return;
        }
        std::cerr << sensor_id << " subscribing to " << topic;
        std::cerr << " with SecondaryMsgT = " << type_name<SecondaryMsgT>() << std::endl;
        secondary_subscribers_[sensor_id] = nh_.subscribe<SecondaryMsgT>(topic,
                                                                         10000,
                                                                         boost::bind(&ROSFrontEnd::secondaryCallback<MsgT, SecondaryMsgT>,
                                                                                     this,
                                                                                     _1,
                                                                                     sensor_id),
                                                                         ros::VoidConstPtr(),
                                                                         ros::TransportHints().tcpNoDelay());
    }


    template <class MsgT>
    void addInitModule(SensingModule<MsgT>& module,
                       const SensorId& sensor_id,
                       const std::string& topic,
                       bool subscribe = true);

    bool areModulesInitialized();

    bool isFilterInitialized();

    inline void getState(RBIS& state, RBIM& cov) const{
        state_est_->getHeadState(state, cov);
    }

    inline bool reset(const RBIS& state, const RBIM& cov) {
        state_est_->addUpdate(new pronto::RBISResetUpdate(state,
                                                               cov,
                                                               RBISUpdateInterface::reset,
                                                               state.utime), true);
    }
    template <class MsgT>
    void initCallback(boost::shared_ptr<MsgT const> msg,
                      const SensorId& Key);

    template <class PrimaryMsgT, class SecondaryMsgT>
    void secondaryCallback(boost::shared_ptr<SecondaryMsgT const> msg,
                           const SensorId& sensor_id);

    template <class MsgT>
    void callback(boost::shared_ptr<MsgT const> msg,
                  const SensorId& Key);
protected:
    bool initializeFilter();

    void initializeState();
    void initializeCovariance();


private:
    ros::NodeHandle& nh_;
    std::shared_ptr<StateEstimator> state_est_;
    std::map<SensorId, ros::Subscriber> sensors_subscribers_;
    std::map<SensorId, ros::Subscriber> secondary_subscribers_;
    std::map<SensorId, ros::Subscriber> init_subscribers_;
    std::map<SensorId, void*> active_modules_;
    std::map<SensorId, void*> init_modules_;
    std::map<SensorId, bool> initialized_list_;
    std::map<SensorId, bool> roll_forward_;
    std::map<SensorId, bool> publish_head_;

    RBIS default_state;
    RBIM default_cov;

    RBIS init_state;
    RBIM init_cov;

    RBIS head_state;
    RBIM head_cov;

    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::StampedTransform tf_pose_;
    bool publish_tf_ = false;

    geometry_msgs::PoseWithCovarianceStamped pose_msg_;
    geometry_msgs::TwistWithCovarianceStamped twist_msg_;

    nav_msgs::Path aicp_path;
    ros::Publisher aicp_path_publisher;

    uint64_t history_span_;

    tf::Vector3 temp_v3;
    tf::Quaternion temp_q;

    bool filter_initialized_ = false;
    bool verbose_ = true;


};
}

namespace pronto {


template <class MsgT>

//MXR:from pronto_ros/pronto_node.cpp
void ROSFrontEnd::addInitModule(SensingModule<MsgT>& module,
                                const SensorId& sensor_id,
                                const std::string& topic,
                                bool subscribe)
{
    std::cout<<"addInitModule........"<<std::endl;
    if(init_modules_.count(sensor_id) > 0){
        ROS_WARN_STREAM("Init Module \"" << sensor_id << "\" already added. Skipping.");
        return;
    }
    ROS_INFO_STREAM("Sensor init id: " << sensor_id);
    ROS_INFO_STREAM("Topic: " << topic);

    // add the sensor to the list of sensor that require initialization
    std::pair<SensorId, bool> init_id_pair(sensor_id, false);
    std::map<SensorId, bool>::iterator iter;
    initialized_list_.insert(init_id_pair);
//MXR:how to cout the STL eg:std::map<>
//    for(iter = initialized_list_.begin(); iter != initialized_list_.end(); iter++){
//          std::cout<<iter->first<<' '<<iter->second<<std::endl;
//    };
//    std::cout<<std::endl;


//     store the module as void*, to allow for different types of module to stay
//     in the same container. The type will be known when the message arrives
//     so we can properly cast back to the right type.
//将模块存储为void*，以允许不同类型的模块保留
//在同一个容器里。消息到达时将知道类型
//这样我们就可以正确地回到正确的类型。
    std::pair<SensorId, void*> pair(sensor_id, (void*) &module);
    init_modules_.insert(pair);
    if(subscribe){
        std::cerr << sensor_id << " subscribing to " << topic;
        std::cerr << " with MsgT = " << type_name<MsgT>() << std::endl;
        init_subscribers_[sensor_id] = nh_.subscribe<MsgT>(topic,
                                                           10000,
                                                           boost::bind(&ROSFrontEnd::initCallback<MsgT>,
                                                                       this,
                                                                       _1,
                                                                       sensor_id),
                                                           ros::VoidConstPtr(),
                                                           ros::TransportHints().tcpNoDelay());
    }
}

template<class MsgT>
void ROSFrontEnd::addSensingModule(SensingModule<MsgT>& module,
                                   const SensorId& sensor_id,
                                   bool roll_forward,
                                   bool publish_head,
                                   const std::string& topic,
                                   bool subscribe)
{

    // int this implementation we allow only one different type of module
    if(active_modules_.count(sensor_id) > 0){
        ROS_WARN_STREAM("Sensing Module \"" << sensor_id << "\" already added. Skipping.");
        return;
    }

    ROS_INFO_STREAM("Sensor id: " << sensor_id);
    ROS_INFO_STREAM("Roll forward: "<< (roll_forward? "yes" : "no"));
    ROS_INFO_STREAM("Publish head: "<< (publish_head? "yes" : "no"));
    ROS_INFO_STREAM("Topic: " << topic);

    // store the will to roll forward when the message is received
    std::pair<SensorId, bool> roll_pair(sensor_id, roll_forward);
    roll_forward_.insert(roll_pair);

    // store the will to publish the estimator state when the message is received
    std::pair<SensorId, bool> publish_pair(sensor_id, publish_head);
    publish_head_.insert(publish_pair);
//    std::map<SensorId, bool>::iterator iter;
//    for(iter = publish_head_.begin(); iter != publish_head_.end(); iter++){
//          std::cout<<iter->first<<' '<<iter->second<<std::endl;

//    };
//    std::cout<<std::endl;




    // store the module as void*, to allow for different types of module to stay
    // in the same container. The type will be known when the message arrives
    // so we can properly cast back to the right type.
    std::pair<SensorId, void*> pair(sensor_id, (SensingModule<MsgT>*) &module);
    active_modules_.insert(pair);
    // subscribe the generic templated callback for all modules
    if(subscribe){
        std::cerr << sensor_id << " subscribing to " << topic;
        std::cerr << " with MsgT = " << type_name<MsgT>() << std::endl;
        sensors_subscribers_[sensor_id] = nh_.subscribe<MsgT>(topic,
                                                              10000,
                                                              boost::bind(&ROSFrontEnd::callback<MsgT>,
                                                                          this, _1, sensor_id),
                                                              ros::VoidConstPtr(),
                                                              ros::TransportHints().tcpNoDelay());
    }
}



template <class MsgT>
void ROSFrontEnd::initCallback(boost::shared_ptr<MsgT const> msg, const SensorId& sensor_id){

    std::cout<<"initCallback............"<<std::endl;
    //if(verbose_){
        ROS_WARN_STREAM("Init callback for sensor " << sensor_id);
    //}
    if(initialized_list_.count(sensor_id) > 0 && !initialized_list_[sensor_id])
    {
        std::cout<<"initialized_list_.count(sensor_id) > 0...."<<std::endl;
        initialized_list_[sensor_id] = static_cast<SensingModule<MsgT>*>(init_modules_[sensor_id])->processMessageInit(msg.get(),
                                                                                                                       initialized_list_,
                                                                                                                       default_state,
                                                                                                                       default_cov,
                                                                                                                     init_state,
                                                                                                                       init_cov);


        //initialized_list_[sensor_id] = true;
//        std::cout<<initialized_list_.count(sensor_id)<<" "<<sensor_id<<std::endl;
//        std::cout<<initialized_list_[sensor_id]<<std::endl;

        // if the sensor has been successfully initialized, we unsubscribe.
        // This happens only for the sensors which are only for initialization.
        // The sensor which are for initialization and active will continue to listen
        //如果传感器已成功初始化，我们将取消订阅。
        //这只发生在仅用于初始化的传感器上。
        //用于初始化和激活的传感器将继续监听
        if(initialized_list_[sensor_id]){
            std::cout<<"!!!!!!!!!!initialized_sensor!!!!!!!!!!"<<std::endl;

            init_subscribers_[sensor_id].shutdown();
            // attempt to initialize the filter, because the value has changed
            // in the list
            //std::cout<<"initializefilter.........."<<std::endl;
            initializeFilter();
        }
    } else {
        // if we are here it means that the module is not in the list of
        // initialized modules or that the module is already initialized
        // in both cases we don't want to subscribe to this topic anymore,
        // unless there is no subscriber because we are processing a rosbag.
        //如果我们在这里，这意味着模块不在列表中
        //已初始化模块或模块已初始化
        //在这两种情况下，我们都不想再订阅这个主题了，
        //除非没有订户，因为我们正在处理一个rosbag。

        if(init_subscribers_.count(sensor_id) > 0){
            init_subscribers_[sensor_id].shutdown();
        }
    }
}

//TODO come up with a better way to activate / deactivate debug mode
#define DEBUG_MODE 0

template <class MsgT>
void ROSFrontEnd::callback(boost::shared_ptr<MsgT const> msg, const SensorId& sensor_id)
{
    std::cout<<"ROSFrontEnd::callback：开始对各个传感器进行回调(from addSensingModule)"<<std::endl;
//#if DEBUG_MODE
        ROS_INFO_STREAM("Callback for sensor " << sensor_id);
//#endif
    // this is a generic templated callback that does the same for every module:
    // if the module is initialized and the filter is ready
    // 1) take the measurement update and pass it to the filter if valid
    // 2) publish the filter state if the module wants to
    //这是一个通用的模板化回调，它对每个模块都执行相同的操作：
    //如果模块已初始化且过滤器已准备就绪
    //1）获取测量更新并将其传递给过滤器（如果有效）
    //2）如果模块想发布过滤器状态

    if(isFilterInitialized()) {
        std::cout<<"滤波器初始化......"<<std::endl;
        // std::cout<<"isFilterInitialized()......"<<std::endl;
        // appropriate casting to the right type and call to the process message
        // function to get the update
        // Record start time
#if DEBUG_MODE

        auto start = std::chrono::high_resolution_clock::now();

        std::cerr << sensor_id << " module address: " << active_modules_[sensor_id] << std::endl;
        std::cerr << "message address " << msg.get() << std::endl;
        std::cerr << "state estimator address " << state_est_.get() << std::endl;
        std::cerr << "Before cast to SensingModule<" <<type_name<MsgT>() <<">*" << std::endl;
        SensingModule<MsgT>* temp = static_cast<SensingModule<MsgT>*>(active_modules_[sensor_id]);
        std::cerr << "after cast before call" << std::endl;
        std::cerr << "address " << temp << std::endl;
        temp->processMessage(msg.get(), state_est_.get());
        std::cerr << "after call" << std::endl;

#endif
        RBISUpdateInterface* update = static_cast<SensingModule<MsgT>*>(active_modules_[sensor_id])->processMessage(msg.get(), state_est_.get());
#if DEBUG_MODE
        auto end = std::chrono::high_resolution_clock::now();
        ROS_INFO_STREAM("Time elapsed process message: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        start = end;
#endif
        // if the update is invalid, we leave
        if(update == nullptr){
#if DEBUG_MODE
            ROS_INFO_STREAM("Invalid " << sensor_id << " update" << std::endl);
#endif
            // special case for pose meas, it returns null when it does not want
            // to process data anymore
            if(sensor_id.compare("pose_meas") == 0){
                sensors_subscribers_["pose_meas"].shutdown();
            }
            return;
        }
#if DEBUG_MODE
        if(sensor_id.compare("fovis") == 0){
          ROS_INFO_STREAM("fovis update posterior: " << update->posterior_state.position().transpose());
        }
#endif
#define DEBUG_AICP 0
#if DEBUG_AICP

        if(sensor_id.compare("scan_matcher") == 0){
          aicp_path.header.frame_id = "odom";
          Eigen::Vector3d p = dynamic_cast<RBISIndexedPlusOrientationMeasurement*>(update)->measurement.head<3>();
          Eigen::Quaterniond q = dynamic_cast<RBISIndexedPlusOrientationMeasurement*>(update)->orientation;

          std::cerr << "MEASR    : " << p.transpose() << "   " << eigen_utils::getEulerAnglesDeg(q).transpose() << std::endl;

         // Eigen::Vector3d p = dynamic_cast<RBISIndexedMeasurement*>(update)->measurement.head<3>();

          // std::cerr << "ABOUT TO SEND TO FILTER THE FOLLOWING: " << p.transpose() << std::endl;

        }
#endif
        RBIS prior;
        RBIS posterior;
        RBIM prior_cov;
        RBIM posterior_cov;

state_est_->getHeadState(prior,prior_cov);
        // tell also the filter if we need to roll forward
//MXR::note
//the most important step to update the state
state_est_->addUpdate(update, roll_forward_[sensor_id]);
state_est_->getHeadState(posterior,posterior_cov);
//MXR:print the state
        eigen_dump(posterior);

if(sensor_id.compare("scan_matcher") == 0){
    ROS_WARN("sensor_id.compare(scan_matcher) == 0!!!!!!!!!!!");
    ROS_WARN("sensor_id.compare(scan_matcher) == 0!!!!!!!!!!!");
    ROS_WARN("sensor_id.compare(scan_matcher) == 0!!!!!!!!!!!");
std::cerr << "PRIOR    : " << prior.position().transpose() << " " << eigen_utils::getEulerAnglesDeg(prior.orientation()).transpose() << std::endl;
std::cerr << "POSTERIOR: " << posterior.position().transpose() << " " << eigen_utils::getEulerAnglesDeg(posterior.orientation()).transpose() << std::endl;
std::cerr << ":::::::" << std::endl;
        }

#if DEBUG_MODE
        end = std::chrono::high_resolution_clock::now();
        ROS_INFO_STREAM("Time elapsed process addupdate: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        start = end;
#endif
        //MXR::publish the pose and twist msg!!!!!!
        if(publish_head_[sensor_id]){

            ROS_ERROR("publish the pose and twist msg!!!!!!");
            //std::cout<<"publish_head_[sensor_id].........."<<std::endl;
            state_est_->getHeadState(head_state, head_cov);


            //MXR::note
            //use a temp vector temp_v3 to collect the state
            // fill in linear velocity
            //std::cout<<temp_v3[0]<<" "<<temp_v3[1]<<" "<<temp_v3[2]<<std::endl;
            tf::vectorEigenToTF(head_state.velocity(),temp_v3);
            //std::cout<<temp_v3[0]<<" "<<temp_v3[1]<<" "<<temp_v3[2]<<std::endl;
            tf::vector3TFToMsg(temp_v3,twist_msg_.twist.twist.linear);
            // fill in angular velocity
            tf::vectorEigenToTF(head_state.angularVelocity(),temp_v3);
            tf::vector3TFToMsg(temp_v3,twist_msg_.twist.twist.angular);

            // fill in time
            twist_msg_.header.stamp = ros::Time().fromNSec(head_state.utime * 1000);

            // TODO insert appropriate covariance into the message


            // set twist covariance to zero
            twist_msg_.twist.covariance.assign(0);

            Eigen::Matrix3d vel_cov = head_cov.block<3,3>(RBIS::velocity_ind,RBIS::velocity_ind);
            Eigen::Matrix3d omega_cov = head_cov.block<3,3>(RBIS::angular_velocity_ind,RBIS::angular_velocity_ind);
            Eigen::Matrix3d pos_cov = head_cov.block<3,3>(RBIS::position_ind,RBIS::position_ind);
            Eigen::Matrix3d ori_cov = head_cov.block<3,3>(RBIS::chi_ind,RBIS::chi_ind);

            for(int i=0; i<3; i++){
              for(int j=0; j<3; j++){
                twist_msg_.twist.covariance[6*i+j] = vel_cov(i,j);
                twist_msg_.twist.covariance[6*(i+3)+j+3] = omega_cov(i,j);
              }
            }

            for(int i=0; i<3; i++){
              for(int j=0; j<3; j++){
                  pose_msg_.pose.covariance[6*i+j] = pos_cov(i,j);
                  pose_msg_.pose.covariance[6*(i+3)+j+3] = ori_cov(i,j);
                //twist_msg_.twist.covariance[6*i+j] = vel_cov(i,j);
                //twist_msg_.twist.covariance[6*(i+3)+j+3] = omega_cov(i,j);
              }
            }
            //MXR::NOTE: JUST FOR LAIKAGO
            twist_msg_.twist.twist.linear.x = twist_msg_.twist.twist.linear.x/3;
            twist_msg_.twist.twist.linear.y = twist_msg_.twist.twist.linear.y/3;
            twist_msg_.twist.twist.linear.z = twist_msg_.twist.twist.linear.z/9;
            twist_msg_.twist.twist.angular.x = twist_msg_.twist.twist.angular.x/3;
            twist_msg_.twist.twist.angular.y = twist_msg_.twist.twist.angular.y/3;
            twist_msg_.twist.twist.angular.z = twist_msg_.twist.twist.angular.z/3;
            // publish the twist
            twist_pub_.publish(twist_msg_);

            // make sure stuff is non-NAN before publishing
            assert(head_state.position().allFinite());

            // fill in message position
            tf::vectorEigenToTF(head_state.position(), temp_v3);
            tf::pointTFToMsg(temp_v3, pose_msg_.pose.pose.position);

            // fill in message orientation
            tf::quaternionEigenToTF(head_state.orientation(), temp_q);
            tf::quaternionTFToMsg(temp_q,pose_msg_.pose.pose.orientation);

            // fill in time
            pose_msg_.header.stamp = ros::Time().fromNSec(head_state.utime * 1000);
            if(publish_tf_){
                tf_pose_.setOrigin(temp_v3);
                tf_pose_.setRotation(temp_q);
                tf_pose_.stamp_ = ros::Time::now();
                tf_broadcaster_.sendTransform(tf_pose_);
            }

//            std::cout<<"pose msg is:"<<std::endl;
//            std::cout<<pose_msg_<<std::endl;
//            std::cout<<std::endl;

            // TODO insert appropriate covariance into the message
            // publish the pose
            pose_pub_.publish(pose_msg_);
        }
#if DEBUG_MODE
        end = std::chrono::high_resolution_clock::now();

        ROS_INFO_STREAM("Time elapsed till the end: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        std::cout << std::endl;
#endif
    }
}

template <class PrimaryMsgT, class SecondaryMsgT>
void ROSFrontEnd::secondaryCallback(boost::shared_ptr<SecondaryMsgT const> msg,
                                    const SensorId& sensor_id)
{
    auto a = dynamic_cast<DualSensingModule<PrimaryMsgT,SecondaryMsgT>*>(static_cast<SensingModule<PrimaryMsgT>*>(active_modules_[sensor_id]));
    a->processSecondaryMessage(*msg);
}

} // namespace pronto
