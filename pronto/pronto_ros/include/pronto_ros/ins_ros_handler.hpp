#pragma once
#include <functional>
#include <sensor_msgs/Imu.h>
#include <ros/node_handle.h>
#include <pronto_core/ins_module.hpp>

namespace pronto {
class InsHandlerROS : public SensingModule<sensor_msgs::Imu> {
public:
    InsHandlerROS(ros::NodeHandle& nh);
//在最顶层的虚函数上加上virtual关键字后，其余的子类覆写后就不再加virtual了，但是要统一加上override。
    RBISUpdateInterface* processMessage(const sensor_msgs::Imu *imu_msg, StateEstimator *est) override;

    bool processMessageInit(const sensor_msgs::Imu *imu_msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;


private:
    ros::NodeHandle& nh_;
    ImuMeasurement imu_meas_;
    InsModule ins_module_;
    uint64_t counter = 0;
    uint16_t downsample_factor_ = 1;
    bool initialized_ = false;
    std::string imu_topic_ = "/sensors/imu";
    bool roll_forward_on_receive_ = true;
    int64_t utime_offset_ = 0;
};
}
