#pragma once
#include "pronto_core/sensing_module.hpp"
#include "pronto_core/definitions.hpp"
namespace pronto {

enum class ViconMode {MODE_POSITION,
                      MODE_POSITION_ORIENT,
                      MODE_ORIENTATION, MODE_YAW};

struct ViconConfig{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ViconMode mode;
    Transform body_to_vicon = Transform::Identity();
    /**
     * @brief r_vicon_xyz standard deviation for position in meters
     */
    double r_vicon_xyz;
    /**
     * @brief r_vicon_chi standard deviation for orientation (ZYX Euler) in degrees
     */
    double r_vicon_chi;
};

class ViconModule : public SensingModule<RigidTransform> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    // the measurement and index sizes will be 1, 3 or 6
    // tell Eigen that the max size at compile time is 6 per dimension


    //Options是一个比特标志位，这里，我们只介绍一种RowMajor，它表明matrix使用按行存储，默认是按列存储。Matrix<float, 3, 3, RowMajor>
    //MaxRowsAtCompileTime和MaxColsAtCompileTime表示在编译阶段矩阵的上限。主要是避免动态内存分配，使用数组。
    //Matrix<float, Dynamic, Dynamic, 0, 3, 4> 等价于 float [12]

    using MeasVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 6, 1>;
    using IndexVector = Eigen::Matrix<int, Eigen::Dynamic, 1, 0, 6, 1>;
    using CovMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6 ,6>;
public:
    ViconModule(const ViconConfig& cfg);

    RBISUpdateInterface* processMessage(const RigidTransform *msg,
                                        StateEstimator *est) override;

    bool processMessageInit(const RigidTransform *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state, RBIM &init_cov) override;
protected:
    ViconMode mode;
    Transform body_to_vicon = Transform::Identity();
    Transform local_to_vicon;
    Transform local_to_body;

    IndexVector z_indices;
    MeasVector z_meas;
    CovMatrix cov_vicon;
};
}
