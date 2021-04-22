#include "pronto_core/pose_meas_module.hpp"

namespace pronto {

//MXR::note
//pose_meas 用于初始化和更新过程中的位姿矫正
PoseMeasModule::PoseMeasModule(const PoseMeasConfig &cfg) :
    mode(cfg.mode), no_corrections(cfg.number_of_corrections)
{
    if (mode == PoseMeasMode::MODE_POSITION) {
        z_indices = RBIS::positionInds();
        z_meas.resize(3);
        cov_pose_meas.resize(3, 3);
        cov_pose_meas = std::pow(cfg.r_pose_meas_xyz, 2) * Eigen::Matrix3d::Identity();
    }
    else if(mode == PoseMeasMode::MODE_POSITION_ORIENT) {
        cov_pose_meas.resize(6, 6);
        cov_pose_meas.setZero();
        cov_pose_meas.topLeftCorner<3, 3>() = std::pow(cfg.r_pose_meas_xyz, 2) * Eigen::Matrix3d::Identity();
        cov_pose_meas.bottomRightCorner<3, 3>() = std::pow((cfg.r_pose_meas_chi) * M_PI / 180.0, 2) * Eigen::Matrix3d::Identity();
        z_meas.resize(6);
        z_indices.resize(6);
        z_indices.head<3>() = RBIS::positionInds();
        z_indices.tail<3>() = RBIS::chiInds();
    }
}

RBISUpdateInterface* PoseMeasModule::processMessage(const PoseMeasurement *msg,
                                                    StateEstimator *est)
{

    std::cout<<"@@@@@@@@@@@@@@@@@"<<std::endl;
    std::cout<<" PoseMeasModule::processMessage ......"<<std::endl;
    std::cout<<"@@@@@@@@@@@@@@@@@"<<std::endl;
    // If we have created no_corrections, go silent afterwards
    //MXR::NOTE: Always do pose correction by foot_odom
    //no_corrections--;

    if (no_corrections == 1) {
      std::cout << "Finished making PoseMeas corrections" << std::endl;
    }

    if (no_corrections <= 0){
        // quietly return invalid update if the max number of corrections has
        // been applied
      return nullptr;
    }

    // filter out quasi-zero corrections
    if ((msg->pos.array().abs() < 1e-5).all()){
      return nullptr;
    }

    switch(mode){
    case PoseMeasMode::MODE_POSITION:
        return new RBISIndexedMeasurement(z_indices,
                                          msg->pos,
                                          cov_pose_meas,
                                          RBISUpdateInterface::pose_meas,
                                          msg->utime);
    case PoseMeasMode::MODE_POSITION_ORIENT:
        z_meas.head<3>() = msg->pos;
        return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                         z_meas,
                                                         cov_pose_meas,
                                                         msg->orientation,
                                                         RBISUpdateInterface::pose_meas,
                                                         msg->utime);
    default:
        return nullptr;
    }
}

bool PoseMeasModule::processMessageInit(const PoseMeasurement *msg,
                                        const std::map<std::string, bool> &sensor_initialized,
                                        const RBIS &default_state,
                                        const RBIM &default_cov,
                                        RBIS &init_state,
                                        RBIM &init_cov)
{
    //MXR::note
    //this module used for initilization
    std::cout<<"@@@@@@@@@@@@@@@@@"<<std::endl;
    std::cout<<"pose means module ..............."<<std::endl;
    std::cout<<"@@@@@@@@@@@@@@@@@"<<std::endl;
    init_state.utime = msg->utime;
    init_state.position() = msg->pos;
    init_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = cov_pose_meas.topLeftCorner<3, 3>();

    if(mode == PoseMeasMode::MODE_POSITION_ORIENT)
    {
        init_state.orientation() = msg->orientation;
        init_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = cov_pose_meas.bottomRightCorner<3, 3>();
    }


    Eigen::Vector3d init_rpy_deg = (init_state.getEulerAngles())*180.0 / M_PI;

    std::cout << "Initialized position using a pose_t at xyz: "
              << init_state.position().transpose() << std::endl;

    std::cout << "Initialized orientation using a pose_t at rpy: "
              << init_rpy_deg.transpose() << std::endl;

    return true;
}
} // namespace pronto
