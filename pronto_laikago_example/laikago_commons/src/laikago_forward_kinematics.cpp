#include "laikago_forward_kinematics.hpp"
#include <pronto_quadruped_commons/rbd/utils.h>

using namespace pronto::quadruped;
using namespace iit::rbd;

namespace iit {
namespace laikago {

ForwardKinematics::ForwardKinematics()
{
}

Vector3d ForwardKinematics::getFootPosLF(const JointState& q){
//    std::cout<<"###########################"<<std::endl;
//    std::cout<<q<<std::endl;
//    std::cout<<"###########################"<<std::endl;
//    std::cout<<"from the base to lf_hip"<<std::endl;
//    std::cout<<Utils::positionVector(ht_.fr_base_X_fr_LF_HIP(q))<<std::endl;
//    std::cout<<"from the lf_hip to lf_thigh"<<std::endl;
//    std::cout<<Utils::positionVector(ht_.fr_LF_HIP_X_fr_LF_THIGH(q))<<std::endl;
//    std::cout<<"from the lf_thigh to lf_shank(lf_foot)"<<std::endl;
//    std::cout<<Utils::positionVector(ht_.fr_LF_THIGH_X_fr_LF_SHANK(q))<<std::endl;

//    std::cout<<"homogeneous transform matrix:"<<std::endl;
//    std::cout<<ht_.fr_base_X_fr_LF_HIP(q)<<std::endl;
//    std::cout<<ht_.fr_LF_HIP_X_fr_LF_THIGH(q)<<std::endl;
//    std::cout<<ht_.fr_LF_THIGH_X_fr_LF_SHANK(q)<<std::endl;

    //std::cout<<"LF foot_pos in base_link:"<<std::endl;
    //std::cout<<Utils::positionVector(ht_.fr_base_X_fr_LF_HIP(q)*ht_.fr_LF_HIP_X_fr_LF_THIGH(q)*ht_.fr_LF_THIGH_X_fr_LF_SHANK(q)*ht_.fr_SHANK_TO_FOOT)<<std::endl;
    return Utils::positionVector(ht_.fr_base_X_fr_LF_HIP(q)*ht_.fr_LF_HIP_X_fr_LF_THIGH(q)*ht_.fr_LF_THIGH_X_fr_LF_SHANK(q)*ht_.fr_SHANK_TO_FOOT);
}

Vector3d ForwardKinematics::getFootPosRF(const JointState& q){
    //std::cout<<"RF foot_pos in base_link:"<<std::endl;
    //std::cout<<Utils::positionVector(ht_.fr_base_X_fr_RF_HIP(q)*ht_.fr_RF_HIP_X_fr_RF_THIGH(q)*ht_.fr_RF_THIGH_X_fr_RF_SHANK(q)*ht_.fr_SHANK_TO_FOOT)<<std::endl;
    return Utils::positionVector(ht_.fr_base_X_fr_RF_HIP(q)*ht_.fr_RF_HIP_X_fr_RF_THIGH(q)*ht_.fr_RF_THIGH_X_fr_RF_SHANK(q)*ht_.fr_SHANK_TO_FOOT);
}

Vector3d ForwardKinematics::getFootPosLH(const JointState& q){
    //std::cout<<"LH foot_pos in base_link:"<<std::endl;
    //std::cout<<Utils::positionVector(ht_.fr_base_X_fr_LH_HIP(q)*ht_.fr_LH_HIP_X_fr_LH_THIGH(q)*ht_.fr_LH_THIGH_X_fr_LH_SHANK(q)*ht_.fr_SHANK_TO_FOOT)<<std::endl;
    return Utils::positionVector(ht_.fr_base_X_fr_LH_HIP(q)*ht_.fr_LH_HIP_X_fr_LH_THIGH(q)*ht_.fr_LH_THIGH_X_fr_LH_SHANK(q)*ht_.fr_SHANK_TO_FOOT);
}

Vector3d ForwardKinematics::getFootPosRH(const JointState& q){
    //std::cout<<"RH foot_pos in base_link:"<<std::endl;
    //std::cout<<Utils::positionVector(ht_.fr_base_X_fr_RH_HIP(q)*ht_.fr_RH_HIP_X_fr_RH_THIGH(q)*ht_.fr_RH_THIGH_X_fr_RH_SHANK(q)*ht_.fr_SHANK_TO_FOOT)<<std::endl;
    return Utils::positionVector(ht_.fr_base_X_fr_RH_HIP(q)*ht_.fr_RH_HIP_X_fr_RH_THIGH(q)*ht_.fr_RH_THIGH_X_fr_RH_SHANK(q)*ht_.fr_SHANK_TO_FOOT);
}

Vector3d ForwardKinematics::getFootPos(const JointState& q, const LegID& leg){
    switch(leg){
    case LegID::LF:
        return getFootPosLF(q);
    case LegID::RF:
        return getFootPosRF(q);
    case LegID::LH:
        return getFootPosLH(q);
    case LegID::RH:
        return getFootPosRH(q);
    default:
        return Vector3d::Zero();
    }
}

Matrix3d ForwardKinematics::getFootOrientation(const JointState &q, const LegID &leg){

    switch(leg){
    case LegID::LF:
        return Utils::rotationMx(ht_.fr_base_X_fr_LF_HIP);
    case LegID::RF:
        return Utils::rotationMx(ht_.fr_base_X_fr_RF_HIP);
    case LegID::LH:
        return Utils::rotationMx(ht_.fr_base_X_fr_LH_HIP);
    case LegID::RH:
        return Utils::rotationMx(ht_.fr_base_X_fr_RH_HIP);
    default:
        std::cerr << "[ ForwardKinematics::getFootOrientation(...) ] "
                  << "ERROR: legID not recognized. Returning identity."
                  << std::endl;
        return Matrix3d::Identity();
    }
}

Vector3d ForwardKinematics::getShinPos(const JointState& q,
                                               const double& contact_pos,
                                               const LegID& leg){
    return getFootPos(q, leg);
}


}
}
