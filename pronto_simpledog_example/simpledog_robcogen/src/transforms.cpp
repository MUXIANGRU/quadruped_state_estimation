#include "transforms.h"

using namespace iit::simpledog;

// Constructors

MotionTransforms::MotionTransforms()
 :     fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_HIP(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK()
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_HIP(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK()
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_HIP(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK()
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(2,1) = 0.0;
    (*this)(2,2) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) =  ty_LF_HAA * cos_ry_LF_HAA;    // Maxima DSL: _k__ty_LF_HAA*cos(_k__ry_LF_HAA)
    (*this)(5,1) = ( tz_LF_HAA * sin_ry_LF_HAA)-( tx_LF_HAA * cos_ry_LF_HAA);    // Maxima DSL: _k__tz_LF_HAA*sin(_k__ry_LF_HAA)-_k__tx_LF_HAA*cos(_k__ry_LF_HAA)
    (*this)(5,2) = - ty_LF_HAA * sin_ry_LF_HAA;    // Maxima DSL: -_k__ty_LF_HAA*sin(_k__ry_LF_HAA)
    (*this)(5,3) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(5,4) = 0.0;
    (*this)(5,5) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
}

const MotionTransforms::Type_fr_LF_HIP_X_fr_base& MotionTransforms::Type_fr_LF_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_LF_HAA  = ScalarTraits::sin( q(LF_HAA) );
    Scalar cos_q_LF_HAA  = ScalarTraits::cos( q(LF_HAA) );
    (*this)(0,0) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(0,1) = sin_q_LF_HAA;
    (*this)(0,2) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(1,0) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(1,1) = cos_q_LF_HAA;
    (*this)(1,2) = sin_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(3,0) = (- tz_LF_HAA * sin_q_LF_HAA)-( ty_LF_HAA * sin_ry_LF_HAA * cos_q_LF_HAA);
    (*this)(3,1) = (( tx_LF_HAA * sin_ry_LF_HAA)+( tz_LF_HAA * cos_ry_LF_HAA)) * cos_q_LF_HAA;
    (*this)(3,2) = ( tx_LF_HAA * sin_q_LF_HAA)-( ty_LF_HAA * cos_ry_LF_HAA * cos_q_LF_HAA);
    (*this)(3,3) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(3,4) = sin_q_LF_HAA;
    (*this)(3,5) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(4,0) = ( ty_LF_HAA * sin_ry_LF_HAA * sin_q_LF_HAA)-( tz_LF_HAA * cos_q_LF_HAA);
    (*this)(4,1) = ((- tx_LF_HAA * sin_ry_LF_HAA)-( tz_LF_HAA * cos_ry_LF_HAA)) * sin_q_LF_HAA;
    (*this)(4,2) = ( ty_LF_HAA * cos_ry_LF_HAA * sin_q_LF_HAA)+( tx_LF_HAA * cos_q_LF_HAA);
    (*this)(4,3) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(4,4) = cos_q_LF_HAA;
    (*this)(4,5) = sin_ry_LF_HAA * sin_q_LF_HAA;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
{
    (*this)(0,2) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) =  ty_LF_HAA * cos_ry_LF_HAA;    // Maxima DSL: _k__ty_LF_HAA*cos(_k__ry_LF_HAA)
    (*this)(3,5) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(4,2) = ( tz_LF_HAA * sin_ry_LF_HAA)-( tx_LF_HAA * cos_ry_LF_HAA);    // Maxima DSL: _k__tz_LF_HAA*sin(_k__ry_LF_HAA)-_k__tx_LF_HAA*cos(_k__ry_LF_HAA)
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_LF_HAA * sin_ry_LF_HAA;    // Maxima DSL: -_k__ty_LF_HAA*sin(_k__ry_LF_HAA)
    (*this)(5,5) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
}

const MotionTransforms::Type_fr_base_X_fr_LF_HIP& MotionTransforms::Type_fr_base_X_fr_LF_HIP::update(const state_t& q)
{
    Scalar sin_q_LF_HAA  = ScalarTraits::sin( q(LF_HAA) );
    Scalar cos_q_LF_HAA  = ScalarTraits::cos( q(LF_HAA) );
    (*this)(0,0) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(0,1) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(1,0) = sin_q_LF_HAA;
    (*this)(1,1) = cos_q_LF_HAA;
    (*this)(2,0) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(2,1) = sin_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(3,0) = (- tz_LF_HAA * sin_q_LF_HAA)-( ty_LF_HAA * sin_ry_LF_HAA * cos_q_LF_HAA);
    (*this)(3,1) = ( ty_LF_HAA * sin_ry_LF_HAA * sin_q_LF_HAA)-( tz_LF_HAA * cos_q_LF_HAA);
    (*this)(3,3) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(3,4) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(4,0) = (( tx_LF_HAA * sin_ry_LF_HAA)+( tz_LF_HAA * cos_ry_LF_HAA)) * cos_q_LF_HAA;
    (*this)(4,1) = ((- tx_LF_HAA * sin_ry_LF_HAA)-( tz_LF_HAA * cos_ry_LF_HAA)) * sin_q_LF_HAA;
    (*this)(4,3) = sin_q_LF_HAA;
    (*this)(4,4) = cos_q_LF_HAA;
    (*this)(5,0) = ( tx_LF_HAA * sin_q_LF_HAA)-( ty_LF_HAA * cos_ry_LF_HAA * cos_q_LF_HAA);
    (*this)(5,1) = ( ty_LF_HAA * cos_ry_LF_HAA * sin_q_LF_HAA)+( tx_LF_HAA * cos_q_LF_HAA);
    (*this)(5,3) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(5,4) = sin_ry_LF_HAA * sin_q_LF_HAA;
    return *this;
}
MotionTransforms::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(2,2) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(5,5) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
}

const MotionTransforms::Type_fr_LF_THIGH_X_fr_LF_HIP& MotionTransforms::Type_fr_LF_THIGH_X_fr_LF_HIP::update(const state_t& q)
{
    Scalar sin_q_LF_HFE  = ScalarTraits::sin( q(LF_HFE) );
    Scalar cos_q_LF_HFE  = ScalarTraits::cos( q(LF_HFE) );
    (*this)(0,0) = cos_q_LF_HFE;
    (*this)(0,1) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(0,2) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(1,0) = -sin_q_LF_HFE;
    (*this)(1,1) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(1,2) = sin_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(3,3) = cos_q_LF_HFE;
    (*this)(3,4) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(3,5) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(4,3) = -sin_q_LF_HFE;
    (*this)(4,4) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(4,5) = sin_rx_LF_HFE * cos_q_LF_HFE;
    return *this;
}
MotionTransforms::Type_fr_LF_HIP_X_fr_LF_THIGH::Type_fr_LF_HIP_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
}

const MotionTransforms::Type_fr_LF_HIP_X_fr_LF_THIGH& MotionTransforms::Type_fr_LF_HIP_X_fr_LF_THIGH::update(const state_t& q)
{
    Scalar sin_q_LF_HFE  = ScalarTraits::sin( q(LF_HFE) );
    Scalar cos_q_LF_HFE  = ScalarTraits::cos( q(LF_HFE) );
    (*this)(0,0) = cos_q_LF_HFE;
    (*this)(0,1) = -sin_q_LF_HFE;
    (*this)(1,0) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(1,1) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(2,0) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(2,1) = sin_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(3,3) = cos_q_LF_HFE;
    (*this)(3,4) = -sin_q_LF_HFE;
    (*this)(4,3) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(4,4) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(5,3) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(5,4) = sin_rx_LF_HFE * cos_q_LF_HFE;
    return *this;
}
MotionTransforms::Type_fr_LF_SHANK_X_fr_LF_THIGH::Type_fr_LF_SHANK_X_fr_LF_THIGH()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(2,2) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) =  ty_LF_KFE * cos_rx_LF_KFE;    // Maxima DSL: _k__ty_LF_KFE*cos(_k__rx_LF_KFE)
    (*this)(5,1) = - tx_LF_KFE * cos_rx_LF_KFE;    // Maxima DSL: -_k__tx_LF_KFE*cos(_k__rx_LF_KFE)
    (*this)(5,2) = - tx_LF_KFE * sin_rx_LF_KFE;    // Maxima DSL: -_k__tx_LF_KFE*sin(_k__rx_LF_KFE)
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(5,5) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
}

const MotionTransforms::Type_fr_LF_SHANK_X_fr_LF_THIGH& MotionTransforms::Type_fr_LF_SHANK_X_fr_LF_THIGH::update(const state_t& q)
{
    Scalar sin_q_LF_KFE  = ScalarTraits::sin( q(LF_KFE) );
    Scalar cos_q_LF_KFE  = ScalarTraits::cos( q(LF_KFE) );
    (*this)(0,0) = cos_q_LF_KFE;
    (*this)(0,1) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(0,2) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(1,0) = -sin_q_LF_KFE;
    (*this)(1,1) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(1,2) = sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(3,0) =  ty_LF_KFE * sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(3,1) = - tx_LF_KFE * sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(3,2) = ( tx_LF_KFE * cos_rx_LF_KFE * sin_q_LF_KFE)-( ty_LF_KFE * cos_q_LF_KFE);
    (*this)(3,3) = cos_q_LF_KFE;
    (*this)(3,4) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(3,5) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(4,0) =  ty_LF_KFE * sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(4,1) = - tx_LF_KFE * sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(4,2) = ( ty_LF_KFE * sin_q_LF_KFE)+( tx_LF_KFE * cos_rx_LF_KFE * cos_q_LF_KFE);
    (*this)(4,3) = -sin_q_LF_KFE;
    (*this)(4,4) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(4,5) = sin_rx_LF_KFE * cos_q_LF_KFE;
    return *this;
}
MotionTransforms::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) =  ty_LF_KFE * cos_rx_LF_KFE;    // Maxima DSL: _k__ty_LF_KFE*cos(_k__rx_LF_KFE)
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_LF_KFE * cos_rx_LF_KFE;    // Maxima DSL: -_k__tx_LF_KFE*cos(_k__rx_LF_KFE)
    (*this)(4,5) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(5,2) = - tx_LF_KFE * sin_rx_LF_KFE;    // Maxima DSL: -_k__tx_LF_KFE*sin(_k__rx_LF_KFE)
    (*this)(5,5) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
}

const MotionTransforms::Type_fr_LF_THIGH_X_fr_LF_SHANK& MotionTransforms::Type_fr_LF_THIGH_X_fr_LF_SHANK::update(const state_t& q)
{
    Scalar sin_q_LF_KFE  = ScalarTraits::sin( q(LF_KFE) );
    Scalar cos_q_LF_KFE  = ScalarTraits::cos( q(LF_KFE) );
    (*this)(0,0) = cos_q_LF_KFE;
    (*this)(0,1) = -sin_q_LF_KFE;
    (*this)(1,0) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(1,1) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(2,0) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(2,1) = sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(3,0) =  ty_LF_KFE * sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(3,1) =  ty_LF_KFE * sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(3,3) = cos_q_LF_KFE;
    (*this)(3,4) = -sin_q_LF_KFE;
    (*this)(4,0) = - tx_LF_KFE * sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(4,1) = - tx_LF_KFE * sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(4,3) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(4,4) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(5,0) = ( tx_LF_KFE * cos_rx_LF_KFE * sin_q_LF_KFE)-( ty_LF_KFE * cos_q_LF_KFE);
    (*this)(5,1) = ( ty_LF_KFE * sin_q_LF_KFE)+( tx_LF_KFE * cos_rx_LF_KFE * cos_q_LF_KFE);
    (*this)(5,3) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(5,4) = sin_rx_LF_KFE * cos_q_LF_KFE;
    return *this;
}
MotionTransforms::Type_fr_RF_HIP_X_fr_base::Type_fr_RF_HIP_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(2,1) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,2) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = (( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * cos_ry_RF_HAA;    // Maxima DSL: (_k__tz_RF_HAA*sin(_k__rx_RF_HAA)+_k__ty_RF_HAA*cos(_k__rx_RF_HAA))*cos(_k__ry_RF_HAA)
    (*this)(5,1) = ( tz_RF_HAA * sin_ry_RF_HAA)-( tx_RF_HAA * cos_rx_RF_HAA * cos_ry_RF_HAA);    // Maxima DSL: _k__tz_RF_HAA*sin(_k__ry_RF_HAA)-_k__tx_RF_HAA*cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(5,2) = (- ty_RF_HAA * sin_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_ry_RF_HAA);    // Maxima DSL: (-_k__ty_RF_HAA*sin(_k__ry_RF_HAA))-_k__tx_RF_HAA*sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(5,3) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(5,4) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(5,5) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
}

const MotionTransforms::Type_fr_RF_HIP_X_fr_base& MotionTransforms::Type_fr_RF_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RF_HAA  = ScalarTraits::sin( q(RF_HAA) );
    Scalar cos_q_RF_HAA  = ScalarTraits::cos( q(RF_HAA) );
    (*this)(0,0) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(0,1) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(0,2) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,0) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(1,1) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,2) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,0) = ((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((((- tz_RF_HAA * sin_rx_RF_HAA)-( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,1) = (((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_rz_RF_HAA)) * sin_q_RF_HAA)+((((( tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)+( tz_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,2) = ((( tx_RF_HAA * cos_rx_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((((( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)-( ty_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+( tx_RF_HAA * cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,3) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(3,4) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,5) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,0) = ((((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * cos_rz_RF_HAA)+((( tz_RF_HAA * cos_rx_RF_HAA)-( ty_RF_HAA * sin_rx_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,1) = (((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+( tx_RF_HAA * sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,2) = ((((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * cos_rz_RF_HAA)-( tx_RF_HAA * cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((( tx_RF_HAA * cos_rx_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,3) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(4,4) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,5) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
{
    (*this)(0,2) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = (( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * cos_ry_RF_HAA;    // Maxima DSL: (_k__tz_RF_HAA*sin(_k__rx_RF_HAA)+_k__ty_RF_HAA*cos(_k__rx_RF_HAA))*cos(_k__ry_RF_HAA)
    (*this)(3,5) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(4,2) = ( tz_RF_HAA * sin_ry_RF_HAA)-( tx_RF_HAA * cos_rx_RF_HAA * cos_ry_RF_HAA);    // Maxima DSL: _k__tz_RF_HAA*sin(_k__ry_RF_HAA)-_k__tx_RF_HAA*cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(4,5) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(5,2) = (- ty_RF_HAA * sin_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_ry_RF_HAA);    // Maxima DSL: (-_k__ty_RF_HAA*sin(_k__ry_RF_HAA))-_k__tx_RF_HAA*sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(5,5) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
}

const MotionTransforms::Type_fr_base_X_fr_RF_HIP& MotionTransforms::Type_fr_base_X_fr_RF_HIP::update(const state_t& q)
{
    Scalar sin_q_RF_HAA  = ScalarTraits::sin( q(RF_HAA) );
    Scalar cos_q_RF_HAA  = ScalarTraits::cos( q(RF_HAA) );
    (*this)(0,0) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(0,1) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(1,0) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,1) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(2,0) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(2,1) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,0) = ((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((((- tz_RF_HAA * sin_rx_RF_HAA)-( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,1) = ((((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * cos_rz_RF_HAA)+((( tz_RF_HAA * cos_rx_RF_HAA)-( ty_RF_HAA * sin_rx_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,3) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(3,4) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(4,0) = (((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_rz_RF_HAA)) * sin_q_RF_HAA)+((((( tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)+( tz_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,1) = (((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+( tx_RF_HAA * sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,3) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,4) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(5,0) = ((( tx_RF_HAA * cos_rx_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((((( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)-( ty_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+( tx_RF_HAA * cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(5,1) = ((((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * cos_rz_RF_HAA)-( tx_RF_HAA * cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((( tx_RF_HAA * cos_rx_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(5,3) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(5,4) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    return *this;
}
MotionTransforms::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(2,2) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(5,5) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
}

const MotionTransforms::Type_fr_RF_THIGH_X_fr_RF_HIP& MotionTransforms::Type_fr_RF_THIGH_X_fr_RF_HIP::update(const state_t& q)
{
    Scalar sin_q_RF_HFE  = ScalarTraits::sin( q(RF_HFE) );
    Scalar cos_q_RF_HFE  = ScalarTraits::cos( q(RF_HFE) );
    (*this)(0,0) = cos_q_RF_HFE;
    (*this)(0,1) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(0,2) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(1,0) = -sin_q_RF_HFE;
    (*this)(1,1) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(1,2) = sin_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(3,3) = cos_q_RF_HFE;
    (*this)(3,4) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(3,5) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(4,3) = -sin_q_RF_HFE;
    (*this)(4,4) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(4,5) = sin_rx_RF_HFE * cos_q_RF_HFE;
    return *this;
}
MotionTransforms::Type_fr_RF_HIP_X_fr_RF_THIGH::Type_fr_RF_HIP_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
}

const MotionTransforms::Type_fr_RF_HIP_X_fr_RF_THIGH& MotionTransforms::Type_fr_RF_HIP_X_fr_RF_THIGH::update(const state_t& q)
{
    Scalar sin_q_RF_HFE  = ScalarTraits::sin( q(RF_HFE) );
    Scalar cos_q_RF_HFE  = ScalarTraits::cos( q(RF_HFE) );
    (*this)(0,0) = cos_q_RF_HFE;
    (*this)(0,1) = -sin_q_RF_HFE;
    (*this)(1,0) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(1,1) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(2,0) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(2,1) = sin_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(3,3) = cos_q_RF_HFE;
    (*this)(3,4) = -sin_q_RF_HFE;
    (*this)(4,3) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(4,4) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(5,3) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(5,4) = sin_rx_RF_HFE * cos_q_RF_HFE;
    return *this;
}
MotionTransforms::Type_fr_RF_SHANK_X_fr_RF_THIGH::Type_fr_RF_SHANK_X_fr_RF_THIGH()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(2,2) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_RF_KFE * cos_rx_RF_KFE;    // Maxima DSL: -_k__tx_RF_KFE*cos(_k__rx_RF_KFE)
    (*this)(5,2) = - tx_RF_KFE * sin_rx_RF_KFE;    // Maxima DSL: -_k__tx_RF_KFE*sin(_k__rx_RF_KFE)
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(5,5) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
}

const MotionTransforms::Type_fr_RF_SHANK_X_fr_RF_THIGH& MotionTransforms::Type_fr_RF_SHANK_X_fr_RF_THIGH::update(const state_t& q)
{
    Scalar sin_q_RF_KFE  = ScalarTraits::sin( q(RF_KFE) );
    Scalar cos_q_RF_KFE  = ScalarTraits::cos( q(RF_KFE) );
    (*this)(0,0) = cos_q_RF_KFE;
    (*this)(0,1) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(0,2) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(1,0) = -sin_q_RF_KFE;
    (*this)(1,1) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(1,2) = sin_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(3,1) = - tx_RF_KFE * sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(3,2) =  tx_RF_KFE * cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(3,3) = cos_q_RF_KFE;
    (*this)(3,4) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(3,5) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(4,1) = - tx_RF_KFE * sin_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(4,2) =  tx_RF_KFE * cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(4,3) = -sin_q_RF_KFE;
    (*this)(4,4) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(4,5) = sin_rx_RF_KFE * cos_q_RF_KFE;
    return *this;
}
MotionTransforms::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_RF_KFE * cos_rx_RF_KFE;    // Maxima DSL: -_k__tx_RF_KFE*cos(_k__rx_RF_KFE)
    (*this)(4,5) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(5,2) = - tx_RF_KFE * sin_rx_RF_KFE;    // Maxima DSL: -_k__tx_RF_KFE*sin(_k__rx_RF_KFE)
    (*this)(5,5) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
}

const MotionTransforms::Type_fr_RF_THIGH_X_fr_RF_SHANK& MotionTransforms::Type_fr_RF_THIGH_X_fr_RF_SHANK::update(const state_t& q)
{
    Scalar sin_q_RF_KFE  = ScalarTraits::sin( q(RF_KFE) );
    Scalar cos_q_RF_KFE  = ScalarTraits::cos( q(RF_KFE) );
    (*this)(0,0) = cos_q_RF_KFE;
    (*this)(0,1) = -sin_q_RF_KFE;
    (*this)(1,0) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(1,1) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(2,0) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(2,1) = sin_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(3,3) = cos_q_RF_KFE;
    (*this)(3,4) = -sin_q_RF_KFE;
    (*this)(4,0) = - tx_RF_KFE * sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(4,1) = - tx_RF_KFE * sin_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(4,3) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(4,4) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(5,0) =  tx_RF_KFE * cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(5,1) =  tx_RF_KFE * cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(5,3) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(5,4) = sin_rx_RF_KFE * cos_q_RF_KFE;
    return *this;
}
MotionTransforms::Type_fr_RH_HIP_X_fr_base::Type_fr_RH_HIP_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(2,1) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,2) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = (( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * cos_ry_RH_HAA;    // Maxima DSL: (_k__tz_RH_HAA*sin(_k__rx_RH_HAA)+_k__ty_RH_HAA*cos(_k__rx_RH_HAA))*cos(_k__ry_RH_HAA)
    (*this)(5,1) = ( tz_RH_HAA * sin_ry_RH_HAA)-( tx_RH_HAA * cos_rx_RH_HAA * cos_ry_RH_HAA);    // Maxima DSL: _k__tz_RH_HAA*sin(_k__ry_RH_HAA)-_k__tx_RH_HAA*cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(5,2) = (- ty_RH_HAA * sin_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_ry_RH_HAA);    // Maxima DSL: (-_k__ty_RH_HAA*sin(_k__ry_RH_HAA))-_k__tx_RH_HAA*sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(5,3) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(5,4) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(5,5) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
}

const MotionTransforms::Type_fr_RH_HIP_X_fr_base& MotionTransforms::Type_fr_RH_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RH_HAA  = ScalarTraits::sin( q(RH_HAA) );
    Scalar cos_q_RH_HAA  = ScalarTraits::cos( q(RH_HAA) );
    (*this)(0,0) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(0,1) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(0,2) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,0) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(1,1) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,2) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,0) = ((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((((- tz_RH_HAA * sin_rx_RH_HAA)-( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,1) = (((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_rz_RH_HAA)) * sin_q_RH_HAA)+((((( tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)+( tz_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,2) = ((( tx_RH_HAA * cos_rx_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((((( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)-( ty_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+( tx_RH_HAA * cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,3) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(3,4) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,5) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,0) = ((((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * cos_rz_RH_HAA)+((( tz_RH_HAA * cos_rx_RH_HAA)-( ty_RH_HAA * sin_rx_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,1) = (((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+( tx_RH_HAA * sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,2) = ((((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * cos_rz_RH_HAA)-( tx_RH_HAA * cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((( tx_RH_HAA * cos_rx_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,3) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(4,4) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,5) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
{
    (*this)(0,2) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = (( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * cos_ry_RH_HAA;    // Maxima DSL: (_k__tz_RH_HAA*sin(_k__rx_RH_HAA)+_k__ty_RH_HAA*cos(_k__rx_RH_HAA))*cos(_k__ry_RH_HAA)
    (*this)(3,5) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(4,2) = ( tz_RH_HAA * sin_ry_RH_HAA)-( tx_RH_HAA * cos_rx_RH_HAA * cos_ry_RH_HAA);    // Maxima DSL: _k__tz_RH_HAA*sin(_k__ry_RH_HAA)-_k__tx_RH_HAA*cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(4,5) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(5,2) = (- ty_RH_HAA * sin_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_ry_RH_HAA);    // Maxima DSL: (-_k__ty_RH_HAA*sin(_k__ry_RH_HAA))-_k__tx_RH_HAA*sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(5,5) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
}

const MotionTransforms::Type_fr_base_X_fr_RH_HIP& MotionTransforms::Type_fr_base_X_fr_RH_HIP::update(const state_t& q)
{
    Scalar sin_q_RH_HAA  = ScalarTraits::sin( q(RH_HAA) );
    Scalar cos_q_RH_HAA  = ScalarTraits::cos( q(RH_HAA) );
    (*this)(0,0) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(0,1) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(1,0) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,1) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(2,0) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(2,1) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,0) = ((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((((- tz_RH_HAA * sin_rx_RH_HAA)-( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,1) = ((((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * cos_rz_RH_HAA)+((( tz_RH_HAA * cos_rx_RH_HAA)-( ty_RH_HAA * sin_rx_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,3) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(3,4) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(4,0) = (((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_rz_RH_HAA)) * sin_q_RH_HAA)+((((( tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)+( tz_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,1) = (((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+( tx_RH_HAA * sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,3) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,4) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(5,0) = ((( tx_RH_HAA * cos_rx_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((((( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)-( ty_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+( tx_RH_HAA * cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(5,1) = ((((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * cos_rz_RH_HAA)-( tx_RH_HAA * cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((( tx_RH_HAA * cos_rx_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(5,3) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(5,4) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    return *this;
}
MotionTransforms::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(2,2) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(5,5) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
}

const MotionTransforms::Type_fr_RH_THIGH_X_fr_RH_HIP& MotionTransforms::Type_fr_RH_THIGH_X_fr_RH_HIP::update(const state_t& q)
{
    Scalar sin_q_RH_HFE  = ScalarTraits::sin( q(RH_HFE) );
    Scalar cos_q_RH_HFE  = ScalarTraits::cos( q(RH_HFE) );
    (*this)(0,0) = cos_q_RH_HFE;
    (*this)(0,1) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(0,2) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(1,0) = -sin_q_RH_HFE;
    (*this)(1,1) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(1,2) = sin_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(3,3) = cos_q_RH_HFE;
    (*this)(3,4) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(3,5) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(4,3) = -sin_q_RH_HFE;
    (*this)(4,4) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(4,5) = sin_rx_RH_HFE * cos_q_RH_HFE;
    return *this;
}
MotionTransforms::Type_fr_RH_HIP_X_fr_RH_THIGH::Type_fr_RH_HIP_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
}

const MotionTransforms::Type_fr_RH_HIP_X_fr_RH_THIGH& MotionTransforms::Type_fr_RH_HIP_X_fr_RH_THIGH::update(const state_t& q)
{
    Scalar sin_q_RH_HFE  = ScalarTraits::sin( q(RH_HFE) );
    Scalar cos_q_RH_HFE  = ScalarTraits::cos( q(RH_HFE) );
    (*this)(0,0) = cos_q_RH_HFE;
    (*this)(0,1) = -sin_q_RH_HFE;
    (*this)(1,0) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(1,1) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(2,0) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(2,1) = sin_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(3,3) = cos_q_RH_HFE;
    (*this)(3,4) = -sin_q_RH_HFE;
    (*this)(4,3) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(4,4) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(5,3) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(5,4) = sin_rx_RH_HFE * cos_q_RH_HFE;
    return *this;
}
MotionTransforms::Type_fr_RH_SHANK_X_fr_RH_THIGH::Type_fr_RH_SHANK_X_fr_RH_THIGH()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(2,2) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_RH_KFE * cos_rx_RH_KFE;    // Maxima DSL: -_k__tx_RH_KFE*cos(_k__rx_RH_KFE)
    (*this)(5,2) = - tx_RH_KFE * sin_rx_RH_KFE;    // Maxima DSL: -_k__tx_RH_KFE*sin(_k__rx_RH_KFE)
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(5,5) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
}

const MotionTransforms::Type_fr_RH_SHANK_X_fr_RH_THIGH& MotionTransforms::Type_fr_RH_SHANK_X_fr_RH_THIGH::update(const state_t& q)
{
    Scalar sin_q_RH_KFE  = ScalarTraits::sin( q(RH_KFE) );
    Scalar cos_q_RH_KFE  = ScalarTraits::cos( q(RH_KFE) );
    (*this)(0,0) = cos_q_RH_KFE;
    (*this)(0,1) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(0,2) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(1,0) = -sin_q_RH_KFE;
    (*this)(1,1) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(1,2) = sin_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(3,1) = - tx_RH_KFE * sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(3,2) =  tx_RH_KFE * cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(3,3) = cos_q_RH_KFE;
    (*this)(3,4) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(3,5) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(4,1) = - tx_RH_KFE * sin_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(4,2) =  tx_RH_KFE * cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(4,3) = -sin_q_RH_KFE;
    (*this)(4,4) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(4,5) = sin_rx_RH_KFE * cos_q_RH_KFE;
    return *this;
}
MotionTransforms::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_RH_KFE * cos_rx_RH_KFE;    // Maxima DSL: -_k__tx_RH_KFE*cos(_k__rx_RH_KFE)
    (*this)(4,5) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(5,2) = - tx_RH_KFE * sin_rx_RH_KFE;    // Maxima DSL: -_k__tx_RH_KFE*sin(_k__rx_RH_KFE)
    (*this)(5,5) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
}

const MotionTransforms::Type_fr_RH_THIGH_X_fr_RH_SHANK& MotionTransforms::Type_fr_RH_THIGH_X_fr_RH_SHANK::update(const state_t& q)
{
    Scalar sin_q_RH_KFE  = ScalarTraits::sin( q(RH_KFE) );
    Scalar cos_q_RH_KFE  = ScalarTraits::cos( q(RH_KFE) );
    (*this)(0,0) = cos_q_RH_KFE;
    (*this)(0,1) = -sin_q_RH_KFE;
    (*this)(1,0) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(1,1) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(2,0) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(2,1) = sin_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(3,3) = cos_q_RH_KFE;
    (*this)(3,4) = -sin_q_RH_KFE;
    (*this)(4,0) = - tx_RH_KFE * sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(4,1) = - tx_RH_KFE * sin_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(4,3) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(4,4) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(5,0) =  tx_RH_KFE * cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(5,1) =  tx_RH_KFE * cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(5,3) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(5,4) = sin_rx_RH_KFE * cos_q_RH_KFE;
    return *this;
}
MotionTransforms::Type_fr_LH_HIP_X_fr_base::Type_fr_LH_HIP_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(2,1) = 0.0;
    (*this)(2,2) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) =  ty_LH_HAA * cos_ry_LH_HAA;    // Maxima DSL: _k__ty_LH_HAA*cos(_k__ry_LH_HAA)
    (*this)(5,1) = ( tz_LH_HAA * sin_ry_LH_HAA)-( tx_LH_HAA * cos_ry_LH_HAA);    // Maxima DSL: _k__tz_LH_HAA*sin(_k__ry_LH_HAA)-_k__tx_LH_HAA*cos(_k__ry_LH_HAA)
    (*this)(5,2) = - ty_LH_HAA * sin_ry_LH_HAA;    // Maxima DSL: -_k__ty_LH_HAA*sin(_k__ry_LH_HAA)
    (*this)(5,3) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(5,4) = 0.0;
    (*this)(5,5) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
}

const MotionTransforms::Type_fr_LH_HIP_X_fr_base& MotionTransforms::Type_fr_LH_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_LH_HAA  = ScalarTraits::sin( q(LH_HAA) );
    Scalar cos_q_LH_HAA  = ScalarTraits::cos( q(LH_HAA) );
    (*this)(0,0) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(0,1) = sin_q_LH_HAA;
    (*this)(0,2) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(1,0) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(1,1) = cos_q_LH_HAA;
    (*this)(1,2) = sin_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(3,0) = (- tz_LH_HAA * sin_q_LH_HAA)-( ty_LH_HAA * sin_ry_LH_HAA * cos_q_LH_HAA);
    (*this)(3,1) = (( tx_LH_HAA * sin_ry_LH_HAA)+( tz_LH_HAA * cos_ry_LH_HAA)) * cos_q_LH_HAA;
    (*this)(3,2) = ( tx_LH_HAA * sin_q_LH_HAA)-( ty_LH_HAA * cos_ry_LH_HAA * cos_q_LH_HAA);
    (*this)(3,3) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(3,4) = sin_q_LH_HAA;
    (*this)(3,5) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(4,0) = ( ty_LH_HAA * sin_ry_LH_HAA * sin_q_LH_HAA)-( tz_LH_HAA * cos_q_LH_HAA);
    (*this)(4,1) = ((- tx_LH_HAA * sin_ry_LH_HAA)-( tz_LH_HAA * cos_ry_LH_HAA)) * sin_q_LH_HAA;
    (*this)(4,2) = ( ty_LH_HAA * cos_ry_LH_HAA * sin_q_LH_HAA)+( tx_LH_HAA * cos_q_LH_HAA);
    (*this)(4,3) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(4,4) = cos_q_LH_HAA;
    (*this)(4,5) = sin_ry_LH_HAA * sin_q_LH_HAA;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
{
    (*this)(0,2) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) =  ty_LH_HAA * cos_ry_LH_HAA;    // Maxima DSL: _k__ty_LH_HAA*cos(_k__ry_LH_HAA)
    (*this)(3,5) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(4,2) = ( tz_LH_HAA * sin_ry_LH_HAA)-( tx_LH_HAA * cos_ry_LH_HAA);    // Maxima DSL: _k__tz_LH_HAA*sin(_k__ry_LH_HAA)-_k__tx_LH_HAA*cos(_k__ry_LH_HAA)
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_LH_HAA * sin_ry_LH_HAA;    // Maxima DSL: -_k__ty_LH_HAA*sin(_k__ry_LH_HAA)
    (*this)(5,5) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
}

const MotionTransforms::Type_fr_base_X_fr_LH_HIP& MotionTransforms::Type_fr_base_X_fr_LH_HIP::update(const state_t& q)
{
    Scalar sin_q_LH_HAA  = ScalarTraits::sin( q(LH_HAA) );
    Scalar cos_q_LH_HAA  = ScalarTraits::cos( q(LH_HAA) );
    (*this)(0,0) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(0,1) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(1,0) = sin_q_LH_HAA;
    (*this)(1,1) = cos_q_LH_HAA;
    (*this)(2,0) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(2,1) = sin_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(3,0) = (- tz_LH_HAA * sin_q_LH_HAA)-( ty_LH_HAA * sin_ry_LH_HAA * cos_q_LH_HAA);
    (*this)(3,1) = ( ty_LH_HAA * sin_ry_LH_HAA * sin_q_LH_HAA)-( tz_LH_HAA * cos_q_LH_HAA);
    (*this)(3,3) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(3,4) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(4,0) = (( tx_LH_HAA * sin_ry_LH_HAA)+( tz_LH_HAA * cos_ry_LH_HAA)) * cos_q_LH_HAA;
    (*this)(4,1) = ((- tx_LH_HAA * sin_ry_LH_HAA)-( tz_LH_HAA * cos_ry_LH_HAA)) * sin_q_LH_HAA;
    (*this)(4,3) = sin_q_LH_HAA;
    (*this)(4,4) = cos_q_LH_HAA;
    (*this)(5,0) = ( tx_LH_HAA * sin_q_LH_HAA)-( ty_LH_HAA * cos_ry_LH_HAA * cos_q_LH_HAA);
    (*this)(5,1) = ( ty_LH_HAA * cos_ry_LH_HAA * sin_q_LH_HAA)+( tx_LH_HAA * cos_q_LH_HAA);
    (*this)(5,3) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(5,4) = sin_ry_LH_HAA * sin_q_LH_HAA;
    return *this;
}
MotionTransforms::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(2,2) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(5,5) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
}

const MotionTransforms::Type_fr_LH_THIGH_X_fr_LH_HIP& MotionTransforms::Type_fr_LH_THIGH_X_fr_LH_HIP::update(const state_t& q)
{
    Scalar sin_q_LH_HFE  = ScalarTraits::sin( q(LH_HFE) );
    Scalar cos_q_LH_HFE  = ScalarTraits::cos( q(LH_HFE) );
    (*this)(0,0) = cos_q_LH_HFE;
    (*this)(0,1) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(0,2) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(1,0) = -sin_q_LH_HFE;
    (*this)(1,1) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(1,2) = sin_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(3,3) = cos_q_LH_HFE;
    (*this)(3,4) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(3,5) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(4,3) = -sin_q_LH_HFE;
    (*this)(4,4) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(4,5) = sin_rx_LH_HFE * cos_q_LH_HFE;
    return *this;
}
MotionTransforms::Type_fr_LH_HIP_X_fr_LH_THIGH::Type_fr_LH_HIP_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
}

const MotionTransforms::Type_fr_LH_HIP_X_fr_LH_THIGH& MotionTransforms::Type_fr_LH_HIP_X_fr_LH_THIGH::update(const state_t& q)
{
    Scalar sin_q_LH_HFE  = ScalarTraits::sin( q(LH_HFE) );
    Scalar cos_q_LH_HFE  = ScalarTraits::cos( q(LH_HFE) );
    (*this)(0,0) = cos_q_LH_HFE;
    (*this)(0,1) = -sin_q_LH_HFE;
    (*this)(1,0) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(1,1) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(2,0) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(2,1) = sin_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(3,3) = cos_q_LH_HFE;
    (*this)(3,4) = -sin_q_LH_HFE;
    (*this)(4,3) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(4,4) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(5,3) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(5,4) = sin_rx_LH_HFE * cos_q_LH_HFE;
    return *this;
}
MotionTransforms::Type_fr_LH_SHANK_X_fr_LH_THIGH::Type_fr_LH_SHANK_X_fr_LH_THIGH()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(2,2) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) =  ty_LH_KFE * cos_rx_LH_KFE;    // Maxima DSL: _k__ty_LH_KFE*cos(_k__rx_LH_KFE)
    (*this)(5,1) = - tx_LH_KFE * cos_rx_LH_KFE;    // Maxima DSL: -_k__tx_LH_KFE*cos(_k__rx_LH_KFE)
    (*this)(5,2) = - tx_LH_KFE * sin_rx_LH_KFE;    // Maxima DSL: -_k__tx_LH_KFE*sin(_k__rx_LH_KFE)
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(5,5) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
}

const MotionTransforms::Type_fr_LH_SHANK_X_fr_LH_THIGH& MotionTransforms::Type_fr_LH_SHANK_X_fr_LH_THIGH::update(const state_t& q)
{
    Scalar sin_q_LH_KFE  = ScalarTraits::sin( q(LH_KFE) );
    Scalar cos_q_LH_KFE  = ScalarTraits::cos( q(LH_KFE) );
    (*this)(0,0) = cos_q_LH_KFE;
    (*this)(0,1) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(0,2) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(1,0) = -sin_q_LH_KFE;
    (*this)(1,1) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(1,2) = sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(3,0) =  ty_LH_KFE * sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(3,1) = - tx_LH_KFE * sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(3,2) = ( tx_LH_KFE * cos_rx_LH_KFE * sin_q_LH_KFE)-( ty_LH_KFE * cos_q_LH_KFE);
    (*this)(3,3) = cos_q_LH_KFE;
    (*this)(3,4) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(3,5) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(4,0) =  ty_LH_KFE * sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(4,1) = - tx_LH_KFE * sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(4,2) = ( ty_LH_KFE * sin_q_LH_KFE)+( tx_LH_KFE * cos_rx_LH_KFE * cos_q_LH_KFE);
    (*this)(4,3) = -sin_q_LH_KFE;
    (*this)(4,4) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(4,5) = sin_rx_LH_KFE * cos_q_LH_KFE;
    return *this;
}
MotionTransforms::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) =  ty_LH_KFE * cos_rx_LH_KFE;    // Maxima DSL: _k__ty_LH_KFE*cos(_k__rx_LH_KFE)
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_LH_KFE * cos_rx_LH_KFE;    // Maxima DSL: -_k__tx_LH_KFE*cos(_k__rx_LH_KFE)
    (*this)(4,5) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(5,2) = - tx_LH_KFE * sin_rx_LH_KFE;    // Maxima DSL: -_k__tx_LH_KFE*sin(_k__rx_LH_KFE)
    (*this)(5,5) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
}

const MotionTransforms::Type_fr_LH_THIGH_X_fr_LH_SHANK& MotionTransforms::Type_fr_LH_THIGH_X_fr_LH_SHANK::update(const state_t& q)
{
    Scalar sin_q_LH_KFE  = ScalarTraits::sin( q(LH_KFE) );
    Scalar cos_q_LH_KFE  = ScalarTraits::cos( q(LH_KFE) );
    (*this)(0,0) = cos_q_LH_KFE;
    (*this)(0,1) = -sin_q_LH_KFE;
    (*this)(1,0) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(1,1) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(2,0) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(2,1) = sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(3,0) =  ty_LH_KFE * sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(3,1) =  ty_LH_KFE * sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(3,3) = cos_q_LH_KFE;
    (*this)(3,4) = -sin_q_LH_KFE;
    (*this)(4,0) = - tx_LH_KFE * sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(4,1) = - tx_LH_KFE * sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(4,3) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(4,4) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(5,0) = ( tx_LH_KFE * cos_rx_LH_KFE * sin_q_LH_KFE)-( ty_LH_KFE * cos_q_LH_KFE);
    (*this)(5,1) = ( ty_LH_KFE * sin_q_LH_KFE)+( tx_LH_KFE * cos_rx_LH_KFE * cos_q_LH_KFE);
    (*this)(5,3) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(5,4) = sin_rx_LH_KFE * cos_q_LH_KFE;
    return *this;
}

ForceTransforms::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
{
    (*this)(2,0) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(2,1) = 0.0;
    (*this)(2,2) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
    (*this)(2,3) =  ty_LF_HAA * cos_ry_LF_HAA;    // Maxima DSL: _k__ty_LF_HAA*cos(_k__ry_LF_HAA)
    (*this)(2,4) = ( tz_LF_HAA * sin_ry_LF_HAA)-( tx_LF_HAA * cos_ry_LF_HAA);    // Maxima DSL: _k__tz_LF_HAA*sin(_k__ry_LF_HAA)-_k__tx_LF_HAA*cos(_k__ry_LF_HAA)
    (*this)(2,5) = - ty_LF_HAA * sin_ry_LF_HAA;    // Maxima DSL: -_k__ty_LF_HAA*sin(_k__ry_LF_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(5,4) = 0.0;
    (*this)(5,5) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
}

const ForceTransforms::Type_fr_LF_HIP_X_fr_base& ForceTransforms::Type_fr_LF_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_LF_HAA  = ScalarTraits::sin( q(LF_HAA) );
    Scalar cos_q_LF_HAA  = ScalarTraits::cos( q(LF_HAA) );
    (*this)(0,0) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(0,1) = sin_q_LF_HAA;
    (*this)(0,2) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(0,3) = (- tz_LF_HAA * sin_q_LF_HAA)-( ty_LF_HAA * sin_ry_LF_HAA * cos_q_LF_HAA);
    (*this)(0,4) = (( tx_LF_HAA * sin_ry_LF_HAA)+( tz_LF_HAA * cos_ry_LF_HAA)) * cos_q_LF_HAA;
    (*this)(0,5) = ( tx_LF_HAA * sin_q_LF_HAA)-( ty_LF_HAA * cos_ry_LF_HAA * cos_q_LF_HAA);
    (*this)(1,0) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(1,1) = cos_q_LF_HAA;
    (*this)(1,2) = sin_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(1,3) = ( ty_LF_HAA * sin_ry_LF_HAA * sin_q_LF_HAA)-( tz_LF_HAA * cos_q_LF_HAA);
    (*this)(1,4) = ((- tx_LF_HAA * sin_ry_LF_HAA)-( tz_LF_HAA * cos_ry_LF_HAA)) * sin_q_LF_HAA;
    (*this)(1,5) = ( ty_LF_HAA * cos_ry_LF_HAA * sin_q_LF_HAA)+( tx_LF_HAA * cos_q_LF_HAA);
    (*this)(3,3) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(3,4) = sin_q_LF_HAA;
    (*this)(3,5) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(4,3) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(4,4) = cos_q_LF_HAA;
    (*this)(4,5) = sin_ry_LF_HAA * sin_q_LF_HAA;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
{
    (*this)(0,2) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(0,5) =  ty_LF_HAA * cos_ry_LF_HAA;    // Maxima DSL: _k__ty_LF_HAA*cos(_k__ry_LF_HAA)
    (*this)(1,2) = 0.0;
    (*this)(1,5) = ( tz_LF_HAA * sin_ry_LF_HAA)-( tx_LF_HAA * cos_ry_LF_HAA);    // Maxima DSL: _k__tz_LF_HAA*sin(_k__ry_LF_HAA)-_k__tx_LF_HAA*cos(_k__ry_LF_HAA)
    (*this)(2,2) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
    (*this)(2,5) = - ty_LF_HAA * sin_ry_LF_HAA;    // Maxima DSL: -_k__ty_LF_HAA*sin(_k__ry_LF_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
}

const ForceTransforms::Type_fr_base_X_fr_LF_HIP& ForceTransforms::Type_fr_base_X_fr_LF_HIP::update(const state_t& q)
{
    Scalar sin_q_LF_HAA  = ScalarTraits::sin( q(LF_HAA) );
    Scalar cos_q_LF_HAA  = ScalarTraits::cos( q(LF_HAA) );
    (*this)(0,0) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(0,1) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(0,3) = (- tz_LF_HAA * sin_q_LF_HAA)-( ty_LF_HAA * sin_ry_LF_HAA * cos_q_LF_HAA);
    (*this)(0,4) = ( ty_LF_HAA * sin_ry_LF_HAA * sin_q_LF_HAA)-( tz_LF_HAA * cos_q_LF_HAA);
    (*this)(1,0) = sin_q_LF_HAA;
    (*this)(1,1) = cos_q_LF_HAA;
    (*this)(1,3) = (( tx_LF_HAA * sin_ry_LF_HAA)+( tz_LF_HAA * cos_ry_LF_HAA)) * cos_q_LF_HAA;
    (*this)(1,4) = ((- tx_LF_HAA * sin_ry_LF_HAA)-( tz_LF_HAA * cos_ry_LF_HAA)) * sin_q_LF_HAA;
    (*this)(2,0) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(2,1) = sin_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(2,3) = ( tx_LF_HAA * sin_q_LF_HAA)-( ty_LF_HAA * cos_ry_LF_HAA * cos_q_LF_HAA);
    (*this)(2,4) = ( ty_LF_HAA * cos_ry_LF_HAA * sin_q_LF_HAA)+( tx_LF_HAA * cos_q_LF_HAA);
    (*this)(3,3) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(3,4) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(4,3) = sin_q_LF_HAA;
    (*this)(4,4) = cos_q_LF_HAA;
    (*this)(5,3) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(5,4) = sin_ry_LF_HAA * sin_q_LF_HAA;
    return *this;
}
ForceTransforms::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(2,2) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(5,5) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
}

const ForceTransforms::Type_fr_LF_THIGH_X_fr_LF_HIP& ForceTransforms::Type_fr_LF_THIGH_X_fr_LF_HIP::update(const state_t& q)
{
    Scalar sin_q_LF_HFE  = ScalarTraits::sin( q(LF_HFE) );
    Scalar cos_q_LF_HFE  = ScalarTraits::cos( q(LF_HFE) );
    (*this)(0,0) = cos_q_LF_HFE;
    (*this)(0,1) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(0,2) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(1,0) = -sin_q_LF_HFE;
    (*this)(1,1) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(1,2) = sin_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(3,3) = cos_q_LF_HFE;
    (*this)(3,4) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(3,5) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(4,3) = -sin_q_LF_HFE;
    (*this)(4,4) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(4,5) = sin_rx_LF_HFE * cos_q_LF_HFE;
    return *this;
}
ForceTransforms::Type_fr_LF_HIP_X_fr_LF_THIGH::Type_fr_LF_HIP_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
}

const ForceTransforms::Type_fr_LF_HIP_X_fr_LF_THIGH& ForceTransforms::Type_fr_LF_HIP_X_fr_LF_THIGH::update(const state_t& q)
{
    Scalar sin_q_LF_HFE  = ScalarTraits::sin( q(LF_HFE) );
    Scalar cos_q_LF_HFE  = ScalarTraits::cos( q(LF_HFE) );
    (*this)(0,0) = cos_q_LF_HFE;
    (*this)(0,1) = -sin_q_LF_HFE;
    (*this)(1,0) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(1,1) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(2,0) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(2,1) = sin_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(3,3) = cos_q_LF_HFE;
    (*this)(3,4) = -sin_q_LF_HFE;
    (*this)(4,3) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(4,4) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(5,3) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(5,4) = sin_rx_LF_HFE * cos_q_LF_HFE;
    return *this;
}
ForceTransforms::Type_fr_LF_SHANK_X_fr_LF_THIGH::Type_fr_LF_SHANK_X_fr_LF_THIGH()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(2,2) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
    (*this)(2,3) =  ty_LF_KFE * cos_rx_LF_KFE;    // Maxima DSL: _k__ty_LF_KFE*cos(_k__rx_LF_KFE)
    (*this)(2,4) = - tx_LF_KFE * cos_rx_LF_KFE;    // Maxima DSL: -_k__tx_LF_KFE*cos(_k__rx_LF_KFE)
    (*this)(2,5) = - tx_LF_KFE * sin_rx_LF_KFE;    // Maxima DSL: -_k__tx_LF_KFE*sin(_k__rx_LF_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(5,5) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
}

const ForceTransforms::Type_fr_LF_SHANK_X_fr_LF_THIGH& ForceTransforms::Type_fr_LF_SHANK_X_fr_LF_THIGH::update(const state_t& q)
{
    Scalar sin_q_LF_KFE  = ScalarTraits::sin( q(LF_KFE) );
    Scalar cos_q_LF_KFE  = ScalarTraits::cos( q(LF_KFE) );
    (*this)(0,0) = cos_q_LF_KFE;
    (*this)(0,1) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(0,2) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(0,3) =  ty_LF_KFE * sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(0,4) = - tx_LF_KFE * sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(0,5) = ( tx_LF_KFE * cos_rx_LF_KFE * sin_q_LF_KFE)-( ty_LF_KFE * cos_q_LF_KFE);
    (*this)(1,0) = -sin_q_LF_KFE;
    (*this)(1,1) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(1,2) = sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(1,3) =  ty_LF_KFE * sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(1,4) = - tx_LF_KFE * sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(1,5) = ( ty_LF_KFE * sin_q_LF_KFE)+( tx_LF_KFE * cos_rx_LF_KFE * cos_q_LF_KFE);
    (*this)(3,3) = cos_q_LF_KFE;
    (*this)(3,4) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(3,5) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(4,3) = -sin_q_LF_KFE;
    (*this)(4,4) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(4,5) = sin_rx_LF_KFE * cos_q_LF_KFE;
    return *this;
}
ForceTransforms::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) =  ty_LF_KFE * cos_rx_LF_KFE;    // Maxima DSL: _k__ty_LF_KFE*cos(_k__rx_LF_KFE)
    (*this)(1,2) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(1,5) = - tx_LF_KFE * cos_rx_LF_KFE;    // Maxima DSL: -_k__tx_LF_KFE*cos(_k__rx_LF_KFE)
    (*this)(2,2) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
    (*this)(2,5) = - tx_LF_KFE * sin_rx_LF_KFE;    // Maxima DSL: -_k__tx_LF_KFE*sin(_k__rx_LF_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
}

const ForceTransforms::Type_fr_LF_THIGH_X_fr_LF_SHANK& ForceTransforms::Type_fr_LF_THIGH_X_fr_LF_SHANK::update(const state_t& q)
{
    Scalar sin_q_LF_KFE  = ScalarTraits::sin( q(LF_KFE) );
    Scalar cos_q_LF_KFE  = ScalarTraits::cos( q(LF_KFE) );
    (*this)(0,0) = cos_q_LF_KFE;
    (*this)(0,1) = -sin_q_LF_KFE;
    (*this)(0,3) =  ty_LF_KFE * sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(0,4) =  ty_LF_KFE * sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(1,0) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(1,1) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(1,3) = - tx_LF_KFE * sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(1,4) = - tx_LF_KFE * sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(2,0) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(2,1) = sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(2,3) = ( tx_LF_KFE * cos_rx_LF_KFE * sin_q_LF_KFE)-( ty_LF_KFE * cos_q_LF_KFE);
    (*this)(2,4) = ( ty_LF_KFE * sin_q_LF_KFE)+( tx_LF_KFE * cos_rx_LF_KFE * cos_q_LF_KFE);
    (*this)(3,3) = cos_q_LF_KFE;
    (*this)(3,4) = -sin_q_LF_KFE;
    (*this)(4,3) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(4,4) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(5,3) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(5,4) = sin_rx_LF_KFE * cos_q_LF_KFE;
    return *this;
}
ForceTransforms::Type_fr_RF_HIP_X_fr_base::Type_fr_RF_HIP_X_fr_base()
{
    (*this)(2,0) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(2,1) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,2) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,3) = (( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * cos_ry_RF_HAA;    // Maxima DSL: (_k__tz_RF_HAA*sin(_k__rx_RF_HAA)+_k__ty_RF_HAA*cos(_k__rx_RF_HAA))*cos(_k__ry_RF_HAA)
    (*this)(2,4) = ( tz_RF_HAA * sin_ry_RF_HAA)-( tx_RF_HAA * cos_rx_RF_HAA * cos_ry_RF_HAA);    // Maxima DSL: _k__tz_RF_HAA*sin(_k__ry_RF_HAA)-_k__tx_RF_HAA*cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,5) = (- ty_RF_HAA * sin_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_ry_RF_HAA);    // Maxima DSL: (-_k__ty_RF_HAA*sin(_k__ry_RF_HAA))-_k__tx_RF_HAA*sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(5,4) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(5,5) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
}

const ForceTransforms::Type_fr_RF_HIP_X_fr_base& ForceTransforms::Type_fr_RF_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RF_HAA  = ScalarTraits::sin( q(RF_HAA) );
    Scalar cos_q_RF_HAA  = ScalarTraits::cos( q(RF_HAA) );
    (*this)(0,0) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(0,1) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(0,2) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(0,3) = ((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((((- tz_RF_HAA * sin_rx_RF_HAA)-( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(0,4) = (((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_rz_RF_HAA)) * sin_q_RF_HAA)+((((( tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)+( tz_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(0,5) = ((( tx_RF_HAA * cos_rx_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((((( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)-( ty_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+( tx_RF_HAA * cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,0) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(1,1) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,2) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,3) = ((((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * cos_rz_RF_HAA)+((( tz_RF_HAA * cos_rx_RF_HAA)-( ty_RF_HAA * sin_rx_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,4) = (((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+( tx_RF_HAA * sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,5) = ((((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * cos_rz_RF_HAA)-( tx_RF_HAA * cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((( tx_RF_HAA * cos_rx_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,3) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(3,4) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,5) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,3) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(4,4) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,5) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
{
    (*this)(0,2) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(0,5) = (( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * cos_ry_RF_HAA;    // Maxima DSL: (_k__tz_RF_HAA*sin(_k__rx_RF_HAA)+_k__ty_RF_HAA*cos(_k__rx_RF_HAA))*cos(_k__ry_RF_HAA)
    (*this)(1,2) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(1,5) = ( tz_RF_HAA * sin_ry_RF_HAA)-( tx_RF_HAA * cos_rx_RF_HAA * cos_ry_RF_HAA);    // Maxima DSL: _k__tz_RF_HAA*sin(_k__ry_RF_HAA)-_k__tx_RF_HAA*cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,2) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,5) = (- ty_RF_HAA * sin_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_ry_RF_HAA);    // Maxima DSL: (-_k__ty_RF_HAA*sin(_k__ry_RF_HAA))-_k__tx_RF_HAA*sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
}

const ForceTransforms::Type_fr_base_X_fr_RF_HIP& ForceTransforms::Type_fr_base_X_fr_RF_HIP::update(const state_t& q)
{
    Scalar sin_q_RF_HAA  = ScalarTraits::sin( q(RF_HAA) );
    Scalar cos_q_RF_HAA  = ScalarTraits::cos( q(RF_HAA) );
    (*this)(0,0) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(0,1) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(0,3) = ((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((((- tz_RF_HAA * sin_rx_RF_HAA)-( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(0,4) = ((((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * cos_rz_RF_HAA)+((( tz_RF_HAA * cos_rx_RF_HAA)-( ty_RF_HAA * sin_rx_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,0) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,1) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,3) = (((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_rz_RF_HAA)) * sin_q_RF_HAA)+((((( tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)+( tz_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,4) = (((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+( tx_RF_HAA * sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((((- tx_RF_HAA * cos_rx_RF_HAA * sin_ry_RF_HAA)-( tz_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(2,0) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(2,1) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(2,3) = ((( tx_RF_HAA * cos_rx_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((((( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)-( ty_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+( tx_RF_HAA * cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(2,4) = ((((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * cos_rz_RF_HAA)-( tx_RF_HAA * cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((( tx_RF_HAA * cos_rx_RF_HAA * cos_rz_RF_HAA)+((( ty_RF_HAA * cos_ry_RF_HAA)-( tx_RF_HAA * sin_rx_RF_HAA * sin_ry_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(3,3) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(3,4) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(4,3) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(4,4) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(5,3) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(5,4) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    return *this;
}
ForceTransforms::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(2,2) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(5,5) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
}

const ForceTransforms::Type_fr_RF_THIGH_X_fr_RF_HIP& ForceTransforms::Type_fr_RF_THIGH_X_fr_RF_HIP::update(const state_t& q)
{
    Scalar sin_q_RF_HFE  = ScalarTraits::sin( q(RF_HFE) );
    Scalar cos_q_RF_HFE  = ScalarTraits::cos( q(RF_HFE) );
    (*this)(0,0) = cos_q_RF_HFE;
    (*this)(0,1) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(0,2) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(1,0) = -sin_q_RF_HFE;
    (*this)(1,1) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(1,2) = sin_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(3,3) = cos_q_RF_HFE;
    (*this)(3,4) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(3,5) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(4,3) = -sin_q_RF_HFE;
    (*this)(4,4) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(4,5) = sin_rx_RF_HFE * cos_q_RF_HFE;
    return *this;
}
ForceTransforms::Type_fr_RF_HIP_X_fr_RF_THIGH::Type_fr_RF_HIP_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
}

const ForceTransforms::Type_fr_RF_HIP_X_fr_RF_THIGH& ForceTransforms::Type_fr_RF_HIP_X_fr_RF_THIGH::update(const state_t& q)
{
    Scalar sin_q_RF_HFE  = ScalarTraits::sin( q(RF_HFE) );
    Scalar cos_q_RF_HFE  = ScalarTraits::cos( q(RF_HFE) );
    (*this)(0,0) = cos_q_RF_HFE;
    (*this)(0,1) = -sin_q_RF_HFE;
    (*this)(1,0) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(1,1) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(2,0) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(2,1) = sin_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(3,3) = cos_q_RF_HFE;
    (*this)(3,4) = -sin_q_RF_HFE;
    (*this)(4,3) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(4,4) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(5,3) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(5,4) = sin_rx_RF_HFE * cos_q_RF_HFE;
    return *this;
}
ForceTransforms::Type_fr_RF_SHANK_X_fr_RF_THIGH::Type_fr_RF_SHANK_X_fr_RF_THIGH()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(2,2) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_RF_KFE * cos_rx_RF_KFE;    // Maxima DSL: -_k__tx_RF_KFE*cos(_k__rx_RF_KFE)
    (*this)(2,5) = - tx_RF_KFE * sin_rx_RF_KFE;    // Maxima DSL: -_k__tx_RF_KFE*sin(_k__rx_RF_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(5,5) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
}

const ForceTransforms::Type_fr_RF_SHANK_X_fr_RF_THIGH& ForceTransforms::Type_fr_RF_SHANK_X_fr_RF_THIGH::update(const state_t& q)
{
    Scalar sin_q_RF_KFE  = ScalarTraits::sin( q(RF_KFE) );
    Scalar cos_q_RF_KFE  = ScalarTraits::cos( q(RF_KFE) );
    (*this)(0,0) = cos_q_RF_KFE;
    (*this)(0,1) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(0,2) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(0,4) = - tx_RF_KFE * sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(0,5) =  tx_RF_KFE * cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(1,0) = -sin_q_RF_KFE;
    (*this)(1,1) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(1,2) = sin_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(1,4) = - tx_RF_KFE * sin_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(1,5) =  tx_RF_KFE * cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(3,3) = cos_q_RF_KFE;
    (*this)(3,4) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(3,5) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(4,3) = -sin_q_RF_KFE;
    (*this)(4,4) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(4,5) = sin_rx_RF_KFE * cos_q_RF_KFE;
    return *this;
}
ForceTransforms::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(1,5) = - tx_RF_KFE * cos_rx_RF_KFE;    // Maxima DSL: -_k__tx_RF_KFE*cos(_k__rx_RF_KFE)
    (*this)(2,2) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
    (*this)(2,5) = - tx_RF_KFE * sin_rx_RF_KFE;    // Maxima DSL: -_k__tx_RF_KFE*sin(_k__rx_RF_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
}

const ForceTransforms::Type_fr_RF_THIGH_X_fr_RF_SHANK& ForceTransforms::Type_fr_RF_THIGH_X_fr_RF_SHANK::update(const state_t& q)
{
    Scalar sin_q_RF_KFE  = ScalarTraits::sin( q(RF_KFE) );
    Scalar cos_q_RF_KFE  = ScalarTraits::cos( q(RF_KFE) );
    (*this)(0,0) = cos_q_RF_KFE;
    (*this)(0,1) = -sin_q_RF_KFE;
    (*this)(1,0) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(1,1) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(1,3) = - tx_RF_KFE * sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(1,4) = - tx_RF_KFE * sin_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(2,0) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(2,1) = sin_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(2,3) =  tx_RF_KFE * cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(2,4) =  tx_RF_KFE * cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(3,3) = cos_q_RF_KFE;
    (*this)(3,4) = -sin_q_RF_KFE;
    (*this)(4,3) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(4,4) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(5,3) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(5,4) = sin_rx_RF_KFE * cos_q_RF_KFE;
    return *this;
}
ForceTransforms::Type_fr_RH_HIP_X_fr_base::Type_fr_RH_HIP_X_fr_base()
{
    (*this)(2,0) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(2,1) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,2) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,3) = (( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * cos_ry_RH_HAA;    // Maxima DSL: (_k__tz_RH_HAA*sin(_k__rx_RH_HAA)+_k__ty_RH_HAA*cos(_k__rx_RH_HAA))*cos(_k__ry_RH_HAA)
    (*this)(2,4) = ( tz_RH_HAA * sin_ry_RH_HAA)-( tx_RH_HAA * cos_rx_RH_HAA * cos_ry_RH_HAA);    // Maxima DSL: _k__tz_RH_HAA*sin(_k__ry_RH_HAA)-_k__tx_RH_HAA*cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,5) = (- ty_RH_HAA * sin_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_ry_RH_HAA);    // Maxima DSL: (-_k__ty_RH_HAA*sin(_k__ry_RH_HAA))-_k__tx_RH_HAA*sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(5,4) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(5,5) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
}

const ForceTransforms::Type_fr_RH_HIP_X_fr_base& ForceTransforms::Type_fr_RH_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RH_HAA  = ScalarTraits::sin( q(RH_HAA) );
    Scalar cos_q_RH_HAA  = ScalarTraits::cos( q(RH_HAA) );
    (*this)(0,0) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(0,1) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(0,2) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(0,3) = ((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((((- tz_RH_HAA * sin_rx_RH_HAA)-( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(0,4) = (((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_rz_RH_HAA)) * sin_q_RH_HAA)+((((( tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)+( tz_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(0,5) = ((( tx_RH_HAA * cos_rx_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((((( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)-( ty_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+( tx_RH_HAA * cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,0) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(1,1) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,2) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,3) = ((((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * cos_rz_RH_HAA)+((( tz_RH_HAA * cos_rx_RH_HAA)-( ty_RH_HAA * sin_rx_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,4) = (((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+( tx_RH_HAA * sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,5) = ((((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * cos_rz_RH_HAA)-( tx_RH_HAA * cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((( tx_RH_HAA * cos_rx_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,3) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(3,4) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,5) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,3) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(4,4) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,5) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
{
    (*this)(0,2) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(0,5) = (( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * cos_ry_RH_HAA;    // Maxima DSL: (_k__tz_RH_HAA*sin(_k__rx_RH_HAA)+_k__ty_RH_HAA*cos(_k__rx_RH_HAA))*cos(_k__ry_RH_HAA)
    (*this)(1,2) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(1,5) = ( tz_RH_HAA * sin_ry_RH_HAA)-( tx_RH_HAA * cos_rx_RH_HAA * cos_ry_RH_HAA);    // Maxima DSL: _k__tz_RH_HAA*sin(_k__ry_RH_HAA)-_k__tx_RH_HAA*cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,2) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,5) = (- ty_RH_HAA * sin_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_ry_RH_HAA);    // Maxima DSL: (-_k__ty_RH_HAA*sin(_k__ry_RH_HAA))-_k__tx_RH_HAA*sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
}

const ForceTransforms::Type_fr_base_X_fr_RH_HIP& ForceTransforms::Type_fr_base_X_fr_RH_HIP::update(const state_t& q)
{
    Scalar sin_q_RH_HAA  = ScalarTraits::sin( q(RH_HAA) );
    Scalar cos_q_RH_HAA  = ScalarTraits::cos( q(RH_HAA) );
    (*this)(0,0) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(0,1) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(0,3) = ((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((((- tz_RH_HAA * sin_rx_RH_HAA)-( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(0,4) = ((((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * cos_rz_RH_HAA)+((( tz_RH_HAA * cos_rx_RH_HAA)-( ty_RH_HAA * sin_rx_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,0) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,1) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,3) = (((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_rz_RH_HAA)) * sin_q_RH_HAA)+((((( tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)+( tz_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,4) = (((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+( tx_RH_HAA * sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((((- tx_RH_HAA * cos_rx_RH_HAA * sin_ry_RH_HAA)-( tz_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(2,0) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(2,1) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(2,3) = ((( tx_RH_HAA * cos_rx_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((((( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)-( ty_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+( tx_RH_HAA * cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(2,4) = ((((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * cos_rz_RH_HAA)-( tx_RH_HAA * cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((( tx_RH_HAA * cos_rx_RH_HAA * cos_rz_RH_HAA)+((( ty_RH_HAA * cos_ry_RH_HAA)-( tx_RH_HAA * sin_rx_RH_HAA * sin_ry_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(3,3) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(3,4) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(4,3) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(4,4) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(5,3) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(5,4) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    return *this;
}
ForceTransforms::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(2,2) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(5,5) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
}

const ForceTransforms::Type_fr_RH_THIGH_X_fr_RH_HIP& ForceTransforms::Type_fr_RH_THIGH_X_fr_RH_HIP::update(const state_t& q)
{
    Scalar sin_q_RH_HFE  = ScalarTraits::sin( q(RH_HFE) );
    Scalar cos_q_RH_HFE  = ScalarTraits::cos( q(RH_HFE) );
    (*this)(0,0) = cos_q_RH_HFE;
    (*this)(0,1) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(0,2) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(1,0) = -sin_q_RH_HFE;
    (*this)(1,1) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(1,2) = sin_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(3,3) = cos_q_RH_HFE;
    (*this)(3,4) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(3,5) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(4,3) = -sin_q_RH_HFE;
    (*this)(4,4) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(4,5) = sin_rx_RH_HFE * cos_q_RH_HFE;
    return *this;
}
ForceTransforms::Type_fr_RH_HIP_X_fr_RH_THIGH::Type_fr_RH_HIP_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
}

const ForceTransforms::Type_fr_RH_HIP_X_fr_RH_THIGH& ForceTransforms::Type_fr_RH_HIP_X_fr_RH_THIGH::update(const state_t& q)
{
    Scalar sin_q_RH_HFE  = ScalarTraits::sin( q(RH_HFE) );
    Scalar cos_q_RH_HFE  = ScalarTraits::cos( q(RH_HFE) );
    (*this)(0,0) = cos_q_RH_HFE;
    (*this)(0,1) = -sin_q_RH_HFE;
    (*this)(1,0) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(1,1) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(2,0) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(2,1) = sin_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(3,3) = cos_q_RH_HFE;
    (*this)(3,4) = -sin_q_RH_HFE;
    (*this)(4,3) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(4,4) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(5,3) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(5,4) = sin_rx_RH_HFE * cos_q_RH_HFE;
    return *this;
}
ForceTransforms::Type_fr_RH_SHANK_X_fr_RH_THIGH::Type_fr_RH_SHANK_X_fr_RH_THIGH()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(2,2) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_RH_KFE * cos_rx_RH_KFE;    // Maxima DSL: -_k__tx_RH_KFE*cos(_k__rx_RH_KFE)
    (*this)(2,5) = - tx_RH_KFE * sin_rx_RH_KFE;    // Maxima DSL: -_k__tx_RH_KFE*sin(_k__rx_RH_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(5,5) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
}

const ForceTransforms::Type_fr_RH_SHANK_X_fr_RH_THIGH& ForceTransforms::Type_fr_RH_SHANK_X_fr_RH_THIGH::update(const state_t& q)
{
    Scalar sin_q_RH_KFE  = ScalarTraits::sin( q(RH_KFE) );
    Scalar cos_q_RH_KFE  = ScalarTraits::cos( q(RH_KFE) );
    (*this)(0,0) = cos_q_RH_KFE;
    (*this)(0,1) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(0,2) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(0,4) = - tx_RH_KFE * sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(0,5) =  tx_RH_KFE * cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(1,0) = -sin_q_RH_KFE;
    (*this)(1,1) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(1,2) = sin_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(1,4) = - tx_RH_KFE * sin_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(1,5) =  tx_RH_KFE * cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(3,3) = cos_q_RH_KFE;
    (*this)(3,4) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(3,5) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(4,3) = -sin_q_RH_KFE;
    (*this)(4,4) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(4,5) = sin_rx_RH_KFE * cos_q_RH_KFE;
    return *this;
}
ForceTransforms::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(1,5) = - tx_RH_KFE * cos_rx_RH_KFE;    // Maxima DSL: -_k__tx_RH_KFE*cos(_k__rx_RH_KFE)
    (*this)(2,2) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
    (*this)(2,5) = - tx_RH_KFE * sin_rx_RH_KFE;    // Maxima DSL: -_k__tx_RH_KFE*sin(_k__rx_RH_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
}

const ForceTransforms::Type_fr_RH_THIGH_X_fr_RH_SHANK& ForceTransforms::Type_fr_RH_THIGH_X_fr_RH_SHANK::update(const state_t& q)
{
    Scalar sin_q_RH_KFE  = ScalarTraits::sin( q(RH_KFE) );
    Scalar cos_q_RH_KFE  = ScalarTraits::cos( q(RH_KFE) );
    (*this)(0,0) = cos_q_RH_KFE;
    (*this)(0,1) = -sin_q_RH_KFE;
    (*this)(1,0) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(1,1) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(1,3) = - tx_RH_KFE * sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(1,4) = - tx_RH_KFE * sin_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(2,0) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(2,1) = sin_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(2,3) =  tx_RH_KFE * cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(2,4) =  tx_RH_KFE * cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(3,3) = cos_q_RH_KFE;
    (*this)(3,4) = -sin_q_RH_KFE;
    (*this)(4,3) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(4,4) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(5,3) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(5,4) = sin_rx_RH_KFE * cos_q_RH_KFE;
    return *this;
}
ForceTransforms::Type_fr_LH_HIP_X_fr_base::Type_fr_LH_HIP_X_fr_base()
{
    (*this)(2,0) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(2,1) = 0.0;
    (*this)(2,2) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
    (*this)(2,3) =  ty_LH_HAA * cos_ry_LH_HAA;    // Maxima DSL: _k__ty_LH_HAA*cos(_k__ry_LH_HAA)
    (*this)(2,4) = ( tz_LH_HAA * sin_ry_LH_HAA)-( tx_LH_HAA * cos_ry_LH_HAA);    // Maxima DSL: _k__tz_LH_HAA*sin(_k__ry_LH_HAA)-_k__tx_LH_HAA*cos(_k__ry_LH_HAA)
    (*this)(2,5) = - ty_LH_HAA * sin_ry_LH_HAA;    // Maxima DSL: -_k__ty_LH_HAA*sin(_k__ry_LH_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(5,4) = 0.0;
    (*this)(5,5) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
}

const ForceTransforms::Type_fr_LH_HIP_X_fr_base& ForceTransforms::Type_fr_LH_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_LH_HAA  = ScalarTraits::sin( q(LH_HAA) );
    Scalar cos_q_LH_HAA  = ScalarTraits::cos( q(LH_HAA) );
    (*this)(0,0) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(0,1) = sin_q_LH_HAA;
    (*this)(0,2) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(0,3) = (- tz_LH_HAA * sin_q_LH_HAA)-( ty_LH_HAA * sin_ry_LH_HAA * cos_q_LH_HAA);
    (*this)(0,4) = (( tx_LH_HAA * sin_ry_LH_HAA)+( tz_LH_HAA * cos_ry_LH_HAA)) * cos_q_LH_HAA;
    (*this)(0,5) = ( tx_LH_HAA * sin_q_LH_HAA)-( ty_LH_HAA * cos_ry_LH_HAA * cos_q_LH_HAA);
    (*this)(1,0) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(1,1) = cos_q_LH_HAA;
    (*this)(1,2) = sin_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(1,3) = ( ty_LH_HAA * sin_ry_LH_HAA * sin_q_LH_HAA)-( tz_LH_HAA * cos_q_LH_HAA);
    (*this)(1,4) = ((- tx_LH_HAA * sin_ry_LH_HAA)-( tz_LH_HAA * cos_ry_LH_HAA)) * sin_q_LH_HAA;
    (*this)(1,5) = ( ty_LH_HAA * cos_ry_LH_HAA * sin_q_LH_HAA)+( tx_LH_HAA * cos_q_LH_HAA);
    (*this)(3,3) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(3,4) = sin_q_LH_HAA;
    (*this)(3,5) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(4,3) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(4,4) = cos_q_LH_HAA;
    (*this)(4,5) = sin_ry_LH_HAA * sin_q_LH_HAA;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
{
    (*this)(0,2) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(0,5) =  ty_LH_HAA * cos_ry_LH_HAA;    // Maxima DSL: _k__ty_LH_HAA*cos(_k__ry_LH_HAA)
    (*this)(1,2) = 0.0;
    (*this)(1,5) = ( tz_LH_HAA * sin_ry_LH_HAA)-( tx_LH_HAA * cos_ry_LH_HAA);    // Maxima DSL: _k__tz_LH_HAA*sin(_k__ry_LH_HAA)-_k__tx_LH_HAA*cos(_k__ry_LH_HAA)
    (*this)(2,2) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
    (*this)(2,5) = - ty_LH_HAA * sin_ry_LH_HAA;    // Maxima DSL: -_k__ty_LH_HAA*sin(_k__ry_LH_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
}

const ForceTransforms::Type_fr_base_X_fr_LH_HIP& ForceTransforms::Type_fr_base_X_fr_LH_HIP::update(const state_t& q)
{
    Scalar sin_q_LH_HAA  = ScalarTraits::sin( q(LH_HAA) );
    Scalar cos_q_LH_HAA  = ScalarTraits::cos( q(LH_HAA) );
    (*this)(0,0) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(0,1) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(0,3) = (- tz_LH_HAA * sin_q_LH_HAA)-( ty_LH_HAA * sin_ry_LH_HAA * cos_q_LH_HAA);
    (*this)(0,4) = ( ty_LH_HAA * sin_ry_LH_HAA * sin_q_LH_HAA)-( tz_LH_HAA * cos_q_LH_HAA);
    (*this)(1,0) = sin_q_LH_HAA;
    (*this)(1,1) = cos_q_LH_HAA;
    (*this)(1,3) = (( tx_LH_HAA * sin_ry_LH_HAA)+( tz_LH_HAA * cos_ry_LH_HAA)) * cos_q_LH_HAA;
    (*this)(1,4) = ((- tx_LH_HAA * sin_ry_LH_HAA)-( tz_LH_HAA * cos_ry_LH_HAA)) * sin_q_LH_HAA;
    (*this)(2,0) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(2,1) = sin_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(2,3) = ( tx_LH_HAA * sin_q_LH_HAA)-( ty_LH_HAA * cos_ry_LH_HAA * cos_q_LH_HAA);
    (*this)(2,4) = ( ty_LH_HAA * cos_ry_LH_HAA * sin_q_LH_HAA)+( tx_LH_HAA * cos_q_LH_HAA);
    (*this)(3,3) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(3,4) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(4,3) = sin_q_LH_HAA;
    (*this)(4,4) = cos_q_LH_HAA;
    (*this)(5,3) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(5,4) = sin_ry_LH_HAA * sin_q_LH_HAA;
    return *this;
}
ForceTransforms::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(2,2) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(5,5) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
}

const ForceTransforms::Type_fr_LH_THIGH_X_fr_LH_HIP& ForceTransforms::Type_fr_LH_THIGH_X_fr_LH_HIP::update(const state_t& q)
{
    Scalar sin_q_LH_HFE  = ScalarTraits::sin( q(LH_HFE) );
    Scalar cos_q_LH_HFE  = ScalarTraits::cos( q(LH_HFE) );
    (*this)(0,0) = cos_q_LH_HFE;
    (*this)(0,1) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(0,2) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(1,0) = -sin_q_LH_HFE;
    (*this)(1,1) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(1,2) = sin_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(3,3) = cos_q_LH_HFE;
    (*this)(3,4) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(3,5) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(4,3) = -sin_q_LH_HFE;
    (*this)(4,4) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(4,5) = sin_rx_LH_HFE * cos_q_LH_HFE;
    return *this;
}
ForceTransforms::Type_fr_LH_HIP_X_fr_LH_THIGH::Type_fr_LH_HIP_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
}

const ForceTransforms::Type_fr_LH_HIP_X_fr_LH_THIGH& ForceTransforms::Type_fr_LH_HIP_X_fr_LH_THIGH::update(const state_t& q)
{
    Scalar sin_q_LH_HFE  = ScalarTraits::sin( q(LH_HFE) );
    Scalar cos_q_LH_HFE  = ScalarTraits::cos( q(LH_HFE) );
    (*this)(0,0) = cos_q_LH_HFE;
    (*this)(0,1) = -sin_q_LH_HFE;
    (*this)(1,0) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(1,1) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(2,0) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(2,1) = sin_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(3,3) = cos_q_LH_HFE;
    (*this)(3,4) = -sin_q_LH_HFE;
    (*this)(4,3) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(4,4) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(5,3) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(5,4) = sin_rx_LH_HFE * cos_q_LH_HFE;
    return *this;
}
ForceTransforms::Type_fr_LH_SHANK_X_fr_LH_THIGH::Type_fr_LH_SHANK_X_fr_LH_THIGH()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(2,2) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
    (*this)(2,3) =  ty_LH_KFE * cos_rx_LH_KFE;    // Maxima DSL: _k__ty_LH_KFE*cos(_k__rx_LH_KFE)
    (*this)(2,4) = - tx_LH_KFE * cos_rx_LH_KFE;    // Maxima DSL: -_k__tx_LH_KFE*cos(_k__rx_LH_KFE)
    (*this)(2,5) = - tx_LH_KFE * sin_rx_LH_KFE;    // Maxima DSL: -_k__tx_LH_KFE*sin(_k__rx_LH_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(5,5) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
}

const ForceTransforms::Type_fr_LH_SHANK_X_fr_LH_THIGH& ForceTransforms::Type_fr_LH_SHANK_X_fr_LH_THIGH::update(const state_t& q)
{
    Scalar sin_q_LH_KFE  = ScalarTraits::sin( q(LH_KFE) );
    Scalar cos_q_LH_KFE  = ScalarTraits::cos( q(LH_KFE) );
    (*this)(0,0) = cos_q_LH_KFE;
    (*this)(0,1) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(0,2) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(0,3) =  ty_LH_KFE * sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(0,4) = - tx_LH_KFE * sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(0,5) = ( tx_LH_KFE * cos_rx_LH_KFE * sin_q_LH_KFE)-( ty_LH_KFE * cos_q_LH_KFE);
    (*this)(1,0) = -sin_q_LH_KFE;
    (*this)(1,1) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(1,2) = sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(1,3) =  ty_LH_KFE * sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(1,4) = - tx_LH_KFE * sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(1,5) = ( ty_LH_KFE * sin_q_LH_KFE)+( tx_LH_KFE * cos_rx_LH_KFE * cos_q_LH_KFE);
    (*this)(3,3) = cos_q_LH_KFE;
    (*this)(3,4) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(3,5) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(4,3) = -sin_q_LH_KFE;
    (*this)(4,4) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(4,5) = sin_rx_LH_KFE * cos_q_LH_KFE;
    return *this;
}
ForceTransforms::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) =  ty_LH_KFE * cos_rx_LH_KFE;    // Maxima DSL: _k__ty_LH_KFE*cos(_k__rx_LH_KFE)
    (*this)(1,2) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(1,5) = - tx_LH_KFE * cos_rx_LH_KFE;    // Maxima DSL: -_k__tx_LH_KFE*cos(_k__rx_LH_KFE)
    (*this)(2,2) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
    (*this)(2,5) = - tx_LH_KFE * sin_rx_LH_KFE;    // Maxima DSL: -_k__tx_LH_KFE*sin(_k__rx_LH_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
}

const ForceTransforms::Type_fr_LH_THIGH_X_fr_LH_SHANK& ForceTransforms::Type_fr_LH_THIGH_X_fr_LH_SHANK::update(const state_t& q)
{
    Scalar sin_q_LH_KFE  = ScalarTraits::sin( q(LH_KFE) );
    Scalar cos_q_LH_KFE  = ScalarTraits::cos( q(LH_KFE) );
    (*this)(0,0) = cos_q_LH_KFE;
    (*this)(0,1) = -sin_q_LH_KFE;
    (*this)(0,3) =  ty_LH_KFE * sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(0,4) =  ty_LH_KFE * sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(1,0) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(1,1) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(1,3) = - tx_LH_KFE * sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(1,4) = - tx_LH_KFE * sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(2,0) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(2,1) = sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(2,3) = ( tx_LH_KFE * cos_rx_LH_KFE * sin_q_LH_KFE)-( ty_LH_KFE * cos_q_LH_KFE);
    (*this)(2,4) = ( ty_LH_KFE * sin_q_LH_KFE)+( tx_LH_KFE * cos_rx_LH_KFE * cos_q_LH_KFE);
    (*this)(3,3) = cos_q_LH_KFE;
    (*this)(3,4) = -sin_q_LH_KFE;
    (*this)(4,3) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(4,4) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(5,3) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(5,4) = sin_rx_LH_KFE * cos_q_LH_KFE;
    return *this;
}

HomogeneousTransforms::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
{
    (*this)(2,0) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(2,1) = 0.0;
    (*this)(2,2) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
    (*this)(2,3) = (- tx_LF_HAA * sin_ry_LF_HAA)-( tz_LF_HAA * cos_ry_LF_HAA);    // Maxima DSL: (-_k__tx_LF_HAA*sin(_k__ry_LF_HAA))-_k__tz_LF_HAA*cos(_k__ry_LF_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LF_HIP_X_fr_base& HomogeneousTransforms::Type_fr_LF_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_LF_HAA  = ScalarTraits::sin( q(LF_HAA) );
    Scalar cos_q_LF_HAA  = ScalarTraits::cos( q(LF_HAA) );
    (*this)(0,0) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(0,1) = sin_q_LF_HAA;
    (*this)(0,2) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(0,3) = ((( tz_LF_HAA * sin_ry_LF_HAA)-( tx_LF_HAA * cos_ry_LF_HAA)) * cos_q_LF_HAA)-( ty_LF_HAA * sin_q_LF_HAA);
    (*this)(1,0) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(1,1) = cos_q_LF_HAA;
    (*this)(1,2) = sin_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(1,3) = ((( tx_LF_HAA * cos_ry_LF_HAA)-( tz_LF_HAA * sin_ry_LF_HAA)) * sin_q_LF_HAA)-( ty_LF_HAA * cos_q_LF_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
{
    (*this)(0,2) = sin_ry_LF_HAA;    // Maxima DSL: sin(_k__ry_LF_HAA)
    (*this)(0,3) =  tx_LF_HAA;    // Maxima DSL: _k__tx_LF_HAA
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_LF_HAA;    // Maxima DSL: _k__ty_LF_HAA
    (*this)(2,2) = cos_ry_LF_HAA;    // Maxima DSL: cos(_k__ry_LF_HAA)
    (*this)(2,3) =  tz_LF_HAA;    // Maxima DSL: _k__tz_LF_HAA
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_LF_HIP& HomogeneousTransforms::Type_fr_base_X_fr_LF_HIP::update(const state_t& q)
{
    Scalar sin_q_LF_HAA  = ScalarTraits::sin( q(LF_HAA) );
    Scalar cos_q_LF_HAA  = ScalarTraits::cos( q(LF_HAA) );
    (*this)(0,0) = cos_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(0,1) = -cos_ry_LF_HAA * sin_q_LF_HAA;
    (*this)(1,0) = sin_q_LF_HAA;
    (*this)(1,1) = cos_q_LF_HAA;
    (*this)(2,0) = -sin_ry_LF_HAA * cos_q_LF_HAA;
    (*this)(2,1) = sin_ry_LF_HAA * sin_q_LF_HAA;
    return *this;
}
HomogeneousTransforms::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(2,2) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LF_THIGH_X_fr_LF_HIP& HomogeneousTransforms::Type_fr_LF_THIGH_X_fr_LF_HIP::update(const state_t& q)
{
    Scalar sin_q_LF_HFE  = ScalarTraits::sin( q(LF_HFE) );
    Scalar cos_q_LF_HFE  = ScalarTraits::cos( q(LF_HFE) );
    (*this)(0,0) = cos_q_LF_HFE;
    (*this)(0,1) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(0,2) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(1,0) = -sin_q_LF_HFE;
    (*this)(1,1) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(1,2) = sin_rx_LF_HFE * cos_q_LF_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_LF_HIP_X_fr_LF_THIGH::Type_fr_LF_HIP_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = -sin_rx_LF_HFE;    // Maxima DSL: -sin(_k__rx_LF_HFE)
    (*this)(1,3) = 0.0;
    (*this)(2,2) = cos_rx_LF_HFE;    // Maxima DSL: cos(_k__rx_LF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LF_HIP_X_fr_LF_THIGH& HomogeneousTransforms::Type_fr_LF_HIP_X_fr_LF_THIGH::update(const state_t& q)
{
    Scalar sin_q_LF_HFE  = ScalarTraits::sin( q(LF_HFE) );
    Scalar cos_q_LF_HFE  = ScalarTraits::cos( q(LF_HFE) );
    (*this)(0,0) = cos_q_LF_HFE;
    (*this)(0,1) = -sin_q_LF_HFE;
    (*this)(1,0) = cos_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(1,1) = cos_rx_LF_HFE * cos_q_LF_HFE;
    (*this)(2,0) = sin_rx_LF_HFE * sin_q_LF_HFE;
    (*this)(2,1) = sin_rx_LF_HFE * cos_q_LF_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_LF_SHANK_X_fr_LF_THIGH::Type_fr_LF_SHANK_X_fr_LF_THIGH()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(2,2) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
    (*this)(2,3) =  ty_LF_KFE * sin_rx_LF_KFE;    // Maxima DSL: _k__ty_LF_KFE*sin(_k__rx_LF_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LF_SHANK_X_fr_LF_THIGH& HomogeneousTransforms::Type_fr_LF_SHANK_X_fr_LF_THIGH::update(const state_t& q)
{
    Scalar sin_q_LF_KFE  = ScalarTraits::sin( q(LF_KFE) );
    Scalar cos_q_LF_KFE  = ScalarTraits::cos( q(LF_KFE) );
    (*this)(0,0) = cos_q_LF_KFE;
    (*this)(0,1) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(0,2) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(0,3) = (- ty_LF_KFE * cos_rx_LF_KFE * sin_q_LF_KFE)-( tx_LF_KFE * cos_q_LF_KFE);
    (*this)(1,0) = -sin_q_LF_KFE;
    (*this)(1,1) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(1,2) = sin_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(1,3) = ( tx_LF_KFE * sin_q_LF_KFE)-( ty_LF_KFE * cos_rx_LF_KFE * cos_q_LF_KFE);
    return *this;
}
HomogeneousTransforms::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_LF_KFE;    // Maxima DSL: _k__tx_LF_KFE
    (*this)(1,2) = -sin_rx_LF_KFE;    // Maxima DSL: -sin(_k__rx_LF_KFE)
    (*this)(1,3) =  ty_LF_KFE;    // Maxima DSL: _k__ty_LF_KFE
    (*this)(2,2) = cos_rx_LF_KFE;    // Maxima DSL: cos(_k__rx_LF_KFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LF_THIGH_X_fr_LF_SHANK& HomogeneousTransforms::Type_fr_LF_THIGH_X_fr_LF_SHANK::update(const state_t& q)
{
    Scalar sin_q_LF_KFE  = ScalarTraits::sin( q(LF_KFE) );
    Scalar cos_q_LF_KFE  = ScalarTraits::cos( q(LF_KFE) );
    (*this)(0,0) = cos_q_LF_KFE;
    (*this)(0,1) = -sin_q_LF_KFE;
    (*this)(1,0) = cos_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(1,1) = cos_rx_LF_KFE * cos_q_LF_KFE;
    (*this)(2,0) = sin_rx_LF_KFE * sin_q_LF_KFE;
    (*this)(2,1) = sin_rx_LF_KFE * cos_q_LF_KFE;
    return *this;
}
HomogeneousTransforms::Type_fr_RF_HIP_X_fr_base::Type_fr_RF_HIP_X_fr_base()
{
    (*this)(2,0) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(2,1) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,2) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,3) = ((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * cos_ry_RF_HAA)-( tx_RF_HAA * sin_ry_RF_HAA);    // Maxima DSL: (_k__ty_RF_HAA*sin(_k__rx_RF_HAA)-_k__tz_RF_HAA*cos(_k__rx_RF_HAA))*cos(_k__ry_RF_HAA)-_k__tx_RF_HAA*sin(_k__ry_RF_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RF_HIP_X_fr_base& HomogeneousTransforms::Type_fr_RF_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RF_HAA  = ScalarTraits::sin( q(RF_HAA) );
    Scalar cos_q_RF_HAA  = ScalarTraits::cos( q(RF_HAA) );
    (*this)(0,0) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(0,1) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(0,2) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(0,3) = (((((- tz_RF_HAA * sin_rx_RF_HAA)-( ty_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA)+( tx_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+((((((( tz_RF_HAA * cos_rx_RF_HAA)-( ty_RF_HAA * sin_rx_RF_HAA)) * sin_ry_RF_HAA)-( tx_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+(((- tz_RF_HAA * sin_rx_RF_HAA)-( ty_RF_HAA * cos_rx_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,0) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(1,1) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,2) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,3) = ((((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA)+( tx_RF_HAA * cos_ry_RF_HAA)) * cos_rz_RF_HAA)+((( tz_RF_HAA * sin_rx_RF_HAA)+( ty_RF_HAA * cos_rx_RF_HAA)) * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((((- tz_RF_HAA * sin_rx_RF_HAA)-( ty_RF_HAA * cos_rx_RF_HAA)) * cos_rz_RF_HAA)+((((( ty_RF_HAA * sin_rx_RF_HAA)-( tz_RF_HAA * cos_rx_RF_HAA)) * sin_ry_RF_HAA)+( tx_RF_HAA * cos_ry_RF_HAA)) * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
{
    (*this)(0,2) = sin_ry_RF_HAA;    // Maxima DSL: sin(_k__ry_RF_HAA)
    (*this)(0,3) =  tx_RF_HAA;    // Maxima DSL: _k__tx_RF_HAA
    (*this)(1,2) = -sin_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: -sin(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(1,3) =  ty_RF_HAA;    // Maxima DSL: _k__ty_RF_HAA
    (*this)(2,2) = cos_rx_RF_HAA * cos_ry_RF_HAA;    // Maxima DSL: cos(_k__rx_RF_HAA)*cos(_k__ry_RF_HAA)
    (*this)(2,3) =  tz_RF_HAA;    // Maxima DSL: _k__tz_RF_HAA
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RF_HIP& HomogeneousTransforms::Type_fr_base_X_fr_RF_HIP::update(const state_t& q)
{
    Scalar sin_q_RF_HAA  = ScalarTraits::sin( q(RF_HAA) );
    Scalar cos_q_RF_HAA  = ScalarTraits::cos( q(RF_HAA) );
    (*this)(0,0) = (cos_ry_RF_HAA * cos_rz_RF_HAA * cos_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * sin_q_RF_HAA);
    (*this)(0,1) = (-cos_ry_RF_HAA * cos_rz_RF_HAA * sin_q_RF_HAA)-(cos_ry_RF_HAA * sin_rz_RF_HAA * cos_q_RF_HAA);
    (*this)(1,0) = (((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(1,1) = (((-sin_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(cos_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((cos_rx_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(2,0) = (((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * sin_rz_RF_HAA)-(cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)) * cos_q_RF_HAA);
    (*this)(2,1) = (((cos_rx_RF_HAA * sin_ry_RF_HAA * cos_rz_RF_HAA)-(sin_rx_RF_HAA * sin_rz_RF_HAA)) * sin_q_RF_HAA)+(((sin_rx_RF_HAA * cos_rz_RF_HAA)+(cos_rx_RF_HAA * sin_ry_RF_HAA * sin_rz_RF_HAA)) * cos_q_RF_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(2,2) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RF_THIGH_X_fr_RF_HIP& HomogeneousTransforms::Type_fr_RF_THIGH_X_fr_RF_HIP::update(const state_t& q)
{
    Scalar sin_q_RF_HFE  = ScalarTraits::sin( q(RF_HFE) );
    Scalar cos_q_RF_HFE  = ScalarTraits::cos( q(RF_HFE) );
    (*this)(0,0) = cos_q_RF_HFE;
    (*this)(0,1) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(0,2) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(1,0) = -sin_q_RF_HFE;
    (*this)(1,1) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(1,2) = sin_rx_RF_HFE * cos_q_RF_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_SHANK_TO_FOOT::Type_fr_SHANK_TO_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.308;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
HomogeneousTransforms::Type_fr_RF_HIP_X_fr_RF_THIGH::Type_fr_RF_HIP_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = -sin_rx_RF_HFE;    // Maxima DSL: -sin(_k__rx_RF_HFE)
    (*this)(1,3) = 0.0;
    (*this)(2,2) = cos_rx_RF_HFE;    // Maxima DSL: cos(_k__rx_RF_HFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RF_HIP_X_fr_RF_THIGH& HomogeneousTransforms::Type_fr_RF_HIP_X_fr_RF_THIGH::update(const state_t& q)
{
    Scalar sin_q_RF_HFE  = ScalarTraits::sin( q(RF_HFE) );
    Scalar cos_q_RF_HFE  = ScalarTraits::cos( q(RF_HFE) );
    (*this)(0,0) = cos_q_RF_HFE;
    (*this)(0,1) = -sin_q_RF_HFE;
    (*this)(1,0) = cos_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(1,1) = cos_rx_RF_HFE * cos_q_RF_HFE;
    (*this)(2,0) = sin_rx_RF_HFE * sin_q_RF_HFE;
    (*this)(2,1) = sin_rx_RF_HFE * cos_q_RF_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_RF_SHANK_X_fr_RF_THIGH::Type_fr_RF_SHANK_X_fr_RF_THIGH()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(2,2) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RF_SHANK_X_fr_RF_THIGH& HomogeneousTransforms::Type_fr_RF_SHANK_X_fr_RF_THIGH::update(const state_t& q)
{
    Scalar sin_q_RF_KFE  = ScalarTraits::sin( q(RF_KFE) );
    Scalar cos_q_RF_KFE  = ScalarTraits::cos( q(RF_KFE) );
    (*this)(0,0) = cos_q_RF_KFE;
    (*this)(0,1) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(0,2) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(0,3) = - tx_RF_KFE * cos_q_RF_KFE;
    (*this)(1,0) = -sin_q_RF_KFE;
    (*this)(1,1) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(1,2) = sin_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(1,3) =  tx_RF_KFE * sin_q_RF_KFE;
    return *this;
}
HomogeneousTransforms::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_RF_KFE;    // Maxima DSL: _k__tx_RF_KFE
    (*this)(1,2) = -sin_rx_RF_KFE;    // Maxima DSL: -sin(_k__rx_RF_KFE)
    (*this)(1,3) = 0.0;
    (*this)(2,2) = cos_rx_RF_KFE;    // Maxima DSL: cos(_k__rx_RF_KFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RF_THIGH_X_fr_RF_SHANK& HomogeneousTransforms::Type_fr_RF_THIGH_X_fr_RF_SHANK::update(const state_t& q)
{
    Scalar sin_q_RF_KFE  = ScalarTraits::sin( q(RF_KFE) );
    Scalar cos_q_RF_KFE  = ScalarTraits::cos( q(RF_KFE) );
    (*this)(0,0) = cos_q_RF_KFE;
    (*this)(0,1) = -sin_q_RF_KFE;
    (*this)(1,0) = cos_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(1,1) = cos_rx_RF_KFE * cos_q_RF_KFE;
    (*this)(2,0) = sin_rx_RF_KFE * sin_q_RF_KFE;
    (*this)(2,1) = sin_rx_RF_KFE * cos_q_RF_KFE;
    return *this;
}
HomogeneousTransforms::Type_fr_RH_HIP_X_fr_base::Type_fr_RH_HIP_X_fr_base()
{
    (*this)(2,0) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(2,1) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,2) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,3) = ((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * cos_ry_RH_HAA)-( tx_RH_HAA * sin_ry_RH_HAA);    // Maxima DSL: (_k__ty_RH_HAA*sin(_k__rx_RH_HAA)-_k__tz_RH_HAA*cos(_k__rx_RH_HAA))*cos(_k__ry_RH_HAA)-_k__tx_RH_HAA*sin(_k__ry_RH_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RH_HIP_X_fr_base& HomogeneousTransforms::Type_fr_RH_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RH_HAA  = ScalarTraits::sin( q(RH_HAA) );
    Scalar cos_q_RH_HAA  = ScalarTraits::cos( q(RH_HAA) );
    (*this)(0,0) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(0,1) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(0,2) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(0,3) = (((((- tz_RH_HAA * sin_rx_RH_HAA)-( ty_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA)+( tx_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+((((((( tz_RH_HAA * cos_rx_RH_HAA)-( ty_RH_HAA * sin_rx_RH_HAA)) * sin_ry_RH_HAA)-( tx_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+(((- tz_RH_HAA * sin_rx_RH_HAA)-( ty_RH_HAA * cos_rx_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,0) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(1,1) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,2) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,3) = ((((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA)+( tx_RH_HAA * cos_ry_RH_HAA)) * cos_rz_RH_HAA)+((( tz_RH_HAA * sin_rx_RH_HAA)+( ty_RH_HAA * cos_rx_RH_HAA)) * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((((- tz_RH_HAA * sin_rx_RH_HAA)-( ty_RH_HAA * cos_rx_RH_HAA)) * cos_rz_RH_HAA)+((((( ty_RH_HAA * sin_rx_RH_HAA)-( tz_RH_HAA * cos_rx_RH_HAA)) * sin_ry_RH_HAA)+( tx_RH_HAA * cos_ry_RH_HAA)) * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
{
    (*this)(0,2) = sin_ry_RH_HAA;    // Maxima DSL: sin(_k__ry_RH_HAA)
    (*this)(0,3) =  tx_RH_HAA;    // Maxima DSL: _k__tx_RH_HAA
    (*this)(1,2) = -sin_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: -sin(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(1,3) =  ty_RH_HAA;    // Maxima DSL: _k__ty_RH_HAA
    (*this)(2,2) = cos_rx_RH_HAA * cos_ry_RH_HAA;    // Maxima DSL: cos(_k__rx_RH_HAA)*cos(_k__ry_RH_HAA)
    (*this)(2,3) =  tz_RH_HAA;    // Maxima DSL: _k__tz_RH_HAA
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RH_HIP& HomogeneousTransforms::Type_fr_base_X_fr_RH_HIP::update(const state_t& q)
{
    Scalar sin_q_RH_HAA  = ScalarTraits::sin( q(RH_HAA) );
    Scalar cos_q_RH_HAA  = ScalarTraits::cos( q(RH_HAA) );
    (*this)(0,0) = (cos_ry_RH_HAA * cos_rz_RH_HAA * cos_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * sin_q_RH_HAA);
    (*this)(0,1) = (-cos_ry_RH_HAA * cos_rz_RH_HAA * sin_q_RH_HAA)-(cos_ry_RH_HAA * sin_rz_RH_HAA * cos_q_RH_HAA);
    (*this)(1,0) = (((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(1,1) = (((-sin_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(cos_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((cos_rx_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(2,0) = (((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * sin_rz_RH_HAA)-(cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)) * cos_q_RH_HAA);
    (*this)(2,1) = (((cos_rx_RH_HAA * sin_ry_RH_HAA * cos_rz_RH_HAA)-(sin_rx_RH_HAA * sin_rz_RH_HAA)) * sin_q_RH_HAA)+(((sin_rx_RH_HAA * cos_rz_RH_HAA)+(cos_rx_RH_HAA * sin_ry_RH_HAA * sin_rz_RH_HAA)) * cos_q_RH_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(2,2) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RH_THIGH_X_fr_RH_HIP& HomogeneousTransforms::Type_fr_RH_THIGH_X_fr_RH_HIP::update(const state_t& q)
{
    Scalar sin_q_RH_HFE  = ScalarTraits::sin( q(RH_HFE) );
    Scalar cos_q_RH_HFE  = ScalarTraits::cos( q(RH_HFE) );
    (*this)(0,0) = cos_q_RH_HFE;
    (*this)(0,1) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(0,2) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(1,0) = -sin_q_RH_HFE;
    (*this)(1,1) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(1,2) = sin_rx_RH_HFE * cos_q_RH_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_RH_HIP_X_fr_RH_THIGH::Type_fr_RH_HIP_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = -sin_rx_RH_HFE;    // Maxima DSL: -sin(_k__rx_RH_HFE)
    (*this)(1,3) = 0.0;
    (*this)(2,2) = cos_rx_RH_HFE;    // Maxima DSL: cos(_k__rx_RH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RH_HIP_X_fr_RH_THIGH& HomogeneousTransforms::Type_fr_RH_HIP_X_fr_RH_THIGH::update(const state_t& q)
{
    Scalar sin_q_RH_HFE  = ScalarTraits::sin( q(RH_HFE) );
    Scalar cos_q_RH_HFE  = ScalarTraits::cos( q(RH_HFE) );
    (*this)(0,0) = cos_q_RH_HFE;
    (*this)(0,1) = -sin_q_RH_HFE;
    (*this)(1,0) = cos_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(1,1) = cos_rx_RH_HFE * cos_q_RH_HFE;
    (*this)(2,0) = sin_rx_RH_HFE * sin_q_RH_HFE;
    (*this)(2,1) = sin_rx_RH_HFE * cos_q_RH_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_RH_SHANK_X_fr_RH_THIGH::Type_fr_RH_SHANK_X_fr_RH_THIGH()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(2,2) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RH_SHANK_X_fr_RH_THIGH& HomogeneousTransforms::Type_fr_RH_SHANK_X_fr_RH_THIGH::update(const state_t& q)
{
    Scalar sin_q_RH_KFE  = ScalarTraits::sin( q(RH_KFE) );
    Scalar cos_q_RH_KFE  = ScalarTraits::cos( q(RH_KFE) );
    (*this)(0,0) = cos_q_RH_KFE;
    (*this)(0,1) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(0,2) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(0,3) = - tx_RH_KFE * cos_q_RH_KFE;
    (*this)(1,0) = -sin_q_RH_KFE;
    (*this)(1,1) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(1,2) = sin_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(1,3) =  tx_RH_KFE * sin_q_RH_KFE;
    return *this;
}
HomogeneousTransforms::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_RH_KFE;    // Maxima DSL: _k__tx_RH_KFE
    (*this)(1,2) = -sin_rx_RH_KFE;    // Maxima DSL: -sin(_k__rx_RH_KFE)
    (*this)(1,3) = 0.0;
    (*this)(2,2) = cos_rx_RH_KFE;    // Maxima DSL: cos(_k__rx_RH_KFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RH_THIGH_X_fr_RH_SHANK& HomogeneousTransforms::Type_fr_RH_THIGH_X_fr_RH_SHANK::update(const state_t& q)
{
    Scalar sin_q_RH_KFE  = ScalarTraits::sin( q(RH_KFE) );
    Scalar cos_q_RH_KFE  = ScalarTraits::cos( q(RH_KFE) );
    (*this)(0,0) = cos_q_RH_KFE;
    (*this)(0,1) = -sin_q_RH_KFE;
    (*this)(1,0) = cos_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(1,1) = cos_rx_RH_KFE * cos_q_RH_KFE;
    (*this)(2,0) = sin_rx_RH_KFE * sin_q_RH_KFE;
    (*this)(2,1) = sin_rx_RH_KFE * cos_q_RH_KFE;
    return *this;
}
HomogeneousTransforms::Type_fr_LH_HIP_X_fr_base::Type_fr_LH_HIP_X_fr_base()
{
    (*this)(2,0) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(2,1) = 0.0;
    (*this)(2,2) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
    (*this)(2,3) = (- tx_LH_HAA * sin_ry_LH_HAA)-( tz_LH_HAA * cos_ry_LH_HAA);    // Maxima DSL: (-_k__tx_LH_HAA*sin(_k__ry_LH_HAA))-_k__tz_LH_HAA*cos(_k__ry_LH_HAA)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LH_HIP_X_fr_base& HomogeneousTransforms::Type_fr_LH_HIP_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_LH_HAA  = ScalarTraits::sin( q(LH_HAA) );
    Scalar cos_q_LH_HAA  = ScalarTraits::cos( q(LH_HAA) );
    (*this)(0,0) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(0,1) = sin_q_LH_HAA;
    (*this)(0,2) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(0,3) = ((( tz_LH_HAA * sin_ry_LH_HAA)-( tx_LH_HAA * cos_ry_LH_HAA)) * cos_q_LH_HAA)-( ty_LH_HAA * sin_q_LH_HAA);
    (*this)(1,0) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(1,1) = cos_q_LH_HAA;
    (*this)(1,2) = sin_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(1,3) = ((( tx_LH_HAA * cos_ry_LH_HAA)-( tz_LH_HAA * sin_ry_LH_HAA)) * sin_q_LH_HAA)-( ty_LH_HAA * cos_q_LH_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
{
    (*this)(0,2) = sin_ry_LH_HAA;    // Maxima DSL: sin(_k__ry_LH_HAA)
    (*this)(0,3) =  tx_LH_HAA;    // Maxima DSL: _k__tx_LH_HAA
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_LH_HAA;    // Maxima DSL: _k__ty_LH_HAA
    (*this)(2,2) = cos_ry_LH_HAA;    // Maxima DSL: cos(_k__ry_LH_HAA)
    (*this)(2,3) =  tz_LH_HAA;    // Maxima DSL: _k__tz_LH_HAA
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_LH_HIP& HomogeneousTransforms::Type_fr_base_X_fr_LH_HIP::update(const state_t& q)
{
    Scalar sin_q_LH_HAA  = ScalarTraits::sin( q(LH_HAA) );
    Scalar cos_q_LH_HAA  = ScalarTraits::cos( q(LH_HAA) );
    (*this)(0,0) = cos_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(0,1) = -cos_ry_LH_HAA * sin_q_LH_HAA;
    (*this)(1,0) = sin_q_LH_HAA;
    (*this)(1,1) = cos_q_LH_HAA;
    (*this)(2,0) = -sin_ry_LH_HAA * cos_q_LH_HAA;
    (*this)(2,1) = sin_ry_LH_HAA * sin_q_LH_HAA;
    return *this;
}
HomogeneousTransforms::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(2,2) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LH_THIGH_X_fr_LH_HIP& HomogeneousTransforms::Type_fr_LH_THIGH_X_fr_LH_HIP::update(const state_t& q)
{
    Scalar sin_q_LH_HFE  = ScalarTraits::sin( q(LH_HFE) );
    Scalar cos_q_LH_HFE  = ScalarTraits::cos( q(LH_HFE) );
    (*this)(0,0) = cos_q_LH_HFE;
    (*this)(0,1) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(0,2) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(1,0) = -sin_q_LH_HFE;
    (*this)(1,1) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(1,2) = sin_rx_LH_HFE * cos_q_LH_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_LH_HIP_X_fr_LH_THIGH::Type_fr_LH_HIP_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = -sin_rx_LH_HFE;    // Maxima DSL: -sin(_k__rx_LH_HFE)
    (*this)(1,3) = 0.0;
    (*this)(2,2) = cos_rx_LH_HFE;    // Maxima DSL: cos(_k__rx_LH_HFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LH_HIP_X_fr_LH_THIGH& HomogeneousTransforms::Type_fr_LH_HIP_X_fr_LH_THIGH::update(const state_t& q)
{
    Scalar sin_q_LH_HFE  = ScalarTraits::sin( q(LH_HFE) );
    Scalar cos_q_LH_HFE  = ScalarTraits::cos( q(LH_HFE) );
    (*this)(0,0) = cos_q_LH_HFE;
    (*this)(0,1) = -sin_q_LH_HFE;
    (*this)(1,0) = cos_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(1,1) = cos_rx_LH_HFE * cos_q_LH_HFE;
    (*this)(2,0) = sin_rx_LH_HFE * sin_q_LH_HFE;
    (*this)(2,1) = sin_rx_LH_HFE * cos_q_LH_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_LH_SHANK_X_fr_LH_THIGH::Type_fr_LH_SHANK_X_fr_LH_THIGH()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(2,2) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
    (*this)(2,3) =  ty_LH_KFE * sin_rx_LH_KFE;    // Maxima DSL: _k__ty_LH_KFE*sin(_k__rx_LH_KFE)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LH_SHANK_X_fr_LH_THIGH& HomogeneousTransforms::Type_fr_LH_SHANK_X_fr_LH_THIGH::update(const state_t& q)
{
    Scalar sin_q_LH_KFE  = ScalarTraits::sin( q(LH_KFE) );
    Scalar cos_q_LH_KFE  = ScalarTraits::cos( q(LH_KFE) );
    (*this)(0,0) = cos_q_LH_KFE;
    (*this)(0,1) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(0,2) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(0,3) = (- ty_LH_KFE * cos_rx_LH_KFE * sin_q_LH_KFE)-( tx_LH_KFE * cos_q_LH_KFE);
    (*this)(1,0) = -sin_q_LH_KFE;
    (*this)(1,1) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(1,2) = sin_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(1,3) = ( tx_LH_KFE * sin_q_LH_KFE)-( ty_LH_KFE * cos_rx_LH_KFE * cos_q_LH_KFE);
    return *this;
}
HomogeneousTransforms::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_LH_KFE;    // Maxima DSL: _k__tx_LH_KFE
    (*this)(1,2) = -sin_rx_LH_KFE;    // Maxima DSL: -sin(_k__rx_LH_KFE)
    (*this)(1,3) =  ty_LH_KFE;    // Maxima DSL: _k__ty_LH_KFE
    (*this)(2,2) = cos_rx_LH_KFE;    // Maxima DSL: cos(_k__rx_LH_KFE)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_LH_THIGH_X_fr_LH_SHANK& HomogeneousTransforms::Type_fr_LH_THIGH_X_fr_LH_SHANK::update(const state_t& q)
{
    Scalar sin_q_LH_KFE  = ScalarTraits::sin( q(LH_KFE) );
    Scalar cos_q_LH_KFE  = ScalarTraits::cos( q(LH_KFE) );
    (*this)(0,0) = cos_q_LH_KFE;
    (*this)(0,1) = -sin_q_LH_KFE;
    (*this)(1,0) = cos_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(1,1) = cos_rx_LH_KFE * cos_q_LH_KFE;
    (*this)(2,0) = sin_rx_LH_KFE * sin_q_LH_KFE;
    (*this)(2,1) = sin_rx_LH_KFE * cos_q_LH_KFE;
    return *this;
}

