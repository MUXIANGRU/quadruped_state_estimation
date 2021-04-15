#pragma once

#include <pronto_quadruped_commons/feet_jacobians.h>
#include <laikago_robcogen/jacobians.h>
#include <pronto_quadruped_commons/leg_data_map.h>

namespace iit {
namespace laikago {
class FeetJacobians : public pronto::quadruped::FeetJacobians {
public:
    typedef pronto::quadruped::FootJac FootJac;   //3*3 matrix
    typedef pronto::quadruped::JointState JointState;   //12*1
    typedef pronto::quadruped::LegID LegID;  //enum LegID{LF=0, RF, LH, RH};

    inline FeetJacobians() {}

    virtual ~FeetJacobians() {}

    FootJac getFootJacobian(const JointState& q, const LegID& leg);
    FootJac getFootJacobianLF(const JointState& q);
    FootJac getFootJacobianRF(const JointState& q);
    FootJac getFootJacobianLH(const JointState& q);
    FootJac getFootJacobianRH(const JointState& q);

    FootJac getFootJacobian(const JointState &q, const LegID &leg,
                            const double& foot_x, const double& foot_y);

private:
    iit::laikago::Jacobians jacs_;
};
}

}
