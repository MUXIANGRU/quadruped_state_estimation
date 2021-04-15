#ifndef SIMPLEDOG_JACOBIANS_H_
#define SIMPLEDOG_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "model_constants.h"

namespace iit {
namespace simpledog {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians
{
    public:
        class Type_fr_base_J_LF_FOOT : public JacobianT<3, Type_fr_base_J_LF_FOOT>
        {
        public:
            Type_fr_base_J_LF_FOOT();
            const Type_fr_base_J_LF_FOOT& update(const JointState&);
        protected:
        };
        
        class Type_fr_base_J_RF_FOOT : public JacobianT<3, Type_fr_base_J_RF_FOOT>
        {
        public:
            Type_fr_base_J_RF_FOOT();
            const Type_fr_base_J_RF_FOOT& update(const JointState&);
        protected:
        };
        
        class Type_fr_base_J_LH_FOOT : public JacobianT<3, Type_fr_base_J_LH_FOOT>
        {
        public:
            Type_fr_base_J_LH_FOOT();
            const Type_fr_base_J_LH_FOOT& update(const JointState&);
        protected:
        };
        
        class Type_fr_base_J_RH_FOOT : public JacobianT<3, Type_fr_base_J_RH_FOOT>
        {
        public:
            Type_fr_base_J_RH_FOOT();
            const Type_fr_base_J_RH_FOOT& update(const JointState&);
        protected:
        };
        
        class Type_imu_link_J_LF_FOOT : public JacobianT<3, Type_imu_link_J_LF_FOOT>
        {
        public:
            Type_imu_link_J_LF_FOOT();
            const Type_imu_link_J_LF_FOOT& update(const JointState&);
        protected:
        };
        
        class Type_imu_link_J_RF_FOOT : public JacobianT<3, Type_imu_link_J_RF_FOOT>
        {
        public:
            Type_imu_link_J_RF_FOOT();
            const Type_imu_link_J_RF_FOOT& update(const JointState&);
        protected:
        };
        
        class Type_imu_link_J_LH_FOOT : public JacobianT<3, Type_imu_link_J_LH_FOOT>
        {
        public:
            Type_imu_link_J_LH_FOOT();
            const Type_imu_link_J_LH_FOOT& update(const JointState&);
        protected:
        };
        
        class Type_imu_link_J_RH_FOOT : public JacobianT<3, Type_imu_link_J_RH_FOOT>
        {
        public:
            Type_imu_link_J_RH_FOOT();
            const Type_imu_link_J_RH_FOOT& update(const JointState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
    public:
        Type_fr_base_J_LF_FOOT fr_base_J_LF_FOOT;
        Type_fr_base_J_RF_FOOT fr_base_J_RF_FOOT;
        Type_fr_base_J_LH_FOOT fr_base_J_LH_FOOT;
        Type_fr_base_J_RH_FOOT fr_base_J_RH_FOOT;
        Type_imu_link_J_LF_FOOT imu_link_J_LF_FOOT;
        Type_imu_link_J_RF_FOOT imu_link_J_RF_FOOT;
        Type_imu_link_J_LH_FOOT imu_link_J_LH_FOOT;
        Type_imu_link_J_RH_FOOT imu_link_J_RH_FOOT;

    protected:
        Params_lengths lengths;
        Params_angles angles;
};


}
}

#endif
