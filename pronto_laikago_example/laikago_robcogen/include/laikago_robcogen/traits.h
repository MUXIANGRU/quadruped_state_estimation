#ifndef IIT_ROBOGEN__LAIKAGO_TRAITS_H_
#define IIT_ROBOGEN__LAIKAGO_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace laikago {

struct Traits {
    typedef typename laikago::ScalarTraits ScalarTraits;

    typedef typename laikago::JointState JointState;

    typedef typename laikago::JointIdentifiers JointID;
    typedef typename laikago::LinkIdentifiers  LinkID;

    typedef typename laikago::HomogeneousTransforms HomogeneousTransforms;
    typedef typename laikago::MotionTransforms MotionTransforms;
    typedef typename laikago::ForceTransforms ForceTransforms;

    typedef typename laikago::dyn::InertiaProperties InertiaProperties;
    typedef typename laikago::dyn::ForwardDynamics FwdDynEngine;
    typedef typename laikago::dyn::InverseDynamics InvDynEngine;
    typedef typename laikago::dyn::JSIM JSIM;

    static const int joints_count = laikago::jointsCount;
    static const int links_count  = laikago::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return laikago::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return laikago::orderedLinkIDs;
}

}
}

#endif
