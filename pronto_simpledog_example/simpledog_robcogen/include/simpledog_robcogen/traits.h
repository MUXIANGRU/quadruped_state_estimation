#ifndef IIT_ROBOGEN__SIMPLEDOG_TRAITS_H_
#define IIT_ROBOGEN__SIMPLEDOG_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace simpledog {

struct Traits {
    typedef typename simpledog::ScalarTraits ScalarTraits;

    typedef typename simpledog::JointState JointState;

    typedef typename simpledog::JointIdentifiers JointID;
    typedef typename simpledog::LinkIdentifiers  LinkID;

    typedef typename simpledog::HomogeneousTransforms HomogeneousTransforms;
    typedef typename simpledog::MotionTransforms MotionTransforms;
    typedef typename simpledog::ForceTransforms ForceTransforms;

    typedef typename simpledog::dyn::InertiaProperties InertiaProperties;
    typedef typename simpledog::dyn::ForwardDynamics FwdDynEngine;
    typedef typename simpledog::dyn::InverseDynamics InvDynEngine;
    typedef typename simpledog::dyn::JSIM JSIM;

    static const int joints_count = simpledog::jointsCount;
    static const int links_count  = simpledog::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return simpledog::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return simpledog::orderedLinkIDs;
}

}
}

#endif
