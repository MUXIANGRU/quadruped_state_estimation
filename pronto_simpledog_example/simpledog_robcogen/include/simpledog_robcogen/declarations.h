#ifndef IIT_ROBOT_SIMPLEDOG_DECLARATIONS_H_
#define IIT_ROBOT_SIMPLEDOG_DECLARATIONS_H_

#include "rbd_types.h"

namespace iit {
namespace simpledog {

static constexpr int JointSpaceDimension = 12;
static constexpr int jointsCount = 12;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 13;

typedef Matrix<12, 1> Column12d;
typedef Column12d JointState;

enum JointIdentifiers {
    LF_HAA = 0
    , LF_HFE
    , LF_KFE
    , RF_HAA
    , RF_HFE
    , RF_KFE
    , RH_HAA
    , RH_HFE
    , RH_KFE
    , LH_HAA
    , LH_HFE
    , LH_KFE
};

enum LinkIdentifiers {
    BASE = 0
    , LF_HIP
    , LF_THIGH
    , LF_SHANK
    , RF_HIP
    , RF_THIGH
    , RF_SHANK
    , RH_HIP
    , RH_THIGH
    , RH_SHANK
    , LH_HIP
    , LH_THIGH
    , LH_SHANK
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {LF_HAA,LF_HFE,LF_KFE,RF_HAA,RF_HFE,RF_KFE,RH_HAA,RH_HFE,RH_KFE,LH_HAA,LH_HFE,LH_KFE};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,LF_HIP,LF_THIGH,LF_SHANK,RF_HIP,RF_THIGH,RF_SHANK,RH_HIP,RH_THIGH,RH_SHANK,LH_HIP,LH_THIGH,LH_SHANK};

}
}
#endif
