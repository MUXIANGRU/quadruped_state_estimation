#ifndef IIT_ROBOT_SIMPLEDOG_MODEL_CONSTANTS_H_
#define IIT_ROBOT_SIMPLEDOG_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace iit {
namespace simpledog {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar ry_LF_HAA = 1.5707900524139404;
const Scalar sin_ry_LF_HAA = ScalarTraits::sin(ry_LF_HAA);
const Scalar cos_ry_LF_HAA = ScalarTraits::cos(ry_LF_HAA);
const Scalar rx_LF_HFE = -1.5707900524139404;
const Scalar sin_rx_LF_HFE = ScalarTraits::sin(rx_LF_HFE);
const Scalar cos_rx_LF_HFE = ScalarTraits::cos(rx_LF_HFE);
const Scalar rx_LF_KFE = -7.000000096013537E-6;
const Scalar sin_rx_LF_KFE = ScalarTraits::sin(rx_LF_KFE);
const Scalar cos_rx_LF_KFE = ScalarTraits::cos(rx_LF_KFE);
const Scalar rx_RF_HAA = -1.1071499586105347;
const Scalar sin_rx_RF_HAA = ScalarTraits::sin(rx_RF_HAA);
const Scalar cos_rx_RF_HAA = ScalarTraits::cos(rx_RF_HAA);
const Scalar ry_RF_HAA = -1.5707900524139404;
const Scalar sin_ry_RF_HAA = ScalarTraits::sin(ry_RF_HAA);
const Scalar cos_ry_RF_HAA = ScalarTraits::cos(ry_RF_HAA);
const Scalar rz_RF_HAA = 2.034440040588379;
const Scalar sin_rz_RF_HAA = ScalarTraits::sin(rz_RF_HAA);
const Scalar cos_rz_RF_HAA = ScalarTraits::cos(rz_RF_HAA);
const Scalar rx_RF_HFE = -1.5707900524139404;
const Scalar sin_rx_RF_HFE = ScalarTraits::sin(rx_RF_HFE);
const Scalar cos_rx_RF_HFE = ScalarTraits::cos(rx_RF_HFE);
const Scalar rx_RF_KFE = -7.000000096013537E-6;
const Scalar sin_rx_RF_KFE = ScalarTraits::sin(rx_RF_KFE);
const Scalar cos_rx_RF_KFE = ScalarTraits::cos(rx_RF_KFE);
const Scalar rx_RH_HAA = -1.1071499586105347;
const Scalar sin_rx_RH_HAA = ScalarTraits::sin(rx_RH_HAA);
const Scalar cos_rx_RH_HAA = ScalarTraits::cos(rx_RH_HAA);
const Scalar ry_RH_HAA = -1.5707900524139404;
const Scalar sin_ry_RH_HAA = ScalarTraits::sin(ry_RH_HAA);
const Scalar cos_ry_RH_HAA = ScalarTraits::cos(ry_RH_HAA);
const Scalar rz_RH_HAA = 2.034440040588379;
const Scalar sin_rz_RH_HAA = ScalarTraits::sin(rz_RH_HAA);
const Scalar cos_rz_RH_HAA = ScalarTraits::cos(rz_RH_HAA);
const Scalar rx_RH_HFE = -1.5707900524139404;
const Scalar sin_rx_RH_HFE = ScalarTraits::sin(rx_RH_HFE);
const Scalar cos_rx_RH_HFE = ScalarTraits::cos(rx_RH_HFE);
const Scalar rx_RH_KFE = -7.000000096013537E-6;
const Scalar sin_rx_RH_KFE = ScalarTraits::sin(rx_RH_KFE);
const Scalar cos_rx_RH_KFE = ScalarTraits::cos(rx_RH_KFE);
const Scalar ry_LH_HAA = 1.5707900524139404;
const Scalar sin_ry_LH_HAA = ScalarTraits::sin(ry_LH_HAA);
const Scalar cos_ry_LH_HAA = ScalarTraits::cos(ry_LH_HAA);
const Scalar rx_LH_HFE = -1.5707900524139404;
const Scalar sin_rx_LH_HFE = ScalarTraits::sin(rx_LH_HFE);
const Scalar cos_rx_LH_HFE = ScalarTraits::cos(rx_LH_HFE);
const Scalar rx_LH_KFE = -7.000000096013537E-6;
const Scalar sin_rx_LH_KFE = ScalarTraits::sin(rx_LH_KFE);
const Scalar cos_rx_LH_KFE = ScalarTraits::cos(rx_LH_KFE);
const Scalar rz_LF_FOOT = 7.000000096013537E-6;
const Scalar sin_rz_LF_FOOT = ScalarTraits::sin(rz_LF_FOOT);
const Scalar cos_rz_LF_FOOT = ScalarTraits::cos(rz_LF_FOOT);
const Scalar rz_LH_FOOT = 7.000000096013537E-6;
const Scalar sin_rz_LH_FOOT = ScalarTraits::sin(rz_LH_FOOT);
const Scalar cos_rz_LH_FOOT = ScalarTraits::cos(rz_LH_FOOT);
const Scalar tx_LF_HAA = 0.4269999861717224;
const Scalar ty_LF_HAA = 0.07500000298023224;
const Scalar tz_LF_HAA = -0.009499999694526196;
const Scalar tx_LF_KFE = 0.30799999833106995;
const Scalar ty_LF_KFE = 1.9999999949504854E-6;
const Scalar tx_RF_HAA = 0.4269999861717224;
const Scalar ty_RF_HAA = -0.07500000298023224;
const Scalar tz_RF_HAA = -0.009499999694526196;
const Scalar tx_RF_KFE = 0.30799999833106995;
const Scalar tx_RH_HAA = -0.4269999861717224;
const Scalar ty_RH_HAA = -0.07500000298023224;
const Scalar tz_RH_HAA = -0.009499999694526196;
const Scalar tx_RH_KFE = 0.30799999833106995;
const Scalar tx_LH_HAA = -0.4269999861717224;
const Scalar ty_LH_HAA = 0.07500000298023224;
const Scalar tz_LH_HAA = -0.009499999694526196;
const Scalar tx_LH_KFE = 0.30799999833106995;
const Scalar ty_LH_KFE = 1.9999999949504854E-6;
const Scalar tx_imu_link = 0.12999999523162842;
const Scalar tz_imu_link = 0.11999999731779099;
const Scalar tx_LF_FOOT = 0.30799999833106995;
const Scalar ty_LF_FOOT = 1.9999999949504854E-6;
const Scalar tz_LF_FOOT = 0.23000000417232513;
const Scalar tx_RF_FOOT = 0.30799999833106995;
const Scalar tz_RF_FOOT = 0.22053000330924988;
const Scalar tx_RH_FOOT = 0.30799999833106995;
const Scalar tz_RH_FOOT = 0.22053000330924988;
const Scalar tx_LH_FOOT = 0.30799999833106995;
const Scalar ty_LH_FOOT = 1.9999999949504854E-6;
const Scalar tz_LH_FOOT = 0.23000000417232513;
const Scalar m_base = 37.9010009765625;
const Scalar comx_base = 0.007023999933153391;
const Scalar comy_base = -7.340000011026859E-4;
const Scalar comz_base = 0.0016009999671950936;
const Scalar ix_base = 0.2655160129070282;
const Scalar ixy_base = -1.8600000475998968E-4;
const Scalar ixz_base = 0.0018860000418499112;
const Scalar iy_base = 2.0229198932647705;
const Scalar iyz_base = -3.600000127335079E-5;
const Scalar iz_base = 2.059969902038574;
const Scalar m_LF_HIP = 2.923099994659424;
const Scalar comx_LF_HIP = 6.880000000819564E-4;
const Scalar comy_LF_HIP = 0.10487999767065048;
const Scalar comz_LF_HIP = -0.003776000114157796;
const Scalar ix_LF_HIP = 0.037014998495578766;
const Scalar ixy_LF_HIP = 2.1100000594742596E-4;
const Scalar ixz_LF_HIP = -7.999999979801942E-6;
const Scalar iy_LF_HIP = 0.007911000400781631;
const Scalar iyz_LF_HIP = -0.0011579999700188637;
const Scalar iz_LF_HIP = 0.04039600118994713;
const Scalar m_LF_THIGH = 4.216400146484375;
const Scalar comx_LF_THIGH = 0.1847500056028366;
const Scalar comy_LF_THIGH = 3.400000059627928E-5;
const Scalar comz_LF_THIGH = 0.12906000018119812;
const Scalar ix_LF_THIGH = 0.07651600241661072;
const Scalar ixy_LF_THIGH = 2.700000004551839E-5;
const Scalar ixz_LF_THIGH = 0.10053499788045883;
const Scalar iy_LF_THIGH = 0.2423029989004135;
const Scalar iyz_LF_THIGH = 1.8999999156221747E-5;
const Scalar iz_LF_THIGH = 0.17315900325775146;
const Scalar m_LF_SHANK = 0.4778900146484375;
const Scalar comx_LF_SHANK = 0.21122199296951294;
const Scalar comy_LF_SHANK = 1.8699999782256782E-4;
const Scalar comz_LF_SHANK = 0.22578300535678864;
const Scalar ix_LF_SHANK = 0.024762000888586044;
const Scalar ixy_LF_SHANK = 1.8000000636675395E-5;
const Scalar ixz_LF_SHANK = 0.022810999304056168;
const Scalar iy_LF_SHANK = 0.05242700129747391;
const Scalar iyz_LF_SHANK = 1.9999999494757503E-5;
const Scalar iz_LF_SHANK = 0.028348000720143318;
const Scalar m_RF_HIP = 2.923110008239746;
const Scalar comx_RF_HIP = -6.880000000819564E-4;
const Scalar comy_RF_HIP = 0.10488200187683105;
const Scalar comz_RF_HIP = 0.003776000114157796;
const Scalar ix_RF_HIP = 0.03701699897646904;
const Scalar ixy_RF_HIP = -2.1100000594742596E-4;
const Scalar ixz_RF_HIP = -7.999999979801942E-6;
const Scalar iy_RF_HIP = 0.007911000400781631;
const Scalar iyz_RF_HIP = 0.0011579999700188637;
const Scalar iz_RF_HIP = 0.040396999567747116;
const Scalar m_RF_THIGH = 4.216360092163086;
const Scalar comx_RF_THIGH = 0.18474699556827545;
const Scalar comy_RF_THIGH = 3.300000025774352E-5;
const Scalar comz_RF_THIGH = 0.12905600666999817;
const Scalar ix_RF_THIGH = 0.07651100307703018;
const Scalar ixy_RF_THIGH = 2.5999999706982635E-5;
const Scalar ixz_RF_THIGH = 0.10052900016307831;
const Scalar iy_RF_THIGH = 0.24229200184345245;
const Scalar iyz_RF_THIGH = 1.8000000636675395E-5;
const Scalar iz_RF_THIGH = 0.17315199971199036;
const Scalar m_RF_SHANK = 0.4778890013694763;
const Scalar comx_RF_SHANK = 0.2112250030040741;
const Scalar comy_RF_SHANK = 1.849999971454963E-4;
const Scalar comz_RF_SHANK = 0.22508999705314636;
const Scalar ix_RF_SHANK = 0.024612000212073326;
const Scalar ixy_RF_SHANK = 1.8000000636675395E-5;
const Scalar ixz_RF_SHANK = 0.02270199917256832;
const Scalar iy_RF_SHANK = 0.052278000861406326;
const Scalar iyz_RF_SHANK = 1.9999999494757503E-5;
const Scalar iz_RF_SHANK = 0.028348999097943306;
const Scalar m_RH_HIP = 2.923110008239746;
const Scalar comx_RH_HIP = 6.880000000819564E-4;
const Scalar comy_RH_HIP = 0.10488200187683105;
const Scalar comz_RH_HIP = 0.003776000114157796;
const Scalar ix_RH_HIP = 0.03701699897646904;
const Scalar ixy_RH_HIP = 2.1100000594742596E-4;
const Scalar ixz_RH_HIP = 7.999999979801942E-6;
const Scalar iy_RH_HIP = 0.007911000400781631;
const Scalar iyz_RH_HIP = 0.0011579999700188637;
const Scalar iz_RH_HIP = 0.040396999567747116;
const Scalar m_RH_THIGH = 4.216360092163086;
const Scalar comx_RH_THIGH = 0.18474699556827545;
const Scalar comy_RH_THIGH = 3.300000025774352E-5;
const Scalar comz_RH_THIGH = 0.12905600666999817;
const Scalar ix_RH_THIGH = 0.07651100307703018;
const Scalar ixy_RH_THIGH = 2.5999999706982635E-5;
const Scalar ixz_RH_THIGH = 0.10052900016307831;
const Scalar iy_RH_THIGH = 0.24229200184345245;
const Scalar iyz_RH_THIGH = 1.8000000636675395E-5;
const Scalar iz_RH_THIGH = 0.17315199971199036;
const Scalar m_RH_SHANK = 0.4778890013694763;
const Scalar comx_RH_SHANK = 0.2112250030040741;
const Scalar comy_RH_SHANK = 1.849999971454963E-4;
const Scalar comz_RH_SHANK = 0.22508999705314636;
const Scalar ix_RH_SHANK = 0.024612000212073326;
const Scalar ixy_RH_SHANK = 1.8000000636675395E-5;
const Scalar ixz_RH_SHANK = 0.02270199917256832;
const Scalar iy_RH_SHANK = 0.052278000861406326;
const Scalar iyz_RH_SHANK = 1.9999999494757503E-5;
const Scalar iz_RH_SHANK = 0.028348999097943306;
const Scalar m_LH_HIP = 2.923099994659424;
const Scalar comx_LH_HIP = -6.880000000819564E-4;
const Scalar comy_LH_HIP = 0.10487999767065048;
const Scalar comz_LH_HIP = 0.003776000114157796;
const Scalar ix_LH_HIP = 0.037014998495578766;
const Scalar ixy_LH_HIP = -2.1100000594742596E-4;
const Scalar ixz_LH_HIP = -7.999999979801942E-6;
const Scalar iy_LH_HIP = 0.007911000400781631;
const Scalar iyz_LH_HIP = 0.0011579999700188637;
const Scalar iz_LH_HIP = 0.04039600118994713;
const Scalar m_LH_THIGH = 4.216360092163086;
const Scalar comx_LH_THIGH = 0.18471699953079224;
const Scalar comy_LH_THIGH = 3.400000059627928E-5;
const Scalar comz_LH_THIGH = 0.12905600666999817;
const Scalar ix_LH_THIGH = 0.07651100307703018;
const Scalar ixy_LH_THIGH = 2.700000004551839E-5;
const Scalar ixz_LH_THIGH = 0.10051299631595612;
const Scalar iy_LH_THIGH = 0.24224500358104706;
const Scalar iyz_LH_THIGH = 1.8999999156221747E-5;
const Scalar iz_LH_THIGH = 0.17310599982738495;
const Scalar m_LH_SHANK = 0.4778890013694763;
const Scalar comx_LH_SHANK = 0.2112250030040741;
const Scalar comy_LH_SHANK = 1.8699999782256782E-4;
const Scalar comz_LH_SHANK = 0.2257850021123886;
const Scalar ix_LH_SHANK = 0.024762000888586044;
const Scalar ixy_LH_SHANK = 1.8000000636675395E-5;
const Scalar ixz_LH_SHANK = 0.022810999304056168;
const Scalar iy_LH_SHANK = 0.052427999675273895;
const Scalar iyz_LH_SHANK = 1.9999999494757503E-5;
const Scalar iz_LH_SHANK = 0.028348999097943306;
const Scalar tz_LF_KFE = 0.0;
const Scalar ty_RF_FOOT = 0.0;
const Scalar ty_LF_HFE = 0.0;
const Scalar tz_RF_KFE = 0.0;
const Scalar ty_RF_HFE = 0.0;
const Scalar tz_LH_KFE = 0.0;
const Scalar ty_LH_HFE = 0.0;
const Scalar ty_RH_FOOT = 0.0;
const Scalar tz_RH_KFE = 0.0;
const Scalar ty_RH_HFE = 0.0;

}
}
#endif
