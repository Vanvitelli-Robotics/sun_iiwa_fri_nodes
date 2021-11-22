#ifndef IIWA_CLIK_NODELET
#define IIWA_CLIK_NODELET

#include "sun_iiwa_fri_nodes/IIWA_ClikNode.h"
#include "sun_robot_ros/ClikNodelet.h"

namespace sun::iiwa {
class IIWAClikNodelet : public sun::ClikNodelet<IIWAClikNode> {};
} // namespace sun::iiwa

#endif