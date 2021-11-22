#include "sun_iiwa_fri_nodes/IIWA_FkineNode.h"
#include "sun_robot_ros/FkineNodelet.h"

namespace sun::iiwa {
class IIWA_FkineNodelet : public sun::FkineNodelet<IIWA_FkineNode> {};
} // namespace sun_iiwa_nodes