#ifndef SUN_IIWA_NODES_H_
#define SUN_IIWA_NODES_H_

#include "sensor_msgs/JointState.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"
#include "sun_robot_ros/FkineNode.h"

namespace sun::iiwa {

class IIWA_FkineNode : public FkineNode {
private:
  ros::Subscriber joint_sub_;

public:
  static std::shared_ptr<sun::LBRiiwa7> makeIIWA() {
    return std::make_shared<sun::LBRiiwa7>("iiwa7");
  }

  IIWA_FkineNode(
      const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
      const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"),
      ros::CallbackQueue *callbk_queue = ros::getGlobalCallbackQueue())
      : FkineNode(IIWA_FkineNode::makeIIWA(), nh_for_topics, nh_for_parmas,
                  callbk_queue) {}

  ~IIWA_FkineNode() = default;

  /**
     joint_sub_ = ...
   */
  virtual void registerJointSubscriber() override {
    joint_sub_ = nh_.subscribe("/iiwa/state/joint_state", 1,
                               &IIWA_FkineNode::joint_state_cb, this);
  }

  void joint_state_cb(const sensor_msgs::JointState::ConstPtr &joi_state_msg) {
    TooN::Vector<7> qR;
    TooN::Vector<7> qdotR;

    qR[0] = joi_state_msg->position[0];
    qR[1] = joi_state_msg->position[1];
    qR[2] = joi_state_msg->position[2];
    qR[3] = joi_state_msg->position[3];
    qR[4] = joi_state_msg->position[4];
    qR[5] = joi_state_msg->position[5];
    qR[6] = joi_state_msg->position[6];

    qdotR[0] = joi_state_msg->velocity[0];
    qdotR[1] = joi_state_msg->velocity[1];
    qdotR[2] = joi_state_msg->velocity[2];
    qdotR[3] = joi_state_msg->velocity[3];
    qdotR[4] = joi_state_msg->velocity[4];
    qdotR[5] = joi_state_msg->velocity[5];
    qdotR[6] = joi_state_msg->velocity[6];

    updateJoint(qR);
    updateJointVel(qdotR);

    publishAll();
  }
};
} // namespace sun::iiwa

#endif