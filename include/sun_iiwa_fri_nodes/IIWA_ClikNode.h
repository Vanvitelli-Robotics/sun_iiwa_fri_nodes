#ifndef IIWA_CLIK_NODE
#define IIWA_CLIK_NODE

#include "ros/ros.h"
#include "sun_iiwa_fri/IIWACommand.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"
#include "sun_robot_ros/ClikNode.h"

namespace sun::iiwa {
class IIWAClikNode : public sun::ClikNode {
private:
protected:
  ros::Publisher pub_joints_cmd_;
  ros::Subscriber joint_position_sub_;
  std::string joint_state_topic_str_;
  std::string joint_command_topic_str_;

public:
  IIWAClikNode(const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
               const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"))
      : ClikNode(std::make_shared<sun::LBRiiwa7>(), nh_for_topics,
                 nh_for_parmas) {

    nh_for_parmas.param("joint_state_topic", joint_state_topic_str_,
                        std::string("/iiwa/state/commanded_joint"));
    nh_for_parmas.param("joint_command_topic", joint_command_topic_str_,
                        std::string("/iiwa/command/joint_position"));

    clik_->b_checkHardJointVelLimits_ = false;
    clik_->b_checkSoftJointVelLimits_ = false;
    ROS_WARN_STREAM(ros::this_node::getName()
                    << " velocity safe check disabled!");
  }

  virtual void run_init() override {

    // Subscribers
    joint_position_sub_ = nh_.subscribe(joint_state_topic_str_, 1,
                                        &IIWAClikNode::joint_position_cb, this);

    ClikNode::run_init();

    // Publishers
    pub_joints_cmd_ =
        nh_.advertise<sun_iiwa_fri::IIWACommand>(joint_command_topic_str_, 1);
  }

  TooN::Vector<7> qR;
  volatile bool b_joint_state_arrived = false;
  void joint_position_cb(const sun_iiwa_fri::IIWACommandPtr &joi_state_msg) {
    qR[0] = joi_state_msg->joint_position[0];
    qR[1] = joi_state_msg->joint_position[1];
    qR[2] = joi_state_msg->joint_position[2];
    qR[3] = joi_state_msg->joint_position[3];
    qR[4] = joi_state_msg->joint_position[4];
    qR[5] = joi_state_msg->joint_position[5];
    qR[6] = joi_state_msg->joint_position[6];
    b_joint_state_arrived = true;
  }

  //! Cbs
  virtual TooN::Vector<> getJointPositionRobot(bool wait_new_sample) override {

    // wait joint position
    if (wait_new_sample) {
      ROS_INFO_STREAM("IIWA wait new joint position...");
      b_joint_state_arrived = false;
    }

    if (b_joint_state_arrived) {
      return qR;
    }

    boost::shared_ptr<sun_iiwa_fri::IIWACommand const> msg =
        ros::topic::waitForMessage<sun_iiwa_fri::IIWACommand>(
            joint_state_topic_str_, nh_, ros::Duration(1.0));
    if (!msg) {
      ROS_ERROR_STREAM("getJointPositionRobot NO MSG!");
      exit(-1);
    }
    qR[0] = msg->joint_position[0];
    qR[1] = msg->joint_position[1];
    qR[2] = msg->joint_position[2];
    qR[3] = msg->joint_position[3];
    qR[4] = msg->joint_position[4];
    qR[5] = msg->joint_position[5];
    qR[6] = msg->joint_position[6];
    b_joint_state_arrived = true;

    // ros::Time t0 = ros::Time::now();
    // while (ros::ok() && !b_joint_state_arrived) {
    //   spinOnce(ros::WallDuration(0.1));
    //   if ((ros::Time::now() - t0) > ros::Duration(1.0)) {
    //     joint_position_sub_ = nh_.subscribe(
    //         joint_state_topic_str_, 1, &IIWAClikNode::joint_position_cb,
    //         this);
    //     t0 = ros::Time::now();
    //     ROS_INFO_STREAM("IIWA wait new joint position...");
    //     // return getJointPositionRobot(wait_new_sample);
    //   }
    // }

    // if (wait_new_sample) {
    //   ROS_INFO_STREAM("IIWA joit position arrived!");
    // }

    return qR;
  }
  virtual void publishJointRobot(const TooN::Vector<> &qR,
                                 const TooN::Vector<> &qR_dot) override {
    sun_iiwa_fri::IIWACommandPtr out_msg(new sun_iiwa_fri::IIWACommand);

    out_msg->joint_position.resize(7);

    out_msg->joint_position[0] = qR[0];
    out_msg->joint_position[1] = qR[1];
    out_msg->joint_position[2] = qR[2];
    out_msg->joint_position[3] = qR[3];
    out_msg->joint_position[4] = qR[4];
    out_msg->joint_position[5] = qR[5];
    out_msg->joint_position[6] = qR[6];
    /*
        out_msg.velocity.a1 = dqR[0];
        out_msg.velocity.a2 = dqR[1];
        out_msg.velocity.a3 = dqR[2];
        out_msg.velocity.a4 = dqR[3];
        out_msg.velocity.a5 = dqR[4];
        out_msg.velocity.a6 = dqR[5];
        out_msg.velocity.a7 = dqR[6];
    */
    out_msg->header.frame_id = "iiwa";
    out_msg->header.stamp = ros::Time::now();

    pub_joints_cmd_.publish(out_msg);
  }
};
} // namespace sun::iiwa

#endif