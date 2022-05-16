#ifndef IIWA_CLIK_NODE
#define IIWA_CLIK_NODE

#include "ros/ros.h"
#include "sun_iiwa_fri/IIWACommand.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"
#include "sun_robot_ros/ClikNode.h"
#include "std_srvs/Trigger.h"

namespace sun::iiwa {
class IIWAPivClikNode : public sun::ClikNode {
private:
protected:
  ros::Publisher pub_joints_cmd_;
  ros::ServiceServer srv_reset_virtual_joint_;
  ros::Subscriber joint_position_sub_;
  std::string joint_state_topic_str_;
  std::string joint_command_topic_str_;

public:
  static std::shared_ptr<sun::LBRiiwa7> makeIIWA_piv_Joint() {
    double alpha_iiwaVirtualFlangeEE = M_PI / 2;
    double robot2dh_offset_J7 = -M_PI / 2;
    double DZ_iiwaflange_ee = 0.205;
    std::shared_ptr<sun::LBRiiwa7> iiwa7 =
        std::make_shared<sun::LBRiiwa7>("iiwa7");
    iiwa7->pop_back_link();
    iiwa7->push_back_link(sun::RobotLinkRevolute(
        // a,alpha,d,
        0.0, alpha_iiwaVirtualFlangeEE, 0.126 + DZ_iiwaflange_ee,
        // robot2dh_offset, bool robot2dh_flip
        robot2dh_offset_J7, false,
        // Joint_Hard_limit_lower, Joint_Hard_limit_higher
        -175.0 * M_PI / 180.0, 175.0 * M_PI / 180.0,
        // hard_velocity_limit
        3.14,
        // string name
        "A7"));
    iiwa7->push_back_link(
        sun::RobotLinkRevolute(0.0,                 // a,
                               0.0,                 // alpha,
                               0.0,                 // d,
                               0.0,                 // robot2dh_offset,
                               false,               // robot2dh_flip,
                               -INFINITY,           // Joint_Hard_limit_lower,
                               INFINITY,            // Joint_Hard_limit_higher,
                               INFINITY,            // hard_velocity_limit,
                               "iiwa_virtual_joint" // name
                               ));
    return iiwa7;
  }

  IIWAPivClikNode(
      const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
      const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"))
      : ClikNode(IIWAPivClikNode::makeIIWA_piv_Joint(), nh_for_topics,
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
                                        &IIWAPivClikNode::joint_position_cb, this);

    ClikNode::run_init();

    // Publishers
    pub_joints_cmd_ =
        nh_.advertise<sun_iiwa_fri::IIWACommand>(joint_command_topic_str_, 1);

    // Services
    srv_reset_virtual_joint_ =
        nh_.advertiseService("reset_virtual_joint",
                             &IIWAPivClikNode::resetVirtualJoint_srv_cb, this);
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

    double virtual_joint_pos = 0.0;
    {
      TooN::Vector<> currentClikqR =
          clik_->robot_->joints_DH2Robot(clik_integrator_.getJointsDH());
      virtual_joint_pos = currentClikqR[currentClikqR.size() - 1];
    }

    // wait joint position
    if (wait_new_sample) {
      ROS_INFO_STREAM("IIWA wait new joint position...");
      b_joint_state_arrived = false;
    }

    if (b_joint_state_arrived) {
      TooN::Vector<8> qR_virtual;
      qR_virtual.slice<0, 7>() = qR;
      qR_virtual[qR_virtual.size() - 1] = virtual_joint_pos;
      return qR;
    }

    boost::shared_ptr<sun_iiwa_fri::IIWACommand const> msg =
        ros::topic::waitForMessage<sun_iiwa_fri::IIWACommand>(
            joint_state_topic_str_, nh_, ros::Duration(1.0));
    if (!msg) {
      ROS_WARN_STREAM("getJointPositionRobot waiting 15 sec...");
      msg = ros::topic::waitForMessage<sun_iiwa_fri::IIWACommand>(
          joint_state_topic_str_, nh_, ros::Duration(15.0));
      if (!msg) {
        ROS_ERROR_STREAM("getJointPositionRobot NO MSG!");
        exit(-1);
      }
    }

    qR[0] = msg->joint_position[0];
    qR[1] = msg->joint_position[1];
    qR[2] = msg->joint_position[2];
    qR[3] = msg->joint_position[3];
    qR[4] = msg->joint_position[4];
    qR[5] = msg->joint_position[5];
    qR[6] = msg->joint_position[6];

    TooN::Vector<8> qR_virtual;
    qR_virtual.slice<0, 7>() = qR;
    qR_virtual[qR_virtual.size() - 1] = virtual_joint_pos;

    b_joint_state_arrived = true;

    return qR_virtual;
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

  bool resetVirtualJoint_srv_cb(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res) {
    if (mode_ != sun_robot_msgs::ClikSetMode::Request::MODE_STOP) {
      ROS_ERROR_STREAM("Can't reset virtual joint if clik is not stopped!");
      res.success = false;
      return true;
    }
    auto joints = clik_integrator_.getJointsDH();
    joints[7] = 0.0;
    clik_integrator_.setJointsDH(joints);
    ROS_INFO_STREAM("Virtual joint reset");
    res.success = true;
    return true;
  }
};
} // namespace sun::iiwa

#endif