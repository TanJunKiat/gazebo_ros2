// Copyright 2013 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_joint.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <sdf/sdf.hh>

namespace gazebo_plugins
{
class GazeboRosJointPrivate
{
public:
  
  /// A pointer to the Link, where force is applied
  std::vector<gazebo::physics::JointPtr> joints_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Wrench subscriber
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;

  /// Container for the wrench force that this plugin exerts on the body.
  geometry_msgs::msg::Wrench wrench_msg_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

};

GazeboRosJoint::GazeboRosJoint()
: impl_(std::make_unique<GazeboRosJointPrivate>())
{
}

GazeboRosJoint::~GazeboRosJoint()
{
}

void GazeboRosJoint::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  auto logger = rclcpp::get_logger("gazebo_ros_joint");

  // Target link
  if (!sdf->HasElement("joint_name")) {
    RCLCPP_ERROR(logger, "Force plugin missing <joint_name>, cannot proceed");
    return;
  }
  sdf::ElementPtr joint_elem = sdf->GetElement("joint_name");

    auto joint_name = joint_elem->Get<std::string>();

    auto joint = model->GetJoint(joint_name);
    if (!joint) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint %s does not exist!", joint_name.c_str());
    } else {
      impl_->joints_.push_back(joint);
    }
  
  // Subscribe to wrench messages
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->wrench_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Wrench>(
    "gazebo_ros_joint", qos.get_subscription_qos("gazebo_ros_joint", rclcpp::SystemDefaultsQoS()),
    std::bind(&GazeboRosJoint::OnRosWrenchMsg, this, std::placeholders::_1));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosJoint::OnUpdate, this));
}

void GazeboRosJoint::OnRosWrenchMsg(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
  impl_->wrench_msg_.force.x = msg->force.x;
  impl_->wrench_msg_.force.y = msg->force.y;
  impl_->wrench_msg_.force.z = msg->force.z;
  impl_->wrench_msg_.torque.x = msg->torque.x;
  impl_->wrench_msg_.torque.y = msg->torque.y;
  impl_->wrench_msg_.torque.z = msg->torque.z;
}

void GazeboRosJoint::OnUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosJoint::OnUpdate");
  IGN_PROFILE_BEGIN("Apply angles");
#endif
    impl_->joints_[0]->SetPosition(0,impl_->wrench_msg_.force.x,1);

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosJoint)
}  // namespace gazebo_plugins
