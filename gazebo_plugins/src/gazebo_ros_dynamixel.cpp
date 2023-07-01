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
#include <gazebo_plugins/gazebo_ros_dynamixel.hpp>
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

//double angle_0;   			// previous angle
double dt = 0.001;			// dt
double angle_desired;   			// step of angle
double angle_1;   			// Current desired angle
double T = 21.5;  			// Given transfer function is T/(s + T)
//double rpm_max = 3.1416 * dt;			// Maximum RPM
double rpm_max = 0.7 * dt;			// Maximum RPM


double angle_0;
/*double angle_0_bit;
double angle_desired_bit;
double angle_1_bit;
double rpm_max_bit = 204.8 * dt;
double rotation_range = 5.236;
double bit = 1024.0;*/


namespace gazebo_plugins
{
class GazeboRosDynamixelPrivate
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

GazeboRosDynamixel::GazeboRosDynamixel()
: impl_(std::make_unique<GazeboRosDynamixelPrivate>())
{
}

GazeboRosDynamixel::~GazeboRosDynamixel()
{
}

void GazeboRosDynamixel::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  auto logger = rclcpp::get_logger("gazebo_ros_dynamixel");

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
    "gazebo_ros_dynamixel", qos.get_subscription_qos("gazebo_ros_dynamixel", rclcpp::SystemDefaultsQoS()),
    std::bind(&GazeboRosDynamixel::OnRosWrenchMsg, this, std::placeholders::_1));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosDynamixel::OnUpdate, this));
}

void GazeboRosDynamixel::OnRosWrenchMsg(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
  impl_->wrench_msg_.force.x = msg->force.x;
  impl_->wrench_msg_.force.y = msg->force.y;
  impl_->wrench_msg_.force.z = msg->force.z;
  impl_->wrench_msg_.torque.x = msg->torque.x;
  impl_->wrench_msg_.torque.y = msg->torque.y;
  impl_->wrench_msg_.torque.z = msg->torque.z;
}

void GazeboRosDynamixel::OnUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosDynamixel::OnUpdate");
  IGN_PROFILE_BEGIN("Apply angles");
#endif

    angle_desired = impl_->wrench_msg_.force.x;
    angle_0 = impl_->joints_[0]-> Position();
    angle_1 = angle_0 + 2 * (- std::signbit((angle_desired - angle_0)) + 0.5) * std::min(abs(T * dt * (angle_desired - angle_0)),rpm_max);
    impl_->joints_[0]->SetPosition(0,angle_1,1);
    
    /*angle_desired = impl_->wrench_msg_.force.x; 
    angle_desired_bit = std::round(angle_desired/rotation_range * bit);
    
    angle_1_bit = angle_0_bit + 2 * (- std::signbit((angle_desired_bit - angle_0_bit)) + 0.5) * std::min(abs(T * dt * (angle_desired_bit - angle_0_bit)),rpm_max_bit);
    angle_0_bit = angle_1_bit;
    impl_->joints_[0]->SetPosition(0,angle_1_bit/bit*rotation_range,1);
    */
    

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDynamixel)
}  // namespace gazebo_plugins
