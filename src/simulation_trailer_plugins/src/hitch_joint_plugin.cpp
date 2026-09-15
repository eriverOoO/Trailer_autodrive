#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
class PriusHitchJointPlugin final : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->tractorModel = std::move(model);
    this->world = this->tractorModel->GetWorld();
    this->trailerModelName = ReadString(sdf, "trailer_model", "trailer_prius");
    this->parentLinkName = ReadString(sdf, "parent_link", "chassis");
    this->childLinkName = ReadString(sdf, "child_link", "chassis");
    this->parentAnchor = ReadVector(sdf, "parent_anchor", {0.0, 2.4, 0.45});
    this->childAnchor = ReadVector(sdf, "child_anchor", {0.0, -2.4, 0.45});
    this->axis = ReadVector(sdf, "axis", {0.0, 0.0, 1.0});
    this->lowerLimit = ReadDouble(sdf, "lower", -0.7853981634);
    this->upperLimit = ReadDouble(sdf, "upper", 0.7853981634);
    this->damping = ReadDouble(sdf, "damping", 10.0);
    this->rosNode = gazebo_ros::Node::Get(sdf);
    this->articulationPublisher = this->rosNode->create_publisher<std_msgs::msg::Float64>(
      "/trailer/articulation_angle", rclcpp::QoS(10));

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PriusHitchJointPlugin::OnUpdate, this));
    gzmsg << "[PriusHitch] Waiting for trailer model '" << this->trailerModelName
          << "' before creating the physical revolute joint.\n";
  }

private:
  static std::string ReadString(
    const sdf::ElementPtr & sdf, const std::string & name, const std::string & fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
  }

  static double ReadDouble(
    const sdf::ElementPtr & sdf, const std::string & name, const double fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
  }

  static ignition::math::Vector3d ReadVector(
    const sdf::ElementPtr & sdf, const std::string & name,
    const ignition::math::Vector3d & fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<ignition::math::Vector3d>(name) : fallback;
  }

  void OnUpdate()
  {
    if (this->hitchJoint)
    {
      this->PublishArticulation();
      return;
    }

    this->trailerModel = this->world->ModelByName(this->trailerModelName);
    if (!this->trailerModel)
      return;

    auto parentLink = this->tractorModel->GetLink(this->parentLinkName);
    auto childLink = this->trailerModel->GetLink(this->childLinkName);
    if (!parentLink || !childLink)
    {
      gzerr << "[PriusHitch] Cannot find chassis links: parent='"
            << this->parentLinkName << "', child='" << this->childLinkName << "'.\n";
      this->updateConnection.reset();
      return;
    }

    const auto parentAnchorWorld =
      parentLink->WorldPose().CoordPositionAdd(this->parentAnchor);
    const auto childAnchorWorld =
      childLink->WorldPose().CoordPositionAdd(this->childAnchor);
    const double initialError = parentAnchorWorld.Distance(childAnchorWorld);
    if (initialError > 0.05)
    {
      gzerr << "[PriusHitch] Refusing joint creation: initial anchor mismatch is "
            << initialError << " m (must be <= 0.05 m).\n";
      this->updateConnection.reset();
      return;
    }

    this->hitchJoint = this->world->Physics()->CreateJoint(
      "revolute", this->tractorModel);
    if (!this->hitchJoint)
    {
      gzerr << "[PriusHitch] Physics engine could not create a revolute joint.\n";
      this->updateConnection.reset();
      return;
    }

    this->hitchJoint->SetName("prius_hitch_joint");
    this->hitchJoint->Load(
      parentLink, childLink,
      ignition::math::Pose3d(this->childAnchor, ignition::math::Quaterniond::Identity));
    this->hitchJoint->Init();
    // ODE initializes its native hinge in Init(), so set the world anchor and
    // yaw axis afterwards; setting the axis before Init() is overwritten.
    this->hitchJoint->SetAnchor(0, parentAnchorWorld);
    this->hitchJoint->SetAxis(0, this->axis);
    this->hitchJoint->SetLowerLimit(0, this->lowerLimit);
    this->hitchJoint->SetUpperLimit(0, this->upperLimit);
    this->hitchJoint->SetDamping(0, this->damping);

    gzmsg << "[PriusHitch] Physical joint created: "
          << this->tractorModel->GetName() << "::" << this->parentLinkName
          << " -> " << this->trailerModel->GetName() << "::" << this->childLinkName
          << ", world anchor=" << parentAnchorWorld
          << ", requested axis=" << this->axis
          << ", global axis=" << this->hitchJoint->GlobalAxis(0)
          << ", limits=[" << this->lowerLimit
          << ", " << this->upperLimit << "], damping=" << this->damping << ".\n";
  }

  void PublishArticulation()
  {
    const auto simTime = this->world->SimTime();
    if ((simTime - this->lastPublishTime).Double() < 0.1)
      return;
    this->lastPublishTime = simTime;

    auto tractorLink = this->tractorModel->GetLink(this->parentLinkName);
    auto trailerLink = this->trailerModel->GetLink(this->childLinkName);
    if (!tractorLink || !trailerLink)
      return;
    const double raw =
      trailerLink->WorldPose().Rot().Yaw() - tractorLink->WorldPose().Rot().Yaw();
    std_msgs::msg::Float64 message;
    message.data = std::atan2(std::sin(raw), std::cos(raw));
    this->articulationPublisher->publish(message);
  }

  physics::WorldPtr world;
  physics::ModelPtr tractorModel;
  physics::ModelPtr trailerModel;
  physics::JointPtr hitchJoint;
  event::ConnectionPtr updateConnection;
  gazebo_ros::Node::SharedPtr rosNode;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr articulationPublisher;
  common::Time lastPublishTime;
  std::string trailerModelName;
  std::string parentLinkName;
  std::string childLinkName;
  ignition::math::Vector3d parentAnchor;
  ignition::math::Vector3d childAnchor;
  ignition::math::Vector3d axis;
  double lowerLimit{-0.7853981634};
  double upperLimit{0.7853981634};
  double damping{10.0};
};

GZ_REGISTER_MODEL_PLUGIN(PriusHitchJointPlugin)
}  // namespace gazebo
