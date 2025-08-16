#include "UWBGazeboSystem.hpp"

#include <gz/msgs/double.pb.h>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>

using namespace custom;

GZ_ADD_PLUGIN(UWBGazeboSystem,
              gz::sim::System,
              UWBGazeboSystem::ISystemConfigure,
              UWBGazeboSystem::ISystemPreUpdate)

void UWBGazeboSystem::Configure(const gz::sim::Entity &,
                                const std::shared_ptr<const sdf::Element> &,
                                gz::sim::EntityComponentManager &,
                                gz::sim::EventManager &)
{
//   this->publisher_ = this->node_.Advertise<gz::msgs::Double_V>(this->topic_);
}

gz::math::Pose3d computeWorldPose(const gz::sim::Entity &entity,
                                  gz::sim::EntityComponentManager &ecm)
{
  gz::math::Pose3d pose(gz::math::Vector3d::Zero, gz::math::Quaterniond::Identity);
  gz::sim::Entity current = entity;

  while (current != gz::sim::kNullEntity)
  {
    auto poseComp = ecm.Component<gz::sim::components::Pose>(current);
    if (poseComp)
      pose = poseComp->Data() * pose;

    current = ecm.ParentEntity(current);
  }

  return pose;
}

void UWBGazeboSystem::PreUpdate(const gz::sim::UpdateInfo &info,
                                gz::sim::EntityComponentManager &ecm)
{

	const auto now = info.simTime;
	const auto dt = now - this->lastUpdateTime_;

	if (dt < std::chrono::milliseconds(100)) return;

	this->lastUpdateTime_ = now;

	std::vector<std::pair<std::string, gz::sim::Entity>> tags;
	std::vector<std::pair<std::string, gz::sim::Entity>> anchors;

	// --- Step 1 & 2: Find all tag and anchor links ---
	ecm.Each<gz::sim::components::Name, gz::sim::components::Link>(
	[&](const gz::sim::Entity &linkEntity,
	const gz::sim::components::Name *linkName,
	const gz::sim::components::Link *) -> bool
	{
		// Find the parent model
		auto parentEntity = ecm.ParentEntity(linkEntity);
		auto parentNameComp = ecm.Component<gz::sim::components::Name>(parentEntity);

		if (!parentNameComp)
		return true;

		const std::string &modelName = parentNameComp->Data();
		const std::string &linkBaseName = linkName->Data();

		// Check for uwb_tag_* in x500_* models
		if (modelName.rfind(modelTagPrefix_, 0) == 0 && linkBaseName.rfind(tagPrefix_, 0) == 0)
		tags.emplace_back(linkBaseName, linkEntity);

		// Check for uwb_anchor_* in r1_rover_* models
		if (modelName.rfind(modelAnchorPrefix_, 0) == 0 && linkBaseName.rfind(anchorPrefix_, 0) == 0)
		anchors.emplace_back(linkBaseName, linkEntity);

		return true;
	});

	// --- Step 3: Compute distance between each anchor-tag pair ---
	for (const auto &[tagName, tagEntity] : tags)
	{
		auto tagPose = computeWorldPose(tagEntity, ecm);

		for (const auto &[anchorName, anchorEntity] : anchors)
		{

			//With a small probability, skip publishing
			if (dropout_flag_(rng_)) {
				continue;
			}

			auto anchorPose = computeWorldPose(anchorEntity, ecm);

			double dist = tagPose.Pos().Distance(anchorPose.Pos()) * 100.0; // Convert to cm
			dist += noise_dist_(rng_);  // Add zero-mean Gaussian noise

			// With small probability, inject an outlier with exp. bias
			if (outlier_flag_(rng_)) {
				double bias_m = outlier_bias_m_(rng_);
				dist += bias_m * 100.0;
			}

			// Keep distance physically valid
			if (dist < 0.0) dist = 0.0;

			// Extract numeric IDs from names (assumes uwb_tag_1, uwb_anchor_2, etc.)
			std::string tagId = tagName.substr(tagName.find_last_of('_') + 1);
			std::string anchorId = anchorName.substr(anchorName.find_last_of('_') + 1);
			std::string topic = "/uwb_gz_simulator/distances/a" + anchorId + "t" + tagId;

			// Create publisher if not already existing
			if (this->publishers_.find(topic) == this->publishers_.end())
			{
				this->publishers_[topic] = this->node_.Advertise<gz::msgs::Double>(topic);
			}

			gz::msgs::Double msg;
			msg.set_data(dist);
			this->publishers_[topic].Publish(msg);
		}
	}
}


