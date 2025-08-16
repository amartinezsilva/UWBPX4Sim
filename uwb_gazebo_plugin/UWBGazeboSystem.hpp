#pragma once

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>  // Add this
#include <unordered_map>
#include <random>
#include <chrono>

namespace custom
{
class UWBGazeboSystem : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;

private:

  gz::transport::Node node_;
  std::string topic_ = "uwb_gz_simulator/distances";
  std::unordered_map<std::string, gz::transport::Node::Publisher> publishers_;
  std::chrono::steady_clock::duration lastUpdateTime_{0};

  std::string tagPrefix_ = "uwb_tag";
  std::string anchorPrefix_ = "uwb_anchor";
  std::string modelTagPrefix_ = "x500";
  std::string modelAnchorPrefix_ = "r1_rover";

  std::default_random_engine rng_;
  std::normal_distribution<double> noise_dist_{0.0, 10.0};  // zero-mean gaussian noise 10cm stdev
  // Randomness (you already have rng_ and noise_dist_)
  std::bernoulli_distribution outlier_flag_{0.02}; // 2% outliers
  std::exponential_distribution<double> outlier_bias_m_{1.0};
  // Probability of a measurement dropout (no publish)
  std::bernoulli_distribution dropout_flag_{0.005}; // 0.5% chance to skip a measurement
};
}
