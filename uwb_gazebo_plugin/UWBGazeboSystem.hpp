#pragma once

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
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
  std::normal_distribution<double> noise_dist_{0.0, 0.12};

  std::unordered_map<std::string, double> pairBiasCm_;
  std::uniform_real_distribution<double> biasCmDist_{-27.0, 20.0};

  std::bernoulli_distribution dropout_flag_{0.02};

  // Configurable sensor model parameters (units: cm, unless noted).
  double noiseMeanCm_{0.0};
  double noiseStddevCm_{0.12};
  double biasMinCm_{-27.0};
  double biasMaxCm_{20.0};
  double dropoutProbability_{0.02}; // [0, 1]
  bool applyPairBias_{false};

  // If true, any obstacle between tag and anchor causes measurement dropout.
  bool enableNlosDropout_{true};
  // Ignore intersections too close to endpoints to avoid self-hit artifacts.
  double nlosEndpointMarginM_{0.02};
};
}
