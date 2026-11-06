#pragma once

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <chrono>
#include <string>

namespace custom
{
/// \brief Gazebo system plugin that simulates UWB ranges between tags and anchors.
/// \details Each tag-anchor link is evaluated with a link budget: an excess
/// loss L (dB) from free-space path loss plus every obstacle the line of
/// sight crosses, which sets the noise stddev and dropout probability. See
/// README.md, "Measurement model".
class UWBGazeboSystem : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate
{
public:
  /// \brief Reads the plugin SDF parameters, initializes publishers, and
  /// prepares the internal sensor-model state.
  /// \param[in] entity World entity that owns this system.
  /// \param[in] sdf SDF element containing the plugin configuration.
  /// \param[in,out] ecm Gazebo entity-component manager.
  /// \param[in,out] eventMgr Gazebo event manager.
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override;

  /// \brief Periodically computes anchor-tag ground-truth ranges and publishes
  /// simulated measurements with LOS/NLOS-dependent dropout and noise.
  /// \param[in] info Gazebo update timing information.
  /// \param[in,out] ecm Gazebo entity-component manager.
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;

  /// \brief Radio properties of an obstacle material.
  struct MaterialProperties
  {
    /// \brief Attenuation rate in dB per meter of chord length through it.
    double attenuationDbPerM;
    /// \brief Relative permittivity eps_r (dielectrics only), which sets the
    /// excess-delay bias l (sqrt(eps_r) - 1).
    double permittivity;
  };

private:
  /// \brief Free-space path loss relative to 1 m, n * 10 log10(d), in dB.
  double PathLossDb(double distanceM) const;

  /// \brief Obstacle loss and bias of a link, from the chord length through
  /// each material (untagged obstacles count as "generic").
  /// \details Loss is sum_m alpha_m l_m, plus metalDetourLossDb_ once if any
  /// metal is crossed. Bias is the dielectrics' excess delay,
  /// sum_m l_m (sqrt(eps_m) - 1) (only with enableDielectricDelay_).
  /// \param[in] materialLengthM Chord length through each material, in meters.
  /// \param[out] lossDb Obstacle loss, in dB.
  /// \param[out] biasCm Deterministic material bias, in centimeters.
  void ComputeObstacleEffects(const std::unordered_map<std::string, double> &materialLengthM,
                              double &lossDb,
                              double &biasCm) const;

  /// \brief Noise stddev for a total excess loss: sigma_min up to
  /// noise_start_loss_db, growing exponentially in dB to sigma_max at
  /// noise_end_loss_db.
  /// \return Noise standard deviation in centimeters.
  double ComputeNoiseStddevCm(double lossDb) const;

  /// \brief Dropout probability for a total excess loss: a floor of
  /// min_dropout_probability combined with a logistic link-failure curve
  /// centred on dropout_mid_loss_db, and 1 at max_link_loss_db or beyond.
  /// \return Dropout probability in the range [0, 1].
  double ComputeDropoutProbability(double lossDb) const;

  /// \brief Validates and normalizes user-provided plugin parameters.
  /// \details This method enforces valid ordering of thresholds, clamps values
  /// to physically meaningful ranges, and restores safe defaults when needed.
  void ValidateAndNormalizeParameters();

  gz::transport::Node node_;
  std::string topic_ = "/uwb_gz_simulator/distances";
  std::string groundTruthTopic_ = "/uwb_gz_simulator/distances_ground_truth";
  std::unordered_map<std::string, gz::transport::Node::Publisher> publishers_;
  std::unordered_map<std::string, gz::transport::Node::Publisher> groundTruthPublishers_;
  std::chrono::steady_clock::duration lastUpdateTime_{0};

  std::string tagPrefix_ = "uwb_tag";
  std::string anchorPrefix_ = "uwb_anchor";

  std::default_random_engine rng_;
  // 0 keeps the engine's fixed default seed (every run draws the same
  // noise and biases); any other value seeds it, e.g. one seed per
  // experiment repetition.
  unsigned int randomSeed_{0};

  std::unordered_map<std::string, double> pairBiasCm_;
  // Each tag's and anchor's position at the previous update.
  std::unordered_map<gz::sim::Entity, gz::math::Vector3d> lastSensorPosition_;
  std::uniform_real_distribution<double> biasCmDist_{-27.0, 20.0};

  // Configurable sensor model parameters (units: cm, unless noted).
  double noiseMeanCm_{0.0};
  double biasMinCm_{-27.0};
  double biasMaxCm_{20.0};
  bool applyPairBias_{false};

  // --- Link budget. Defaults reproduce the Eliko LOS calibration: with
  // n = 2, 23.5 dB is 15 m and 37.5 dB is 75 m of free space.
  double pathLossExponent_{2.0};
  double maxLinkLossDb_{37.5};
  double noiseStartLossDb_{23.5};
  double noiseEndLossDb_{37.5};
  double minNoiseStddevCm_{12.0};
  double maxNoiseStddevCm_{50.0};
  double minDropoutProbability_{0.02};
  double dropoutMidLossDb_{35.5};
  double dropoutWidthDb_{1.4};

  // If false, obstacles don't affect dropout (only the path loss does);
  // noise still uses the full link budget.
  bool enableNlosDropout_{true};
  // Links with a moving end only: each leg reads long with this probability, by an
  // exponential excess of this mean. 0 disables it.
  double movingOutlierProbability_{0.0};
  double movingOutlierMeanCm_{30.0};
  // Ignore intersections too close to endpoints to avoid self-hit artifacts.
  double nlosEndpointMarginM_{0.02};
  // Obstacle loss at or above which a link is NLOS (noise folded positive).
  double nlosMinLossDb_{3.0};
  // Cap on the loss from the vehicles' own bodies (base_link AABBs), which
  // use the generic attenuation rate.
  double bodyShadowMaxLossDb_{10.0};
  // Received-power ranging bias: a weaker first path is detected later, so
  // each link reads long by k (L - L0) cm. In TDOA only k is observable
  // (L0 cancels between the two legs). 0 disables it.
  double powerBiasCmPerDb_{0.0};
  double powerBiasRefLossDb_{0.0};

  // --- Ranging mode: TWR (default, one range per anchor-tag pair) or TDOA
  // (one differential measurement per anchor-pair-relative-to-tag, matching
  // real TDOA infrastructure like Bitcraze's LPS). A global switch, not
  // per-pair, since real deployments use one or the other for the whole
  // anchor infrastructure, never both.
  enum class UwbMode { Twr, Tdoa };
  UwbMode uwbMode_{UwbMode::Twr};
  // Topics used only in TDOA mode (kept separate from topic_/groundTruthTopic_
  // -- a TDOA value isn't a distance, and conflating them would mislead any
  // downstream consumer expecting the existing TWR topics' semantics).
  std::string tdoaTopic_ = "/uwb_gz_simulator/tdoa";
  std::string tdoaGroundTruthTopic_ = "/uwb_gz_simulator/tdoa_ground_truth";

  // --- Obstacle materials. Untagged obstacles are the "generic" material.
  // With enableMaterialAttenuation_, obstacles whose link or model name
  // carries a "__material_<name>" suffix (see extractMaterialName in the
  // .cpp) use that material's row instead.
  bool enableMaterialAttenuation_{false};
  // Carrier frequency at which the materials' attenuation rates are
  // evaluated (DW1000 channel 2 by default, as used by Eliko KIO and
  // Bitcraze LPS).
  double carrierFrequencyGhz_{4.0};
  // Loss added once to a link crossing any metal: the first path detours
  // around the conductor instead of going through it. It has no bias of its
  // own: on UTIL's NLOS data this loss's power bias and NLOS folding already
  // reproduce metal's shift.
  double metalDetourLossDb_{20.0};
  // Excess delay l (sqrt(eps_r) - 1) of propagating through dielectrics.
  // Off by default: correct for solid slabs, but UTIL's NLOS objects are
  // mostly air and it over-predicts their shifts (decision 2.7).
  bool enableDielectricDelay_{false};
  // Keyed by material name (lowercase, matching the "__material_<name>"
  // suffix), including "generic". Built in Configure() from the .cpp's
  // material table at carrierFrequencyGhz_, each entry overridable through
  // scalar "material_<name>_attenuation_db_per_m" / "material_<name>_permittivity"
  // SDF parameters. Obstacles tagged with a name not in here count as
  // generic (warned once per name).
  std::unordered_map<std::string, MaterialProperties> materials_;
  std::unordered_set<std::string> warnedUnknownMaterials_;
};
}
