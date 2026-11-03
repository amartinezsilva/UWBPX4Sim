#pragma once

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <unordered_map>
#include <random>
#include <chrono>
#include <string>

namespace custom
{
/// \brief Gazebo system plugin that simulates UWB ranges between UAV tags and UGV anchors.
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

private:
  /// \brief Maps effective blocked thickness to a normalized NLOS severity.
  /// \details Returns a value in the range [0, 1]. LOS conditions map to 0 and
  /// blackout maps to 1. Between those limits, severity is piecewise linear in
  /// effective blocked thickness, and the configured hard-NLOS ratio controls
  /// how much steeper the Hard-NLOS segment is relative to the Soft-NLOS
  /// segment while keeping the curve continuous.
  /// \param[in] blockedThicknessM Effective blocked thickness in meters.
  /// \return Normalized thickness severity.
  double ThicknessSeverity(double blockedThicknessM) const;

  /// \brief Converts raw robot-body blockage into an equivalent thickness term.
  /// \details Body-shadow contribution is ignored below the LOS thickness
  /// threshold and capped so that, by itself, it cannot exceed Soft NLOS.
  /// \param[in] bodyBlockedThicknessM Raw blocked thickness due to robot bodies,
  /// in meters.
  /// \return Equivalent blocked thickness used by the NLOS model.
  double ComputeBodyShadowEquivalentThicknessM(double bodyBlockedThicknessM) const;

  /// \brief Computes the measurement-dropout probability for a pair.
  /// \details In LOS, dropout depends only on distance. In NLOS, dropout is
  /// computed from the distance severity baseline plus a configurable weighted
  /// thickness severity term, with complete blackout enforced beyond the
  /// configured limits.
  /// \param[in] distanceM Anchor-tag separation in meters.
  /// \param[in] blockedThicknessM Effective blocked thickness in meters.
  /// \return Dropout probability in the range [0, 1].
  double ComputeDropoutProbability(double distanceM, double blockedThicknessM) const;

  /// \brief Computes the standard deviation of the additive range noise.
  /// \details In LOS, the standard deviation grows with distance only. In NLOS,
  /// it is computed from the distance severity baseline plus a configurable
  /// weighted thickness severity term.
  /// \param[in] distanceM Anchor-tag separation in meters.
  /// \param[in] blockedThicknessM Effective blocked thickness in meters.
  /// \return Noise standard deviation in centimeters.
  double ComputeNoiseStddevCm(double distanceM, double blockedThicknessM) const;

  /// \brief Looks up the attenuation multiplier for a material name.
  /// \details Falls back to materialAttenuationDefault_ for names not
  /// present in materialAttenuation_ (including when material attenuation
  /// is disabled, which leaves the map empty by construction).
  /// \param[in] materialName Lowercase material keyword, or empty.
  /// \return Attenuation multiplier applied to that obstacle's raw
  /// AABB-intersection length before the thickness-severity curve.
  double MaterialAttenuationFor(const std::string &materialName) const;

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

  std::unordered_map<std::string, double> pairBiasCm_;
  std::uniform_real_distribution<double> biasCmDist_{-27.0, 20.0};

  // Configurable sensor model parameters (units: cm, unless noted).
  double noiseMeanCm_{0.0};
  double biasMinCm_{-27.0};
  double biasMaxCm_{20.0};
  bool applyPairBias_{false};

  double losDropoutStartDistanceM_{10.0};
  double losDropoutEndDistanceM_{70.0};
  double losHardDropoutDistanceM_{75.0};

  double losMaxThicknessM_{0.10};
  double softNlosMaxThicknessM_{0.50};
  double blackoutThicknessM_{1.0};
  double hardNlosGapRatio_{0.50};

  double minDropoutProbability_{0.02};
  double maxDropoutProbability_{0.90};

  double minNoiseStddevCm_{12.0};
  double maxNoiseStddevCm_{50.0};

  double losStddevStartDistanceM_{15.0};
  double losStddevEndDistanceM_{75.0};

  // If true, NLOS-aware models are enabled when blocked thickness is non-zero.
  bool enableNlosDropout_{true};
  // Ignore intersections too close to endpoints to avoid self-hit artifacts.
  double nlosEndpointMarginM_{0.02};
  // In NLOS, thickness contributes at least as much as distance. These
  // normalized extra-weight parameters add to the fixed baseline gain of 1.0.
  double nlosDropoutThicknessExtraWeight_{0.50};
  double nlosStddevThicknessExtraWeight_{0.50};

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

  // --- Material-aware NLOS attenuation (opt-in, off by default so existing
  // worlds/layouts that don't name obstacles by material keep their current
  // behaviour byte-for-byte). When enabled, each obstacle link's own
  // AABB-intersection length is scaled by its material's attenuation
  // multiplier (relative to the generic/unscaled thickness this plugin
  // already used, i.e. 1.0) before entering the existing thickness-severity
  // curve -- see ThicknessSeverity, ComputeNoiseStddevCm,
  // ComputeDropoutProbability, which are unchanged.
  bool enableMaterialAttenuation_{false};
  // Applied to any obstacle whose name doesn't carry a recognized
  // "__material_<name>" suffix (see ExtractMaterialName in the .cpp) --
  // 1.0 reproduces the old undifferentiated behaviour for untagged obstacles.
  double materialAttenuationDefault_{1.0};
  // Keyed by material name (lowercase, matching the "__material_<name>"
  // suffix). Populated in Configure() from fixed "material_attenuation_
  // <name>" SDF parameters -- not a free-form map, so there's no custom SDF
  // parsing involved, just more scalar Get<double> calls like every other
  // parameter here. Unrecognized names fall back to materialAttenuationDefault_.
  std::unordered_map<std::string, double> materialAttenuation_;
};
}
