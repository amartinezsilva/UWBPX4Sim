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

  /// \brief Electromagnetic properties of a tagged obstacle material.
  struct MaterialProperties
  {
    /// \brief Beer-Lambert penetration length lambda, in meters: the chord
    /// length through this material at which its severity reaches 1 - 1/e.
    double penetrationM;
    /// \brief Relative permittivity eps_r (dielectrics only), which sets the
    /// excess-delay bias l (sqrt(eps_r) - 1).
    double permittivity;
  };

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

  /// \brief Computes the severity and bias contributed by tagged materials.
  /// \details Severity is Beer-Lambert, s_mat = 1 - exp(-sum_k l_k/lambda_k).
  /// Bias is the dielectrics' excess delay, sum_k l_k (sqrt(eps_k) - 1), plus
  /// metalDetourBiasCm_ once if any metal is crossed.
  /// \param[in] materialLengthM Chord length through each material, in meters.
  /// \param[out] severity Material severity s_mat in [0, 1).
  /// \param[out] biasCm Deterministic material bias, in centimeters.
  void ComputeMaterialEffects(const std::unordered_map<std::string, double> &materialLengthM,
                              double &severity,
                              double &biasCm) const;

  /// \brief Whether a link is NLOS: untagged thickness above
  /// los_max_thickness_m, or material severity above material_los_max_severity.
  bool IsNlos(double untaggedThicknessM, double materialSeverity) const;

  /// \brief Combined obstacle severity of an NLOS link, before distance.
  /// \details min(1, g_tau s_tau(tau)) (+) min(1, g_mat s_mat'), with
  /// g = 1 + extra weight, s_mat' the material severity re-zeroed at the
  /// material LOS threshold, and (+) the probabilistic-OR combination.
  double ObstacleSeverity(double untaggedThicknessM,
                          double materialSeverity,
                          double thicknessExtraWeight,
                          double materialExtraWeight) const;

  /// \brief Computes the measurement-dropout probability for a link.
  /// \details In LOS, dropout depends only on distance. In NLOS, it combines
  /// the distance severity with ObstacleSeverity, with complete blackout
  /// enforced beyond the configured limits.
  /// \param[in] distanceM Anchor-tag separation in meters.
  /// \param[in] untaggedThicknessM Untagged walls + body-shadow thickness, in meters.
  /// \param[in] materialSeverity Material severity s_mat (0 if none).
  /// \return Dropout probability in the range [0, 1].
  double ComputeDropoutProbability(double distanceM,
                                   double untaggedThicknessM,
                                   double materialSeverity) const;

  /// \brief Computes the standard deviation of the additive range noise.
  /// \details In LOS, the standard deviation grows with distance only. In
  /// NLOS, it combines the distance severity with ObstacleSeverity.
  /// \param[in] distanceM Anchor-tag separation in meters.
  /// \param[in] untaggedThicknessM Untagged walls + body-shadow thickness, in meters.
  /// \param[in] materialSeverity Material severity s_mat (0 if none).
  /// \return Noise standard deviation in centimeters.
  double ComputeNoiseStddevCm(double distanceM,
                              double untaggedThicknessM,
                              double materialSeverity) const;

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
  // normalized extra-weight parameters add to the fixed baseline gain of 1.0
  // applied to distance.
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

  // --- Material-aware NLOS model (opt-in, off by default so existing
  // worlds/layouts that don't name obstacles by material keep their exact
  // behaviour). When enabled, obstacles whose link or model name carries a
  // "__material_<name>" suffix (see extractMaterialName in the .cpp) leave
  // the generic thickness curve and enter a Beer-Lambert material term
  // instead, weighted above thickness, plus a deterministic material bias.
  // Untagged obstacles keep the generic thickness curve either way. See
  // README.md, "Measurement model".
  bool enableMaterialAttenuation_{false};
  // Material severity at or below which a link with only tagged obstacles
  // still counts as LOS (the material analogue of losMaxThicknessM_).
  double materialLosMaxSeverity_{0.05};
  // Extra weight of the material term over the baseline gain of 1.0; kept
  // >= the matching thickness extra weight (material > thickness > distance).
  double nlosDropoutMaterialExtraWeight_{1.0};
  double nlosStddevMaterialExtraWeight_{1.0};
  // Bias added once to a link crossing any metal: the first path detours
  // around the conductor instead of going through it.
  double metalDetourBiasCm_{35.0};
  // Keyed by material name (lowercase, matching the "__material_<name>"
  // suffix). Starts from the .cpp's defaultMaterials() in Configure(), each
  // entry overridable through scalar "material_<name>_penetration_m" /
  // "material_<name>_permittivity" SDF parameters. Obstacles tagged with a
  // name not in here are treated as untagged (warned once per name).
  std::unordered_map<std::string, MaterialProperties> materials_;
  std::unordered_set<std::string> warnedUnknownMaterials_;
};
}
