#include "UWBGazeboSystem.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/AxisAlignedBox.hh>
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
#include <sdf/Element.hh>

using namespace custom;

GZ_ADD_PLUGIN(UWBGazeboSystem,
              gz::sim::System,
              UWBGazeboSystem::ISystemConfigure,
              UWBGazeboSystem::ISystemPreUpdate)

// Helper functions            
namespace
{
constexpr double kPositiveEpsilon = 1e-6;
constexpr double kOrderingEpsilon = 1e-6;

  //
  double exponentialRamp(const double _x,
                        const double _xStart,
                        const double _xEnd,
                        const double _yStart,
                        const double _yEnd)
  {
    if (_x <= _xStart)
      return _yStart;

    if (_x >= _xEnd)
      return _yEnd;

    const double growthRate = std::log(_yEnd / _yStart) / (_xEnd - _xStart);
    const double value = _yStart * std::exp(growthRate * (_x - _xStart));
    return std::clamp(value, std::min(_yStart, _yEnd), std::max(_yStart, _yEnd));
  }

  // Normalized linear mapping between a maximum and a minimum value, returns a value in range [0,1]
  double normalizedRamp(const double _x,
                        const double _xStart,
                        const double _xEnd)
  {
    if (_x <= _xStart)
      return 0.0;

    if (_x >= _xEnd)
      return 1.0;

    return (_x - _xStart) / (_xEnd - _xStart);
  }

  // Probabilistic-OR combination of two severities in [0,1]:
  // a (+) b = a + (1 - a) b = 1 - (1 - a)(1 - b). It only fills the headroom
  // the other term leaves (no early saturation), but it's symmetric -- it
  // gives neither input priority. Priority comes from the gains applied to
  // each term before combining (see UWBGazeboSystem::ObstacleSeverity).
  double orCombine(const double _a, const double _b)
  {
    const double a = std::clamp(_a, 0.0, 1.0);
    const double b = std::clamp(_b, 0.0, 1.0);
    return a + ((1.0 - a) * b);
  }

  // The one conductor in the material table: it isn't penetrated like a
  // dielectric (no permittivity, no excess-delay bias); the first path
  // detours around it instead (UWBGazeboSystem::metalDetourBiasCm_).
  const std::string kConductorMaterial = "metal";

  // Default material table: penetration length lambda (m) and relative
  // permittivity eps_r. eps_r are typical few-GHz values from the
  // literature; lambda values are chosen to reproduce the qualitative
  // ordering of the UTIL dataset's NLOS material tests (cardboard/foam/
  // wood ~ LOS, metal severe). See README.md, "Measurement model".
  std::unordered_map<std::string, UWBGazeboSystem::MaterialProperties> defaultMaterials()
  {
    return {
        {"metal", {0.0005, 1.0}},
        {"concrete", {0.5, 5.0}},
        {"glass", {0.5, 6.0}},
        {"wood", {2.0, 2.0}},
        {"plastic", {1.0, 2.5}},
        {"cardboard", {10.0, 1.2}},
        {"foam", {20.0, 1.05}},
    };
  }

  std::string knownMaterialList(
      const std::unordered_map<std::string, UWBGazeboSystem::MaterialProperties> &_materials)
  {
    std::vector<std::string> names;
    for (const auto &[name, props] : _materials)
      names.push_back(name);
    std::sort(names.begin(), names.end());

    std::string list;
    for (const auto &name : names)
      list += (list.empty() ? "" : ", ") + name;
    return list;
  }

  // What a tag-anchor segment crosses: total chord length through untagged
  // obstacles, and per tagged material.
  struct BlockedPath
  {
    double untaggedM{0.0};
    std::unordered_map<std::string, double> materialLengthM;
  };

  // Channel state of one tag-anchor link (see PreUpdate's evaluateLink).
  struct LinkState
  {
    double distanceM{0.0};
    double wallThicknessM{0.0};        // untagged wall chord length (raw)
    double bodyShadowThicknessM{0.0};  // robot-body chord length (raw)
    double materialLengthM{0.0};       // total tagged-material chord length
    double untaggedThicknessM{0.0};    // tau: untagged walls + body-shadow equivalent
    double materialSeverity{0.0};      // s_mat = 1 - exp(-sum l_k / lambda_k)
    double materialBiasCm{0.0};        // beta: excess delay + metal detour
    bool isNlos{false};
    double dropoutProbability{0.0};
    double noiseStddevCm{0.0};
  };

  // Create a topic name based on tag and anchor id pairId (.e.g. a1t2 for anchor 1 - tag2), and a common namespace
  std::string makePairTopic(const std::string &_baseTopic, const std::string &_pairId)
  {
    if (_baseTopic.empty())
      return "/" + _pairId;

    if (_baseTopic.back() == '/')
      return _baseTopic + _pairId;

    return _baseTopic + "/" + _pairId;
  }

  bool extractSensorId(const std::string &_linkName,
                       const std::string &_prefix,
                       std::string &_sensorId)
  {
    const std::string prefixWithSeparator = _prefix + "_";
    if (_linkName.rfind(prefixWithSeparator, 0) != 0)
      return false;

    _sensorId = _linkName.substr(prefixWithSeparator.size());
    if (_sensorId.empty())
      return false;

    return std::all_of(
        _sensorId.begin(),
        _sensorId.end(),
        [](const unsigned char c) { return std::isdigit(c) != 0; });
  }

  // Obstacle material tagging follows the same naming-convention style as
  // extractSensorId above (e.g. "uwb_tag_1") rather than a custom SDF
  // element: an obstacle's link OR model name carries a "__material_<name>"
  // suffix, e.g. "wall_1__material_metal". Checked against both names so
  // either the model or an individual link/collision within it can carry
  // the tag, whichever is more convenient for a given world's authoring.
  // Returns empty (no match) rather than a default, so the caller can fall
  // back to materialAttenuationDefault_ uniformly for both "no tag at all"
  // and "tag present but not in the configured table".
  std::string extractMaterialName(const std::string &_name)
  {
    static const std::string kMarker = "__material_";
    const auto pos = _name.rfind(kMarker);
    if (pos == std::string::npos)
      return "";

    std::string material = _name.substr(pos + kMarker.size());
    std::transform(material.begin(), material.end(), material.begin(),
                   [](const unsigned char c) { return std::tolower(c); });
    return material;
  }
}  // namespace

double UWBGazeboSystem::ThicknessSeverity(const double _blockedThicknessM) const
{
  if (_blockedThicknessM <= this->losMaxThicknessM_)
    return 0.0;

  if (_blockedThicknessM >= this->blackoutThicknessM_)
    return 1.0;

  const double softLength = this->softNlosMaxThicknessM_ - this->losMaxThicknessM_;
  const double hardLength = this->blackoutThicknessM_ - this->softNlosMaxThicknessM_;
  const double hardSlopeMultiplier = 1.0 + this->hardNlosGapRatio_;
  const double softSlope = 1.0 / (softLength + (hardSlopeMultiplier * hardLength));
  const double hardSlope = hardSlopeMultiplier * softSlope;

  if (_blockedThicknessM < this->softNlosMaxThicknessM_)
    return (_blockedThicknessM - this->losMaxThicknessM_) * softSlope;

  const double softEndSeverity = softLength * softSlope;
  return softEndSeverity +
         ((_blockedThicknessM - this->softNlosMaxThicknessM_) * hardSlope);
}

double UWBGazeboSystem::ComputeBodyShadowEquivalentThicknessM(
    const double _bodyBlockedThicknessM) const
{
  if (_bodyBlockedThicknessM <= this->losMaxThicknessM_)
    return 0.0;

  return std::min(_bodyBlockedThicknessM, this->softNlosMaxThicknessM_);
}

void UWBGazeboSystem::ComputeMaterialEffects(
    const std::unordered_map<std::string, double> &_materialLengthM,
    double &_severity,
    double &_biasCm) const
{
  // Beer-Lambert: A = sum_k l_k / lambda_k, s_mat = 1 - exp(-A). The
  // material sets lambda over orders of magnitude (mm for metal, tens of m
  // for foam), the chord length only enters through l/lambda -- which is
  // what makes material more influential than thickness.
  double attenuation = 0.0;
  bool crossesConductor = false;
  _biasCm = 0.0;

  for (const auto &[materialName, lengthM] : _materialLengthM)
  {
    const auto it = this->materials_.find(materialName);
    if (it == this->materials_.end() || lengthM <= 0.0)
      continue;

    attenuation += lengthM / it->second.penetrationM;

    if (materialName == kConductorMaterial)
    {
      crossesConductor = true;
      continue;
    }

    // Excess delay of propagating through a dielectric at c/sqrt(eps_r)
    // instead of c: beta = l (sqrt(eps_r) - 1).
    _biasCm += lengthM * (std::sqrt(it->second.permittivity) - 1.0) * 100.0;
  }

  // Applied once per link, however many metal obstacles it crosses: the
  // first path detours around the whole obstruction.
  if (crossesConductor)
    _biasCm += this->metalDetourBiasCm_;

  _severity = 1.0 - std::exp(-attenuation);
}

bool UWBGazeboSystem::IsNlos(const double _untaggedThicknessM,
                             const double _materialSeverity) const
{
  return (_untaggedThicknessM > this->losMaxThicknessM_) ||
         (_materialSeverity > this->materialLosMaxSeverity_);
}

double UWBGazeboSystem::ObstacleSeverity(const double _untaggedThicknessM,
                                         const double _materialSeverity,
                                         const double _thicknessExtraWeight,
                                         const double _materialExtraWeight) const
{
  // Untagged obstacles: generic thickness curve, gain g_tau = 1 + w_tau.
  const double thicknessTerm = std::min(
      1.0, (1.0 + _thicknessExtraWeight) * this->ThicknessSeverity(_untaggedThicknessM));

  // Tagged materials: Beer-Lambert severity, re-zeroed at the material LOS
  // threshold (so the term is continuous where a link turns NLOS), gain
  // g_mat = 1 + w_mat >= g_tau.
  double materialTerm = 0.0;
  if (_materialSeverity > this->materialLosMaxSeverity_)
  {
    const double rezeroed = (_materialSeverity - this->materialLosMaxSeverity_) /
                            (1.0 - this->materialLosMaxSeverity_);
    materialTerm = std::min(1.0, (1.0 + _materialExtraWeight) * rezeroed);
  }

  return orCombine(thicknessTerm, materialTerm);
}

double UWBGazeboSystem::ComputeDropoutProbability(const double _distanceM,
                                                  const double _untaggedThicknessM,
                                                  const double _materialSeverity) const
{
  if (_distanceM > this->losHardDropoutDistanceM_)
    return 1.0;

  if (this->enableNlosDropout_ && this->IsNlos(_untaggedThicknessM, _materialSeverity))
  {
    if (_untaggedThicknessM >= this->blackoutThicknessM_)
      return 1.0;

    const double distanceSeverity = normalizedRamp(_distanceM,
                                                   this->losDropoutStartDistanceM_,
                                                   this->losDropoutEndDistanceM_);
    const double combinedSeverity = orCombine(
        distanceSeverity,
        this->ObstacleSeverity(_untaggedThicknessM,
                               _materialSeverity,
                               this->nlosDropoutThicknessExtraWeight_,
                               this->nlosDropoutMaterialExtraWeight_));

    return exponentialRamp(combinedSeverity,
                           0.0,
                           1.0,
                           this->minDropoutProbability_,
                           this->maxDropoutProbability_);
  }

  return exponentialRamp(_distanceM,
                         this->losDropoutStartDistanceM_,
                         this->losDropoutEndDistanceM_,
                         this->minDropoutProbability_,
                         this->maxDropoutProbability_);
}

double UWBGazeboSystem::ComputeNoiseStddevCm(const double _distanceM,
                                             const double _untaggedThicknessM,
                                             const double _materialSeverity) const
{
  const double distanceSeverity = normalizedRamp(_distanceM,
                                                 this->losStddevStartDistanceM_,
                                                 this->losStddevEndDistanceM_);

  if (this->IsNlos(_untaggedThicknessM, _materialSeverity))
  {
    if (_untaggedThicknessM >= this->blackoutThicknessM_)
      return this->maxNoiseStddevCm_;

    const double combinedSeverity = orCombine(
        distanceSeverity,
        this->ObstacleSeverity(_untaggedThicknessM,
                               _materialSeverity,
                               this->nlosStddevThicknessExtraWeight_,
                               this->nlosStddevMaterialExtraWeight_));

    return exponentialRamp(combinedSeverity,
                           0.0,
                           1.0,
                           this->minNoiseStddevCm_,
                           this->maxNoiseStddevCm_);
  }

  return exponentialRamp(distanceSeverity,
                         0.0,
                         1.0,
                         this->minNoiseStddevCm_,
                         this->maxNoiseStddevCm_);
}

void UWBGazeboSystem::ValidateAndNormalizeParameters()
{
  if (this->biasMinCm_ > this->biasMaxCm_)
  {
    gzwarn << "[UWBGazeboSystem] bias_min_cm > bias_max_cm. Swapping values.\n";
    std::swap(this->biasMinCm_, this->biasMaxCm_);
  }

  if (this->nlosEndpointMarginM_ < 0.0)
  {
    gzwarn << "[UWBGazeboSystem] nlos_endpoint_margin_m < 0.0. Clamping to 0.0.\n";
    this->nlosEndpointMarginM_ = 0.0;
  }

  auto clampPositive = [](const double value) {
    return std::max(value, kPositiveEpsilon);
  };

  this->minDropoutProbability_ = std::clamp(this->minDropoutProbability_,
                                            kPositiveEpsilon,
                                            1.0);
  this->maxDropoutProbability_ = std::clamp(this->maxDropoutProbability_,
                                            kPositiveEpsilon,
                                            1.0);
  if (this->minDropoutProbability_ > this->maxDropoutProbability_)
  {
    gzwarn << "[UWBGazeboSystem] min_dropout_probability > max_dropout_probability. Swapping values.\n";
    std::swap(this->minDropoutProbability_, this->maxDropoutProbability_);
  }

  this->minNoiseStddevCm_ = clampPositive(this->minNoiseStddevCm_);
  this->maxNoiseStddevCm_ = clampPositive(this->maxNoiseStddevCm_);
  if (this->minNoiseStddevCm_ > this->maxNoiseStddevCm_)
  {
    gzwarn << "[UWBGazeboSystem] min_noise_stddev_cm > max_noise_stddev_cm. Swapping values.\n";
    std::swap(this->minNoiseStddevCm_, this->maxNoiseStddevCm_);
  }

  if (this->losDropoutEndDistanceM_ < this->losDropoutStartDistanceM_)
  {
    gzwarn << "[UWBGazeboSystem] los_dropout_end_distance_m < los_dropout_start_distance_m. Swapping values.\n";
    std::swap(this->losDropoutStartDistanceM_, this->losDropoutEndDistanceM_);
  }

  if (this->losStddevEndDistanceM_ < this->losStddevStartDistanceM_)
  {
    gzwarn << "[UWBGazeboSystem] los_stddev_end_distance_m < los_stddev_start_distance_m. Swapping values.\n";
    std::swap(this->losStddevStartDistanceM_, this->losStddevEndDistanceM_);
  }

  if (this->losHardDropoutDistanceM_ < this->losDropoutEndDistanceM_)
  {
    gzwarn << "[UWBGazeboSystem] los_hard_dropout_distance_m < los_dropout_end_distance_m. Raising to los_dropout_end_distance_m.\n";
    this->losHardDropoutDistanceM_ = this->losDropoutEndDistanceM_;
  }

  this->losMaxThicknessM_ = std::max(this->losMaxThicknessM_, 0.0);
  this->softNlosMaxThicknessM_ =
      std::max(this->softNlosMaxThicknessM_, this->losMaxThicknessM_ + kOrderingEpsilon);
  this->blackoutThicknessM_ =
      std::max(this->blackoutThicknessM_, this->softNlosMaxThicknessM_ + kOrderingEpsilon);

  const bool hardGapWasOutOfRange =
      (this->hardNlosGapRatio_ < 0.0) || (this->hardNlosGapRatio_ > 1.0);
  this->hardNlosGapRatio_ = std::clamp(this->hardNlosGapRatio_, 0.0, 1.0);
  if (hardGapWasOutOfRange)
  {
    gzwarn << "[UWBGazeboSystem] hard_nlos_gap_ratio out of [0,1]. Clamped to "
           << this->hardNlosGapRatio_ << ".\n";
  }

  const bool dropoutThicknessExtraWasOutOfRange =
      (this->nlosDropoutThicknessExtraWeight_ < 0.0) ||
      (this->nlosDropoutThicknessExtraWeight_ > 1.0);
  this->nlosDropoutThicknessExtraWeight_ =
      std::clamp(this->nlosDropoutThicknessExtraWeight_, 0.0, 1.0);
  if (dropoutThicknessExtraWasOutOfRange)
  {
    gzwarn << "[UWBGazeboSystem] nlos_dropout_thickness_extra_weight out of [0,1]. Clamped to "
           << this->nlosDropoutThicknessExtraWeight_ << ".\n";
  }

  const bool stddevThicknessExtraWasOutOfRange =
      (this->nlosStddevThicknessExtraWeight_ < 0.0) ||
      (this->nlosStddevThicknessExtraWeight_ > 1.0);
  this->nlosStddevThicknessExtraWeight_ =
      std::clamp(this->nlosStddevThicknessExtraWeight_, 0.0, 1.0);
  if (stddevThicknessExtraWasOutOfRange)
  {
    gzwarn << "[UWBGazeboSystem] nlos_stddev_thickness_extra_weight out of [0,1]. Clamped to "
           << this->nlosStddevThicknessExtraWeight_ << ".\n";
  }

  if (this->topic_.empty())
  {
    gzwarn << "[UWBGazeboSystem] topic is empty. Falling back to /uwb_gz_simulator/distances.\n";
    this->topic_ = "/uwb_gz_simulator/distances";
  }

  if (this->groundTruthTopic_.empty())
  {
    gzwarn << "[UWBGazeboSystem] ground_truth_topic is empty. Falling back to "
              "/uwb_gz_simulator/distances_ground_truth.\n";
    this->groundTruthTopic_ = "/uwb_gz_simulator/distances_ground_truth";
  }

  if (this->tdoaTopic_.empty())
  {
    gzwarn << "[UWBGazeboSystem] tdoa_topic is empty. Falling back to /uwb_gz_simulator/tdoa.\n";
    this->tdoaTopic_ = "/uwb_gz_simulator/tdoa";
  }

  if (this->tdoaGroundTruthTopic_.empty())
  {
    gzwarn << "[UWBGazeboSystem] tdoa_ground_truth_topic is empty. Falling back to "
              "/uwb_gz_simulator/tdoa_ground_truth.\n";
    this->tdoaGroundTruthTopic_ = "/uwb_gz_simulator/tdoa_ground_truth";
  }

  const bool materialLosWasOutOfRange =
      (this->materialLosMaxSeverity_ < 0.0) || (this->materialLosMaxSeverity_ > 0.99);
  this->materialLosMaxSeverity_ = std::clamp(this->materialLosMaxSeverity_, 0.0, 0.99);
  if (materialLosWasOutOfRange)
  {
    gzwarn << "[UWBGazeboSystem] material_los_max_severity out of [0,0.99]. Clamped to "
           << this->materialLosMaxSeverity_ << ".\n";
  }

  // Material extra weights live in [0,2] (gain 1..3), and must be at least
  // the matching thickness weight: material is meant to be the most
  // influential NLOS factor, then thickness, then distance.
  auto normalizeMaterialWeight = [](double &_weight,
                                    const double _thicknessWeight,
                                    const char *_name,
                                    const char *_thicknessName)
  {
    if (_weight < 0.0 || _weight > 2.0)
    {
      _weight = std::clamp(_weight, 0.0, 2.0);
      gzwarn << "[UWBGazeboSystem] " << _name << " out of [0,2]. Clamped to " << _weight << ".\n";
    }
    if (_weight < _thicknessWeight)
    {
      gzwarn << "[UWBGazeboSystem] " << _name << " < " << _thicknessName
             << " (material must weigh at least as much as thickness). Raising to "
             << _thicknessWeight << ".\n";
      _weight = _thicknessWeight;
    }
  };
  normalizeMaterialWeight(this->nlosDropoutMaterialExtraWeight_,
                          this->nlosDropoutThicknessExtraWeight_,
                          "nlos_dropout_material_extra_weight",
                          "nlos_dropout_thickness_extra_weight");
  normalizeMaterialWeight(this->nlosStddevMaterialExtraWeight_,
                          this->nlosStddevThicknessExtraWeight_,
                          "nlos_stddev_material_extra_weight",
                          "nlos_stddev_thickness_extra_weight");

  if (this->metalDetourBiasCm_ < 0.0)
  {
    gzwarn << "[UWBGazeboSystem] material_metal_detour_bias_cm < 0.0. Clamping to 0.0.\n";
    this->metalDetourBiasCm_ = 0.0;
  }

  for (auto &[materialName, props] : this->materials_)
  {
    if (props.penetrationM < kPositiveEpsilon)
    {
      gzwarn << "[UWBGazeboSystem] material_" << materialName
             << "_penetration_m must be > 0. Clamping to " << kPositiveEpsilon << ".\n";
      props.penetrationM = kPositiveEpsilon;
    }
    if (props.permittivity < 1.0)
    {
      gzwarn << "[UWBGazeboSystem] material_" << materialName
             << "_permittivity < 1.0 (faster than light). Clamping to 1.0.\n";
      props.permittivity = 1.0;
    }
  }
}

void UWBGazeboSystem::Configure(const gz::sim::Entity &,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &,
                                gz::sim::EventManager &)
{
  if (_sdf && _sdf->HasElement("gaussian_noise_mean_cm"))
    this->noiseMeanCm_ = _sdf->Get<double>("gaussian_noise_mean_cm");

  if (_sdf && _sdf->HasElement("bias_min_cm"))
    this->biasMinCm_ = _sdf->Get<double>("bias_min_cm");

  if (_sdf && _sdf->HasElement("bias_max_cm"))
    this->biasMaxCm_ = _sdf->Get<double>("bias_max_cm");

  if (_sdf && _sdf->HasElement("apply_pair_bias"))
    this->applyPairBias_ = _sdf->Get<bool>("apply_pair_bias");

  if (_sdf && _sdf->HasElement("enable_nlos_dropout"))
    this->enableNlosDropout_ = _sdf->Get<bool>("enable_nlos_dropout");

  if (_sdf && _sdf->HasElement("nlos_endpoint_margin_m"))
    this->nlosEndpointMarginM_ = _sdf->Get<double>("nlos_endpoint_margin_m");

  if (_sdf && _sdf->HasElement("nlos_dropout_thickness_extra_weight"))
  {
    this->nlosDropoutThicknessExtraWeight_ =
        _sdf->Get<double>("nlos_dropout_thickness_extra_weight");
  }

  if (_sdf && _sdf->HasElement("nlos_stddev_thickness_extra_weight"))
  {
    this->nlosStddevThicknessExtraWeight_ =
        _sdf->Get<double>("nlos_stddev_thickness_extra_weight");
  }

  if (_sdf && _sdf->HasElement("los_dropout_start_distance_m"))
    this->losDropoutStartDistanceM_ = _sdf->Get<double>("los_dropout_start_distance_m");

  if (_sdf && _sdf->HasElement("los_dropout_end_distance_m"))
    this->losDropoutEndDistanceM_ = _sdf->Get<double>("los_dropout_end_distance_m");

  if (_sdf && _sdf->HasElement("los_hard_dropout_distance_m"))
    this->losHardDropoutDistanceM_ = _sdf->Get<double>("los_hard_dropout_distance_m");

  if (_sdf && _sdf->HasElement("los_max_thickness_m"))
    this->losMaxThicknessM_ = _sdf->Get<double>("los_max_thickness_m");

  if (_sdf && _sdf->HasElement("soft_nlos_max_thickness_m"))
    this->softNlosMaxThicknessM_ = _sdf->Get<double>("soft_nlos_max_thickness_m");

  if (_sdf && _sdf->HasElement("blackout_thickness_m"))
    this->blackoutThicknessM_ = _sdf->Get<double>("blackout_thickness_m");

  if (_sdf && _sdf->HasElement("hard_nlos_gap_ratio"))
    this->hardNlosGapRatio_ = _sdf->Get<double>("hard_nlos_gap_ratio");

  if (_sdf && _sdf->HasElement("min_dropout_probability"))
    this->minDropoutProbability_ = _sdf->Get<double>("min_dropout_probability");

  if (_sdf && _sdf->HasElement("max_dropout_probability"))
    this->maxDropoutProbability_ = _sdf->Get<double>("max_dropout_probability");

  if (_sdf && _sdf->HasElement("min_noise_stddev_cm"))
    this->minNoiseStddevCm_ = _sdf->Get<double>("min_noise_stddev_cm");

  if (_sdf && _sdf->HasElement("max_noise_stddev_cm"))
    this->maxNoiseStddevCm_ = _sdf->Get<double>("max_noise_stddev_cm");

  if (_sdf && _sdf->HasElement("los_stddev_start_distance_m"))
    this->losStddevStartDistanceM_ = _sdf->Get<double>("los_stddev_start_distance_m");

  if (_sdf && _sdf->HasElement("los_stddev_end_distance_m"))
    this->losStddevEndDistanceM_ = _sdf->Get<double>("los_stddev_end_distance_m");

  if (_sdf && _sdf->HasElement("topic"))
    this->topic_ = _sdf->Get<std::string>("topic");

  if (_sdf && _sdf->HasElement("ground_truth_topic"))
    this->groundTruthTopic_ = _sdf->Get<std::string>("ground_truth_topic");

  if (_sdf && _sdf->HasElement("uwb_mode"))
  {
    std::string modeStr = _sdf->Get<std::string>("uwb_mode");
    std::transform(modeStr.begin(), modeStr.end(), modeStr.begin(),
                   [](const unsigned char c) { return std::tolower(c); });
    if (modeStr == "tdoa")
      this->uwbMode_ = UwbMode::Tdoa;
    else if (modeStr == "twr")
      this->uwbMode_ = UwbMode::Twr;
    else
      gzwarn << "[UWBGazeboSystem] Unknown uwb_mode '" << modeStr
             << "' (expected 'twr' or 'tdoa'). Keeping 'twr'.\n";
  }

  if (_sdf && _sdf->HasElement("tdoa_topic"))
    this->tdoaTopic_ = _sdf->Get<std::string>("tdoa_topic");

  if (_sdf && _sdf->HasElement("tdoa_ground_truth_topic"))
    this->tdoaGroundTruthTopic_ = _sdf->Get<std::string>("tdoa_ground_truth_topic");

  if (_sdf && _sdf->HasElement("enable_material_attenuation"))
    this->enableMaterialAttenuation_ = _sdf->Get<bool>("enable_material_attenuation");

  if (_sdf && _sdf->HasElement("material_los_max_severity"))
    this->materialLosMaxSeverity_ = _sdf->Get<double>("material_los_max_severity");

  if (_sdf && _sdf->HasElement("nlos_dropout_material_extra_weight"))
  {
    this->nlosDropoutMaterialExtraWeight_ =
        _sdf->Get<double>("nlos_dropout_material_extra_weight");
  }

  if (_sdf && _sdf->HasElement("nlos_stddev_material_extra_weight"))
  {
    this->nlosStddevMaterialExtraWeight_ =
        _sdf->Get<double>("nlos_stddev_material_extra_weight");
  }

  if (_sdf && _sdf->HasElement("material_metal_detour_bias_cm"))
    this->metalDetourBiasCm_ = _sdf->Get<double>("material_metal_detour_bias_cm");

  // Fixed material set (defaultMaterials()), not a free-form table: each
  // entry is overridable through scalar "material_<name>_penetration_m" /
  // "material_<name>_permittivity" parameters (no permittivity for the
  // conductor). Add a material to defaultMaterials() (and to gui/app.py's
  // PARAM_GROUPS and the presets) if a world needs one not listed.
  this->materials_ = defaultMaterials();
  this->warnedUnknownMaterials_.clear();
  for (auto &[materialName, props] : this->materials_)
  {
    const std::string prefix = "material_" + materialName;
    if (_sdf && _sdf->HasElement(prefix + "_penetration_m"))
      props.penetrationM = _sdf->Get<double>(prefix + "_penetration_m");
    if (materialName != kConductorMaterial && _sdf && _sdf->HasElement(prefix + "_permittivity"))
      props.permittivity = _sdf->Get<double>(prefix + "_permittivity");
  }

  // The previous material model's scalar multipliers no longer exist; say
  // so rather than silently ignoring an old preset.
  for (const char *oldName : {"default", "metal", "wood", "cardboard", "plastic", "foam"})
  {
    const std::string paramName = std::string("material_attenuation_") + oldName;
    if (_sdf && _sdf->HasElement(paramName))
    {
      gzwarn << "[UWBGazeboSystem] " << paramName << " is no longer used (replaced by "
                "material_<name>_penetration_m / _permittivity; see README.md, "
                "\"Measurement model\"). Ignoring it.\n";
    }
  }

  this->ValidateAndNormalizeParameters();

  this->biasCmDist_ =
      std::uniform_real_distribution<double>(this->biasMinCm_, this->biasMaxCm_);
  this->pairBiasCm_.clear();
  this->publishers_.clear();
  this->groundTruthPublishers_.clear();

  std::ostringstream materialSummary;
  if (this->enableMaterialAttenuation_)
  {
    materialSummary << ", material_los_max_severity=" << this->materialLosMaxSeverity_
                    << ", nlos_dropout_material_extra_weight=" << this->nlosDropoutMaterialExtraWeight_
                    << ", nlos_stddev_material_extra_weight=" << this->nlosStddevMaterialExtraWeight_
                    << ", material_metal_detour_bias_cm=" << this->metalDetourBiasCm_
                    << ", materials={";
    std::vector<std::string> names;
    for (const auto &[name, props] : this->materials_)
      names.push_back(name);
    std::sort(names.begin(), names.end());
    for (size_t i = 0; i < names.size(); ++i)
    {
      const auto &props = this->materials_.at(names[i]);
      materialSummary << (i ? ", " : "") << names[i] << ":{lambda_m=" << props.penetrationM;
      if (names[i] != kConductorMaterial)
        materialSummary << ", eps_r=" << props.permittivity;
      materialSummary << "}";
    }
    materialSummary << "}";
  }

  gzmsg << "[UWBGazeboSystem] Sensor model: gaussian_mean_cm=" << this->noiseMeanCm_
        << ", dropout_probability_range=[" << this->minDropoutProbability_ << ", "
        << this->maxDropoutProbability_ << "]"
        << ", dynamic_stddev_cm=[" << this->minNoiseStddevCm_ << ", "
        << this->maxNoiseStddevCm_ << "]"
        << ", los_dropout_range_m=[" << this->losDropoutStartDistanceM_ << ", "
        << this->losDropoutEndDistanceM_ << "]"
        << ", los_hard_dropout_distance_m=" << this->losHardDropoutDistanceM_
        << ", los_stddev_range_m=[" << this->losStddevStartDistanceM_ << ", "
        << this->losStddevEndDistanceM_ << "]"
        << ", thickness_regimes_m={los<=" << this->losMaxThicknessM_
        << ", soft<=" << this->softNlosMaxThicknessM_
        << ", hard<" << this->blackoutThicknessM_
        << ", blackout>=" << this->blackoutThicknessM_ << "}"
        << ", thickness_severity=piecewise_linear(thickness){hard_slope_multiplier="
        << (1.0 + this->hardNlosGapRatio_) << "}"
        << ", body_shadow_equivalent_thickness_m=min(body_blocked_thickness, soft_nlos_max_thickness_m)"
        << ", bias_min_cm=" << this->biasMinCm_
        << ", bias_max_cm=" << this->biasMaxCm_
        << ", apply_pair_bias=" << std::boolalpha << this->applyPairBias_
        << ", enable_nlos_dropout=" << std::boolalpha << this->enableNlosDropout_
        << ", nlos_endpoint_margin_m=" << this->nlosEndpointMarginM_
        << ", nlos_dropout_thickness_extra_weight="
        << this->nlosDropoutThicknessExtraWeight_
        << ", nlos_stddev_thickness_extra_weight="
        << this->nlosStddevThicknessExtraWeight_
        << ", topic=" << this->topic_
        << ", ground_truth_topic=" << this->groundTruthTopic_
        << ", uwb_mode=" << (this->uwbMode_ == UwbMode::Tdoa ? "tdoa" : "twr")
        << ", enable_material_attenuation=" << std::boolalpha << this->enableMaterialAttenuation_
        << materialSummary.str() << "\n";
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

bool segmentIntersectsAabb(const gz::math::Vector3d &_start,
                           const gz::math::Vector3d &_segment,
                           const gz::math::AxisAlignedBox &_box,
                           double &_tEnter,
                           double &_tExit)
{
  constexpr double kParallelEps = 1e-9;

  double tMin = 0.0;
  double tMax = 1.0;

  const auto &boxMin = _box.Min();
  const auto &boxMax = _box.Max();

  for (int axis = 0; axis < 3; ++axis)
  {
    const double startAxis = _start[axis];
    const double segmentAxis = _segment[axis];

    if (std::abs(segmentAxis) < kParallelEps)
    {
      if (startAxis < boxMin[axis] || startAxis > boxMax[axis])
        return false;
      continue;
    }

    const double invSegmentAxis = 1.0 / segmentAxis;
    double t1 = (boxMin[axis] - startAxis) * invSegmentAxis;
    double t2 = (boxMax[axis] - startAxis) * invSegmentAxis;
    if (t1 > t2)
      std::swap(t1, t2);

    tMin = std::max(tMin, t1);
    tMax = std::min(tMax, t2);

    if (tMax < tMin)
      return false;
  }

  _tEnter = tMin;
  _tExit = tMax;
  return true;
}

// Sums the (start,end) parametric intervals in _intervals after merging any
// that overlap, in units of the segment's own length (i.e. [0,1] fractions,
// same convention PreUpdate already used before this helper existed).
double mergeAndSumIntervals(std::vector<std::pair<double, double>> _intervals)
{
  if (_intervals.empty())
    return 0.0;

  std::sort(_intervals.begin(), _intervals.end());

  double total = 0.0;
  double currentStart = _intervals.front().first;
  double currentEnd = _intervals.front().second;

  for (size_t i = 1; i < _intervals.size(); ++i)
  {
    const auto &interval = _intervals[i];
    if (interval.first <= currentEnd)
    {
      currentEnd = std::max(currentEnd, interval.second);
      continue;
    }

    total += (currentEnd - currentStart);
    currentStart = interval.first;
    currentEnd = interval.second;
  }

  total += (currentEnd - currentStart);
  return total;
}

// Length of the tag-anchor segment that lies inside obstacle AABBs, split
// into untagged obstacles (fed to the generic thickness curve) and each
// tagged material (fed to the Beer-Lambert material model). _materials is
// optional (nullptr when the material model is disabled) and maps an
// obstacle link to its material name, resolved once per PreUpdate call;
// links missing from it count as untagged. Intervals are merged *within*
// each group before summing, so overlapping AABBs of the same group (e.g. a
// wall split into several collision links) aren't double-counted, while
// different materials in series each contribute their own length. With
// _materials null, every link is untagged, which reduces to exactly the
// original single-thickness behaviour.
BlockedPath computeAabbBlockedPath(const gz::math::Vector3d &_start,
                                   const gz::math::Vector3d &_end,
                                   const std::vector<gz::sim::Entity> &_allLinks,
                                   const gz::sim::Entity &_tagLinkEntity,
                                   const gz::sim::Entity &_anchorLinkEntity,
                                   const double _endpointMarginM,
                                   gz::sim::EntityComponentManager &_ecm,
                                   const std::unordered_map<gz::sim::Entity, std::string> *_materials = nullptr)
{
  BlockedPath path;
  const double segmentLength = _start.Distance(_end);
  if (segmentLength <= (2.0 * _endpointMarginM))
    return path;

  const gz::math::Vector3d segment = _end - _start;
  const double tMargin = _endpointMarginM / segmentLength;
  const double tMinAllowed = tMargin;
  const double tMaxAllowed = 1.0 - tMargin;
  if (tMaxAllowed <= tMinAllowed)
    return path;

  // Keyed by material name; "" holds the untagged obstacles.
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> intervalsByMaterial;

  for (const auto &linkEntity : _allLinks)
  {
    if (linkEntity == _tagLinkEntity || linkEntity == _anchorLinkEntity)
      continue;

    const gz::sim::Link link(linkEntity);
    const auto worldBox = link.WorldAxisAlignedBox(_ecm);
    if (!worldBox.has_value())
      continue;

    double tEnter = 0.0;
    double tExit = 0.0;
    if (!segmentIntersectsAabb(_start, segment, *worldBox, tEnter, tExit))
      continue;

    const double clippedEnter = std::max(tEnter, tMinAllowed);
    const double clippedExit = std::min(tExit, tMaxAllowed);

    if (clippedExit <= clippedEnter)
      continue;

    std::string materialName;
    if (_materials)
    {
      const auto it = _materials->find(linkEntity);
      if (it != _materials->end())
        materialName = it->second;
    }

    intervalsByMaterial[materialName].emplace_back(clippedEnter, clippedExit);
  }

  for (auto &[materialName, intervals] : intervalsByMaterial)
  {
    const double lengthM = mergeAndSumIntervals(std::move(intervals)) * segmentLength;
    if (lengthM <= 0.0)
      continue;

    if (materialName.empty())
      path.untaggedM = lengthM;
    else
      path.materialLengthM[materialName] = lengthM;
  }

  return path;
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
	std::vector<gz::sim::Entity> obstacleLinks;
	std::vector<gz::sim::Entity> bodyShadowLinks;
	std::unordered_map<std::string, std::string> seenTagIds;
	std::unordered_map<std::string, std::string> seenAnchorIds;

	// Vehicles are recognised by the UWB links they carry, not by model name,
	// so any vehicle type from models/vehicles.yaml is picked up.
	std::unordered_set<gz::sim::Entity> uwbModels;
	ecm.Each<gz::sim::components::Name, gz::sim::components::Link>(
	[&](const gz::sim::Entity &linkEntity,
	const gz::sim::components::Name *linkName,
	const gz::sim::components::Link *) -> bool
	{
		const std::string &linkBaseName = linkName->Data();
		if (linkBaseName.rfind(tagPrefix_, 0) == 0 || linkBaseName.rfind(anchorPrefix_, 0) == 0)
		{
			uwbModels.insert(ecm.ParentEntity(linkEntity));
		}
		return true;
	});

	// Find all tag and anchor links
	ecm.Each<gz::sim::components::Name, gz::sim::components::Link>(
	[&](const gz::sim::Entity &linkEntity,
	const gz::sim::components::Name *linkName,
	const gz::sim::components::Link *) -> bool
	{
		// Find the parent model
		auto parentEntity = ecm.ParentEntity(linkEntity);
		auto parentNameComp = ecm.Component<gz::sim::components::Name>(parentEntity);

		if (!parentNameComp) return true;

		const std::string &modelName = parentNameComp->Data();
		const std::string &linkBaseName = linkName->Data();

		// Check for uwb_tag_* links
		if (linkBaseName.rfind(tagPrefix_, 0) == 0)
		{
			std::string tagId;
			if (!extractSensorId(linkBaseName, tagPrefix_, tagId))
			{
				gzerr << "[UWBGazeboSystem] Invalid tag link name '" << linkBaseName
				      << "' in model '" << modelName << "'. Expected format '"
				      << tagPrefix_ << "_<global_id>'. Skipping.\n";
			}
			else
			{
				const std::string fullName = modelName + "::" + linkBaseName;
				auto [it, inserted] = seenTagIds.emplace(tagId, fullName);
				if (!inserted)
				{
					gzerr << "[UWBGazeboSystem] Duplicate global tag id t" << tagId
					      << " detected in '" << fullName << "' and '" << it->second
					      << "'. Tag ids must be globally unique across all UAVs. Skipping duplicate.\n";
				}
				else
				{
					tags.emplace_back(linkBaseName, linkEntity);
				}
			}
		}

		// Check for uwb_anchor_* links
		if (linkBaseName.rfind(anchorPrefix_, 0) == 0)
		{
			std::string anchorId;
			if (!extractSensorId(linkBaseName, anchorPrefix_, anchorId))
			{
				gzerr << "[UWBGazeboSystem] Invalid anchor link name '" << linkBaseName
				      << "' in model '" << modelName << "'. Expected format '"
				      << anchorPrefix_ << "_<global_id>'. Skipping.\n";
			}
			else
			{
				const std::string fullName = modelName + "::" + linkBaseName;
				auto [it, inserted] = seenAnchorIds.emplace(anchorId, fullName);
				if (!inserted)
				{
					gzerr << "[UWBGazeboSystem] Duplicate global anchor id a" << anchorId
					      << " detected in '" << fullName << "' and '" << it->second
					      << "'. Anchor ids must be globally unique across all UGVs. Skipping duplicate.\n";
				}
				else
				{
					anchors.emplace_back(linkBaseName, linkEntity);
				}
			}
		}

		// For wall-induced NLOS, consider only static map obstacles.
		const bool isUwbModel = uwbModels.count(parentEntity) > 0;
		const bool isGroundPlane = (modelName == "ground_plane");
		if (!isUwbModel && !isGroundPlane)
		{
			obstacleLinks.emplace_back(linkEntity);
		}

		// Add a lightweight body-shadow penalty from the main vehicle bodies only.
		if (isUwbModel && linkBaseName == "base_link")
		{
			bodyShadowLinks.emplace_back(linkEntity);
		}

		return true;
	});

	// Ensure world AABB is available for LOS/NLOS checks.
	for (const auto &linkEntity : obstacleLinks)
	{
		gz::sim::Link(linkEntity).EnableBoundingBoxChecks(ecm, true);
	}
	for (const auto &linkEntity : bodyShadowLinks)
	{
		gz::sim::Link(linkEntity).EnableBoundingBoxChecks(ecm, true);
	}

	// Resolve each obstacle's material once per update (not once per
	// anchor-tag/anchor-pair below) -- obstacles don't change material
	// mid-update. Only obstacleLinks (walls/world geometry) are
	// material-aware; robot bodyShadowLinks always use the generic thickness
	// curve, since a vehicle's own body isn't an NLOS "material" a world
	// author would tag. Obstacles left out of the map (untagged, or tagged
	// with a material not in materials_) use the generic thickness curve too.
	std::unordered_map<gz::sim::Entity, std::string> obstacleMaterials;
	if (this->enableMaterialAttenuation_)
	{
		for (const auto &linkEntity : obstacleLinks)
		{
			std::string materialName;
			const auto linkNameComp = ecm.Component<gz::sim::components::Name>(linkEntity);
			if (linkNameComp)
				materialName = extractMaterialName(linkNameComp->Data());

			if (materialName.empty())
			{
				const auto parentEntity = ecm.ParentEntity(linkEntity);
				const auto modelNameComp = ecm.Component<gz::sim::components::Name>(parentEntity);
				if (modelNameComp)
					materialName = extractMaterialName(modelNameComp->Data());
			}

			if (materialName.empty())
				continue;

			if (this->materials_.find(materialName) == this->materials_.end())
			{
				if (this->warnedUnknownMaterials_.insert(materialName).second)
				{
					gzwarn << "[UWBGazeboSystem] Unknown obstacle material '" << materialName
					       << "' (known: " << knownMaterialList(this->materials_)
					       << "). Treating it as an untagged obstacle.\n";
				}
				continue;
			}

			obstacleMaterials[linkEntity] = materialName;
		}
	}
	const std::unordered_map<gz::sim::Entity, std::string> *obstacleMaterialsPtr =
		this->enableMaterialAttenuation_ ? &obstacleMaterials : nullptr;

	// Geometry + channel state of one tag-anchor link: what the ray crosses
	// and the resulting LOS/NLOS regime, noise stddev and dropout
	// probability. Shared by TWR (one link per measurement) and TDOA (two
	// links per measurement). See README.md, "Measurement model".
	auto evaluateLink = [&](const gz::sim::Entity &_tagEntity,
	                        const gz::math::Vector3d &_tagPos,
	                        const gz::sim::Entity &_anchorEntity,
	                        const gz::math::Vector3d &_anchorPos)
	{
		LinkState link;
		link.distanceM = _tagPos.Distance(_anchorPos);

		const BlockedPath wallPath =
			computeAabbBlockedPath(_tagPos, _anchorPos, obstacleLinks, _tagEntity, _anchorEntity,
			                       this->nlosEndpointMarginM_, ecm, obstacleMaterialsPtr);
		const BlockedPath bodyPath =
			computeAabbBlockedPath(_tagPos, _anchorPos, bodyShadowLinks, _tagEntity, _anchorEntity,
			                       this->nlosEndpointMarginM_, ecm);

		link.wallThicknessM = wallPath.untaggedM;
		link.bodyShadowThicknessM = bodyPath.untaggedM;
		for (const auto &[materialName, lengthM] : wallPath.materialLengthM)
			link.materialLengthM += lengthM;

		link.untaggedThicknessM =
			wallPath.untaggedM + this->ComputeBodyShadowEquivalentThicknessM(bodyPath.untaggedM);
		this->ComputeMaterialEffects(wallPath.materialLengthM, link.materialSeverity, link.materialBiasCm);
		link.isNlos = this->IsNlos(link.untaggedThicknessM, link.materialSeverity);
		link.dropoutProbability = this->ComputeDropoutProbability(
			link.distanceM, link.untaggedThicknessM, link.materialSeverity);
		link.noiseStddevCm = this->ComputeNoiseStddevCm(
			link.distanceM, link.untaggedThicknessM, link.materialSeverity);
		return link;
	};

	// Random + deterministic range error of one link, in cm (excluding the
	// persistent per-pair/per-anchor bias, which differs between modes):
	// Gaussian noise, folded positive in NLOS (the first path arrives late,
	// never early), plus the materials' excess-delay bias.
	auto drawLinkErrorCm = [&](const LinkState &_link)
	{
		double noiseCm =
			std::normal_distribution<double>(this->noiseMeanCm_, _link.noiseStddevCm)(this->rng_);
		if (_link.isNlos && noiseCm < 0.0)
			noiseCm = -noiseCm;
		return noiseCm + _link.materialBiasCm;
	};

	// Persistent bias drawn once per key from U(bias_min_cm, bias_max_cm).
	auto persistentBiasCm = [&](const std::string &_key)
	{
		auto it = this->pairBiasCm_.find(_key);
		if (it == this->pairBiasCm_.end())
			it = this->pairBiasCm_.emplace(_key, this->biasCmDist_(this->rng_)).first;
		return it->second;
	};

	auto publishTo = [&](std::unordered_map<std::string, gz::transport::Node::Publisher> &_publishers,
	                     const std::string &_topic,
	                     const double _value)
	{
		if (_publishers.find(_topic) == _publishers.end())
			_publishers[_topic] = this->node_.Advertise<gz::msgs::Double>(_topic);

		gz::msgs::Double msg;
		msg.set_data(_value);
		_publishers[_topic].Publish(msg);
	};

	if (this->uwbMode_ == UwbMode::Twr)
	{
		// Compute distance between each anchor-tag pair
		for (const auto &[tagName, tagEntity] : tags)
		{
			const auto tagPose = computeWorldPose(tagEntity, ecm);
			const std::string tagId = tagName.substr(tagName.find_last_of('_') + 1);

			for (const auto &[anchorName, anchorEntity] : anchors)
			{
				const auto anchorPose = computeWorldPose(anchorEntity, ecm);
				const std::string anchorId = anchorName.substr(anchorName.find_last_of('_') + 1);
				const std::string pairId = "a" + anchorId + "t" + tagId;

				const LinkState link = evaluateLink(tagEntity, tagPose.Pos(), anchorEntity, anchorPose.Pos());

				if (link.wallThicknessM > 0.0 || link.bodyShadowThicknessM > 0.0 || link.materialLengthM > 0.0)
				{
					gzmsg << "[UWBGazeboSystem] wall_blocked_thickness_m=" << link.wallThicknessM
					      << ", body_shadow_blocked_thickness_m=" << link.bodyShadowThicknessM
					      << ", effective_blocked_thickness_m=" << link.untaggedThicknessM
					      << ", material_blocked_length_m=" << link.materialLengthM
					      << ", material_severity=" << link.materialSeverity
					      << ", material_bias_cm=" << link.materialBiasCm
					      << " pair=" << pairId << "\n";
				}

				const double groundTruthDistCm = link.distanceM * 100.0;
				publishTo(this->groundTruthPublishers_,
				          makePairTopic(this->groundTruthTopic_, pairId),
				          groundTruthDistCm);

				if (std::bernoulli_distribution(link.dropoutProbability)(this->rng_))
					continue;

				// r = d + n + beta + b_pair
				double dist = groundTruthDistCm + drawLinkErrorCm(link);
				if (this->applyPairBias_)
					dist += persistentBiasCm("a" + anchorName + "t" + tagName);

				// Keep distance physically valid
				if (dist < 0.0) dist = 0.0;

				publishTo(this->publishers_, makePairTopic(this->topic_, pairId), dist);
			}
		}
	}
	else  // UwbMode::Tdoa
	{
		if (anchors.size() < 2)
		{
			gzwarn << "[UWBGazeboSystem] uwb_mode=tdoa needs at least 2 anchors; found "
			       << anchors.size() << ". Skipping this update.\n";
			return;
		}

		// Reference ("master") anchor: the one with the smallest numeric id,
		// matching a centralized/TDOA2-style deployment (all other anchors'
		// arrival times get compared against this one). Not configurable yet
		// -- a fixed, deterministic choice is enough to produce realistic
		// TDOA statistics without simulating the clock-sync protocol. Ids are
		// all-digit (enforced by extractSensorId above), so stoul is safe.
		auto anchorNumericId = [](const std::string &_anchorName)
		{
			return std::stoul(_anchorName.substr(_anchorName.find_last_of('_') + 1));
		};
		const auto refIt = std::min_element(anchors.begin(), anchors.end(),
			[&](const auto &_lhs, const auto &_rhs)
			{
				return anchorNumericId(_lhs.first) < anchorNumericId(_rhs.first);
			});
		const auto &[refAnchorName, refAnchorEntity] = *refIt;
		const std::string refAnchorId = refAnchorName.substr(refAnchorName.find_last_of('_') + 1);

		for (const auto &[tagName, tagEntity] : tags)
		{
			const auto tagPose = computeWorldPose(tagEntity, ecm);
			const std::string tagId = tagName.substr(tagName.find_last_of('_') + 1);
			const auto refAnchorPose = computeWorldPose(refAnchorEntity, ecm);

			const LinkState refLink =
				evaluateLink(tagEntity, tagPose.Pos(), refAnchorEntity, refAnchorPose.Pos());

			// One reference-anchor arrival per tag per update, shared by every
			// TDOA formed against it: if it's lost, all of them are lost, and
			// its error enters all of them with the same value (and sign).
			const bool refDropped =
				std::bernoulli_distribution(refLink.dropoutProbability)(this->rng_);
			double refErrorCm = 0.0;
			if (!refDropped)
			{
				refErrorCm = drawLinkErrorCm(refLink);
				if (this->applyPairBias_)
					refErrorCm += persistentBiasCm("tdoa_a" + refAnchorId);
			}

			for (const auto &[anchorName, anchorEntity] : anchors)
			{
				if (anchorEntity == refAnchorEntity)
					continue;

				const auto anchorPose = computeWorldPose(anchorEntity, ecm);
				const std::string anchorId = anchorName.substr(anchorName.find_last_of('_') + 1);
				// 3-component pairId (vs. TWR's 2-component "a<id>t<id>")
				// clearly marks this as a differential measurement between
				// two anchors, not a single-anchor range.
				const std::string pairId = "a" + refAnchorId + "a" + anchorId + "t" + tagId;

				const LinkState link = evaluateLink(tagEntity, tagPose.Pos(), anchorEntity, anchorPose.Pos());

				// Ground truth per the UTIL dataset paper (Zhao et al. 2023,
				// Eq. 1): d_ij = ||p-a_j|| - ||p-a_i||, with anchor i the
				// reference.
				const double groundTruthTdoaCm = (link.distanceM - refLink.distanceM) * 100.0;
				publishTo(this->groundTruthPublishers_,
				          makePairTopic(this->tdoaGroundTruthTopic_, pairId),
				          groundTruthTdoaCm);

				if (refDropped || std::bernoulli_distribution(link.dropoutProbability)(this->rng_))
					continue;

				// d_ij = d_ij_true + e_j - e_i, each leg's error e = n + beta + b_anchor.
				// The per-anchor bias stands in for antenna delay; the tag's own
				// delay is common to both legs and cancels in the difference.
				double legErrorCm = drawLinkErrorCm(link);
				if (this->applyPairBias_)
					legErrorCm += persistentBiasCm("tdoa_a" + anchorId);

				publishTo(this->publishers_,
				          makePairTopic(this->tdoaTopic_, pairId),
				          groundTruthTdoaCm + legErrorCm - refErrorCm);
			}
		}
	}
}
