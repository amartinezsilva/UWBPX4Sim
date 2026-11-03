#include "UWBGazeboSystem.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
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

  // Combine the LOS distance baseline with the NLOS thickness effect while
  // preserving the remaining headroom. This avoids early saturation and keeps
  // more contrast between LOS / Soft NLOS / Hard NLOS.
  double combineNlosSeverity(const double _distanceSeverity,
                             const double _thicknessSeverity,
                             const double _thicknessExtraWeight)
  {
    const double distanceSeverity = std::clamp(_distanceSeverity, 0.0, 1.0);
    const double weightedThicknessSeverity = std::clamp(
        (1.0 + _thicknessExtraWeight) * _thicknessSeverity,
        0.0,
        1.0);

    return distanceSeverity +
           ((1.0 - distanceSeverity) * weightedThicknessSeverity);
  }

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

double UWBGazeboSystem::ComputeDropoutProbability(const double _distanceM,
                                                  const double _blockedThicknessM) const
{
  const double losDropoutProbability = exponentialRamp(_distanceM,
                                                       this->losDropoutStartDistanceM_,
                                                       this->losDropoutEndDistanceM_,
                                                       this->minDropoutProbability_,
                                                       this->maxDropoutProbability_);

  if (_distanceM > this->losHardDropoutDistanceM_)
    return 1.0;

  if (this->enableNlosDropout_ && _blockedThicknessM > this->losMaxThicknessM_)
  {
    if (_blockedThicknessM >= this->blackoutThicknessM_)
      return 1.0;

    const double distanceSeverity = normalizedRamp(_distanceM,
                                                   this->losDropoutStartDistanceM_,
                                                   this->losDropoutEndDistanceM_);
    const double nlosThicknessSeverity = this->ThicknessSeverity(_blockedThicknessM);
    const double combinedSeverity = combineNlosSeverity(
        distanceSeverity,
        nlosThicknessSeverity,
        this->nlosDropoutThicknessExtraWeight_);

    return exponentialRamp(combinedSeverity,
                           0.0,
                           1.0,
                           this->minDropoutProbability_,
                           this->maxDropoutProbability_);
  }

  return losDropoutProbability;
}

double UWBGazeboSystem::ComputeNoiseStddevCm(const double _distanceM,
                                             const double _blockedThicknessM) const
{
  const double distanceSeverity = normalizedRamp(_distanceM,
                                                 this->losStddevStartDistanceM_,
                                                 this->losStddevEndDistanceM_);

  if (_blockedThicknessM > this->losMaxThicknessM_)
  {
    if (_blockedThicknessM >= this->blackoutThicknessM_)
      return this->maxNoiseStddevCm_;

    const double nlosThicknessSeverity = this->ThicknessSeverity(_blockedThicknessM);
    const double combinedSeverity = combineNlosSeverity(
        distanceSeverity,
        nlosThicknessSeverity,
        this->nlosStddevThicknessExtraWeight_);

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

double UWBGazeboSystem::MaterialAttenuationFor(const std::string &_materialName) const
{
  if (_materialName.empty())
    return this->materialAttenuationDefault_;

  const auto it = this->materialAttenuation_.find(_materialName);
  if (it == this->materialAttenuation_.end())
    return this->materialAttenuationDefault_;

  return it->second;
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

  if (this->materialAttenuationDefault_ < 0.0)
  {
    gzwarn << "[UWBGazeboSystem] material_attenuation_default < 0.0. Clamping to 0.0.\n";
    this->materialAttenuationDefault_ = 0.0;
  }

  for (auto &[materialName, multiplier] : this->materialAttenuation_)
  {
    if (multiplier < 0.0)
    {
      gzwarn << "[UWBGazeboSystem] material_attenuation_" << materialName
             << " < 0.0. Clamping to 0.0.\n";
      multiplier = 0.0;
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

  if (_sdf && _sdf->HasElement("material_attenuation_default"))
    this->materialAttenuationDefault_ = _sdf->Get<double>("material_attenuation_default");

  // Fixed material set (matching the UTIL dataset's NLOS obstacle types),
  // not a free-form table -- see the materialAttenuation_ comment in the
  // header for why. Add more "material_attenuation_<name>" entries here
  // (and in params.yaml) if a world needs a material not listed.
  this->materialAttenuation_.clear();
  for (const char *materialName : {"metal", "wood", "cardboard", "plastic", "foam"})
  {
    const std::string paramName = std::string("material_attenuation_") + materialName;
    if (_sdf && _sdf->HasElement(paramName))
      this->materialAttenuation_[materialName] = _sdf->Get<double>(paramName);
  }

  this->ValidateAndNormalizeParameters();

  this->biasCmDist_ =
      std::uniform_real_distribution<double>(this->biasMinCm_, this->biasMaxCm_);
  this->pairBiasCm_.clear();
  this->publishers_.clear();
  this->groundTruthPublishers_.clear();

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
        << "\n";
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

// _materialMultipliers: optional (nullptr when material attenuation is
// disabled) per-link attenuation multiplier, resolved once per PreUpdate
// call from each obstacle's name (see ExtractMaterialName/
// MaterialAttenuationFor) rather than looked up here. Intervals are grouped
// by multiplier and merged *within* each group before summing across
// groups, so overlapping AABBs from the same material (e.g. a wall split
// into several collision links) still aren't double-counted, while
// different materials in series each still contribute their own share.
// With _materialMultipliers null, every link implicitly shares one group
// with multiplier 1.0, which reduces to exactly the prior (pre-material)
// behaviour.
double computeAabbBlockedThicknessM(const gz::math::Vector3d &_start,
                                    const gz::math::Vector3d &_end,
                                    const std::vector<gz::sim::Entity> &_allLinks,
                                    const gz::sim::Entity &_tagLinkEntity,
                                    const gz::sim::Entity &_anchorLinkEntity,
                                    const double _endpointMarginM,
                                    gz::sim::EntityComponentManager &_ecm,
                                    const std::unordered_map<gz::sim::Entity, double> *_materialMultipliers = nullptr)
{
  const double segmentLength = _start.Distance(_end);
  if (segmentLength <= (2.0 * _endpointMarginM))
    return 0.0;

  const gz::math::Vector3d segment = _end - _start;
  const double tMargin = _endpointMarginM / segmentLength;
  const double tMinAllowed = tMargin;
  const double tMaxAllowed = 1.0 - tMargin;
  if (tMaxAllowed <= tMinAllowed)
    return 0.0;

  std::unordered_map<double, std::vector<std::pair<double, double>>> intervalsByMultiplier;

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

    double multiplier = 1.0;
    if (_materialMultipliers)
    {
      const auto it = _materialMultipliers->find(linkEntity);
      if (it != _materialMultipliers->end())
        multiplier = it->second;
    }

    intervalsByMultiplier[multiplier].emplace_back(clippedEnter, clippedExit);
  }

  if (intervalsByMultiplier.empty())
    return 0.0;

  double blockedFraction = 0.0;
  for (auto &[multiplier, intervals] : intervalsByMultiplier)
  {
    blockedFraction += multiplier * mergeAndSumIntervals(std::move(intervals));
  }

  if (blockedFraction <= 0.0)
    return 0.0;

  return blockedFraction * segmentLength;
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

	// Resolve each obstacle's material multiplier once per update (not once
	// per anchor-tag/anchor-pair below) -- obstacles don't move relative to
	// the world between pairs, so their material never changes mid-update.
	// Only obstacleLinks (walls/world geometry) are material-aware; robot
	// bodyShadowLinks keep their existing unscaled behaviour, since a
	// vehicle's own body isn't an NLOS "material" a world author would tag.
	std::unordered_map<gz::sim::Entity, double> obstacleMaterialMultipliers;
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

			obstacleMaterialMultipliers[linkEntity] = this->MaterialAttenuationFor(materialName);
		}
	}
	const std::unordered_map<gz::sim::Entity, double> *obstacleMaterialMultipliersPtr =
		this->enableMaterialAttenuation_ ? &obstacleMaterialMultipliers : nullptr;

	if (this->uwbMode_ == UwbMode::Twr)
	{
	// Compute distance between each anchor-tag pair
	for (const auto &[tagName, tagEntity] : tags)
	{
		auto tagPose = computeWorldPose(tagEntity, ecm);

    // Compute per-pair dropout from LOS distance or NLOS blocked thickness.
		for (const auto &[anchorName, anchorEntity] : anchors)
		{
			auto anchorPose = computeWorldPose(anchorEntity, ecm);
			std::string tagId = tagName.substr(tagName.find_last_of('_') + 1);
			std::string anchorId = anchorName.substr(anchorName.find_last_of('_') + 1);
			std::string pairId = "a" + anchorId + "t" + tagId;

			const double wallBlockedThicknessM =
				computeAabbBlockedThicknessM(tagPose.Pos(),
											 anchorPose.Pos(),
											 obstacleLinks,
											 tagEntity,
											 anchorEntity,
											 this->nlosEndpointMarginM_,
											 ecm,
											 obstacleMaterialMultipliersPtr);
			const double bodyShadowBlockedThicknessM =
				computeAabbBlockedThicknessM(tagPose.Pos(),
											 anchorPose.Pos(),
											 bodyShadowLinks,
											 tagEntity,
											 anchorEntity,
											 this->nlosEndpointMarginM_,
											 ecm);
			const double effectiveBlockedThicknessM =
				wallBlockedThicknessM +
				this->ComputeBodyShadowEquivalentThicknessM(bodyShadowBlockedThicknessM);

			const double distanceM = tagPose.Pos().Distance(anchorPose.Pos());
			const double groundTruthDistCm = distanceM * 100.0;
			const double dropoutProbability =
				this->ComputeDropoutProbability(distanceM, effectiveBlockedThicknessM);
			const double noiseStddevCm =
				this->ComputeNoiseStddevCm(distanceM, effectiveBlockedThicknessM);

			if (wallBlockedThicknessM > 0.0 || bodyShadowBlockedThicknessM > 0.0)
			{
				gzmsg << "[UWBGazeboSystem] wall_blocked_thickness_m=" << wallBlockedThicknessM
				      << ", body_shadow_blocked_thickness_m=" << bodyShadowBlockedThicknessM
				      << ", effective_blocked_thickness_m=" << effectiveBlockedThicknessM
				      << " pair=" << pairId << "\n";
			}

			const std::string groundTruthTopic = makePairTopic(this->groundTruthTopic_, pairId);
			if (this->groundTruthPublishers_.find(groundTruthTopic) == this->groundTruthPublishers_.end())
			{
				this->groundTruthPublishers_[groundTruthTopic] =
					this->node_.Advertise<gz::msgs::Double>(groundTruthTopic);
			}

			gz::msgs::Double groundTruthMsg;
			groundTruthMsg.set_data(groundTruthDistCm);
			this->groundTruthPublishers_[groundTruthTopic].Publish(groundTruthMsg);

			if (std::bernoulli_distribution(dropoutProbability)(this->rng_))
			{
				continue;
			}

			const bool isNlos = (effectiveBlockedThicknessM > this->losMaxThicknessM_);
			double noiseCm =
				std::normal_distribution<double>(this->noiseMeanCm_, noiseStddevCm)(this->rng_);
			if (isNlos && noiseCm < 0.0)
			{
				noiseCm = -noiseCm;
			}

			double dist = groundTruthDistCm;
			dist += noiseCm;

			// Build stable key per anchor–tag pair
			const std::string key = "a" + anchorName + "t" + tagName;

			if (this->applyPairBias_)
			{
				// Draw and cache a persistent bias for this pair (in cm)
				auto it = this->pairBiasCm_.find(key);
				if (it == this->pairBiasCm_.end())
				{
					it = this->pairBiasCm_.emplace(key, this->biasCmDist_(this->rng_)).first;
				}
				dist += it->second;
			}

			// Keep distance physically valid
			if (dist < 0.0) dist = 0.0;

			const std::string topic = makePairTopic(this->topic_, pairId);

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
	}  // uwbMode_ == UwbMode::Twr
	else  // UwbMode::Tdoa
	{
		if (anchors.size() < 2)
		{
			gzwarn << "[UWBGazeboSystem] uwb_mode=tdoa needs at least 2 anchors; found "
			       << anchors.size() << ". Skipping this update.\n";
		}
		else
		{
			// Reference ("master") anchor: the one with the smallest numeric id,
			// matching a believable centralized/TDOA2-style deployment (all
			// other anchors' arrival times get compared against this one). Not
			// configurable yet -- a fixed, deterministic choice is enough to
			// produce realistic TDOA statistics without simulating the full
			// clock-sync protocol (see the uwb_mode comment in the header).
			auto refIt = std::min_element(anchors.begin(), anchors.end(),
				[](const auto &lhs, const auto &rhs)
				{
					const std::string lhsId = lhs.first.substr(lhs.first.find_last_of('_') + 1);
					const std::string rhsId = rhs.first.substr(rhs.first.find_last_of('_') + 1);
					return std::stoi(lhsId) < std::stoi(rhsId);
				});
			const auto &[refAnchorName, refAnchorEntity] = *refIt;
			const std::string refAnchorId = refAnchorName.substr(refAnchorName.find_last_of('_') + 1);

			for (const auto &[tagName, tagEntity] : tags)
			{
				auto tagPose = computeWorldPose(tagEntity, ecm);
				const std::string tagId = tagName.substr(tagName.find_last_of('_') + 1);
				auto refAnchorPose = computeWorldPose(refAnchorEntity, ecm);
				const double refDistanceM = tagPose.Pos().Distance(refAnchorPose.Pos());

				const double refWallBlockedThicknessM =
					computeAabbBlockedThicknessM(tagPose.Pos(), refAnchorPose.Pos(), obstacleLinks,
					                             tagEntity, refAnchorEntity, this->nlosEndpointMarginM_,
					                             ecm, obstacleMaterialMultipliersPtr);
				const double refBodyShadowBlockedThicknessM =
					computeAabbBlockedThicknessM(tagPose.Pos(), refAnchorPose.Pos(), bodyShadowLinks,
					                             tagEntity, refAnchorEntity, this->nlosEndpointMarginM_, ecm);
				const double refEffectiveBlockedThicknessM =
					refWallBlockedThicknessM +
					this->ComputeBodyShadowEquivalentThicknessM(refBodyShadowBlockedThicknessM);
				const double refDropoutProbability =
					this->ComputeDropoutProbability(refDistanceM, refEffectiveBlockedThicknessM);
				const double refNoiseStddevCm =
					this->ComputeNoiseStddevCm(refDistanceM, refEffectiveBlockedThicknessM);

				for (const auto &[anchorName, anchorEntity] : anchors)
				{
					if (anchorEntity == refAnchorEntity)
						continue;

					auto anchorPose = computeWorldPose(anchorEntity, ecm);
					const std::string anchorId = anchorName.substr(anchorName.find_last_of('_') + 1);
					// 3-component pairId (vs. TWR's 2-component "a<id>t<id>")
					// clearly marks this as a differential measurement between
					// two anchors, not a single-anchor range.
					const std::string pairId = "a" + refAnchorId + "a" + anchorId + "t" + tagId;

					const double distanceM = tagPose.Pos().Distance(anchorPose.Pos());
					// Ground truth per the UTIL dataset paper (Zhao et al. 2023,
					// Eq. 1): d_ij = ||p-a_j|| - ||p-a_i||, with anchor i the
					// reference.
					const double groundTruthTdoaCm = (distanceM - refDistanceM) * 100.0;

					const double wallBlockedThicknessM =
						computeAabbBlockedThicknessM(tagPose.Pos(), anchorPose.Pos(), obstacleLinks,
						                             tagEntity, anchorEntity, this->nlosEndpointMarginM_,
						                             ecm, obstacleMaterialMultipliersPtr);
					const double bodyShadowBlockedThicknessM =
						computeAabbBlockedThicknessM(tagPose.Pos(), anchorPose.Pos(), bodyShadowLinks,
						                             tagEntity, anchorEntity, this->nlosEndpointMarginM_, ecm);
					const double effectiveBlockedThicknessM =
						wallBlockedThicknessM +
						this->ComputeBodyShadowEquivalentThicknessM(bodyShadowBlockedThicknessM);
					const double dropoutProbability =
						this->ComputeDropoutProbability(distanceM, effectiveBlockedThicknessM);
					const double noiseStddevCm =
						this->ComputeNoiseStddevCm(distanceM, effectiveBlockedThicknessM);

					// A TDOA reading needs *both* legs (tag-to-reference,
					// tag-to-this-anchor) to succeed; combine as the
					// probability that at least one leg's independent dropout
					// fires. Noise combines as the sum of independent
					// variances (TDOA = difference of two range-like
					// measurements), matching the UTIL-derived calibration
					// note in presets/util_identification.yaml -- unlike that
					// preset's approximation, this is exact here since both
					// legs' per-link curves are the real underlying model.
					const double combinedDropoutProbability =
						1.0 - ((1.0 - refDropoutProbability) * (1.0 - dropoutProbability));
					const double combinedNoiseStddevCm =
						std::sqrt((refNoiseStddevCm * refNoiseStddevCm) +
						          (noiseStddevCm * noiseStddevCm));

					const std::string groundTruthTopic = makePairTopic(this->tdoaGroundTruthTopic_, pairId);
					if (this->groundTruthPublishers_.find(groundTruthTopic) == this->groundTruthPublishers_.end())
					{
						this->groundTruthPublishers_[groundTruthTopic] =
							this->node_.Advertise<gz::msgs::Double>(groundTruthTopic);
					}
					gz::msgs::Double groundTruthMsg;
					groundTruthMsg.set_data(groundTruthTdoaCm);
					this->groundTruthPublishers_[groundTruthTopic].Publish(groundTruthMsg);

					if (std::bernoulli_distribution(combinedDropoutProbability)(this->rng_))
					{
						continue;
					}

					// Unlike TWR's range (which can't be negative, so NLOS
					// noise is folded positive), a TDOA value is a signed
					// difference -- NLOS on either leg can push it either
					// way, so the noise stays a plain, unfolded Gaussian.
					const double noiseCm =
						std::normal_distribution<double>(this->noiseMeanCm_, combinedNoiseStddevCm)(this->rng_);
					double tdoaCm = groundTruthTdoaCm + noiseCm;

					if (this->applyPairBias_)
					{
						const std::string key = "a" + refAnchorId + "a" + anchorId + "t" + tagId;
						auto it = this->pairBiasCm_.find(key);
						if (it == this->pairBiasCm_.end())
						{
							it = this->pairBiasCm_.emplace(key, this->biasCmDist_(this->rng_)).first;
						}
						tdoaCm += it->second;
					}

					const std::string topic = makePairTopic(this->tdoaTopic_, pairId);
					if (this->publishers_.find(topic) == this->publishers_.end())
					{
						this->publishers_[topic] = this->node_.Advertise<gz::msgs::Double>(topic);
					}
					gz::msgs::Double msg;
					msg.set_data(tdoaCm);
					this->publishers_[topic].Publish(msg);
				}
			}
		}
	}
}
