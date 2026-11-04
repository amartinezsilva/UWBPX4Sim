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
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Box.hh>
#include <sdf/Element.hh>
#include <sdf/Geometry.hh>

using namespace custom;

GZ_ADD_PLUGIN(UWBGazeboSystem,
              gz::sim::System,
              UWBGazeboSystem::ISystemConfigure,
              UWBGazeboSystem::ISystemPreUpdate)

// Helper functions
namespace
{
constexpr double kPositiveEpsilon = 1e-6;

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
  // Returns empty (no match), so the caller can treat "no tag at all" and
  // "tag not in the material table" uniformly as the generic material.
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

  // Material of untagged obstacles (and of every obstacle with the material
  // model off).
  const std::string kGenericMaterial = "generic";

  // The one conductor in the material table: it isn't penetrated like a
  // dielectric (no attenuation rate, no excess-delay bias); the first path
  // detours around it instead (metalDetourLossDb_ / metalDetourBiasCm_).
  const std::string kConductorMaterial = "metal";

  // How a material's attenuation rate is derived from the carrier frequency.
  struct MaterialModel
  {
    enum class Kind { Itu, LossTangent, Conductor };
    Kind kind;
    // Itu: ITU-R P.2040 Table 3 coefficients -- eps_r = a, conductivity
    // sigma = c f^d S/m (f in GHz), rate = 1636 sigma / sqrt(eps_r) dB/m.
    double a{1.0}, c{0.0}, d{0.0};
    // LossTangent: rate = (8.686 pi f / c0) sqrt(eps_r) tan(delta)
    //                   = 90.96 f[GHz] sqrt(eps_r) tan(delta) dB/m.
    double tanDelta{0.0};
    // eps_r used for the excess-delay bias (the ITU a, or the given value).
    double permittivity{1.0};
  };

  // Default material table. "generic" uses plasterboard's attenuation (a
  // typical interior wall) but no excess-delay bias by default, since an
  // untagged obstacle's real material is unknown. See README.md,
  // "Measurement model".
  const std::unordered_map<std::string, MaterialModel> &materialModels()
  {
    using Kind = MaterialModel::Kind;
    static const std::unordered_map<std::string, MaterialModel> models = {
        {"generic", {Kind::Itu, 2.73, 0.0085, 0.9395, 0.0, 1.0}},
        {"plasterboard", {Kind::Itu, 2.73, 0.0085, 0.9395, 0.0, 2.73}},
        {"concrete", {Kind::Itu, 5.24, 0.0462, 0.7822, 0.0, 5.24}},
        {"wood", {Kind::Itu, 1.99, 0.0047, 1.0718, 0.0, 1.99}},
        {"glass", {Kind::Itu, 6.31, 0.0036, 1.3394, 0.0, 6.31}},
        {"plastic", {Kind::LossTangent, 2.5, 0.0, 0.0, 0.01, 2.5}},
        {"cardboard", {Kind::LossTangent, 1.2, 0.0, 0.0, 0.003, 1.2}},
        {"foam", {Kind::LossTangent, 1.05, 0.0, 0.0, 0.0005, 1.05}},
        {"metal", {Kind::Conductor}},
    };
    return models;
  }

  UWBGazeboSystem::MaterialProperties materialAt(const MaterialModel &_model,
                                                 const double _frequencyGhz)
  {
    switch (_model.kind)
    {
      case MaterialModel::Kind::Itu:
      {
        const double conductivity = _model.c * std::pow(_frequencyGhz, _model.d);
        return {1636.0 * conductivity / std::sqrt(_model.a), _model.permittivity};
      }
      case MaterialModel::Kind::LossTangent:
        return {90.96 * _frequencyGhz * std::sqrt(_model.a) * _model.tanDelta, _model.permittivity};
      case MaterialModel::Kind::Conductor:
      default:
        return {0.0, 1.0};
    }
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
    double wallThicknessM{0.0};        // untagged wall chord length
    double bodyShadowThicknessM{0.0};  // robot-body chord length
    double materialLengthM{0.0};       // total tagged-material chord length
    double pathLossDb{0.0};            // L_d = 10 n log10(d)
    double obstacleLossDb{0.0};        // L_obs: walls, materials, bodies
    double biasCm{0.0};                // beta: excess delay + metal detour + power bias
    bool isNlos{false};
    double dropoutProbability{0.0};
    double noiseStddevCm{0.0};
  };

  // Parameters of the previous (severity-curve) model. They no longer
  // exist; Configure() says so rather than silently ignoring an old preset.
  const char *const kRemovedParameters[] = {
      "los_dropout_start_distance_m", "los_dropout_end_distance_m",
      "los_hard_dropout_distance_m", "los_stddev_start_distance_m",
      "los_stddev_end_distance_m", "los_max_thickness_m",
      "soft_nlos_max_thickness_m", "blackout_thickness_m", "hard_nlos_gap_ratio",
      "max_dropout_probability", "nlos_dropout_thickness_extra_weight",
      "nlos_stddev_thickness_extra_weight", "nlos_dropout_material_extra_weight",
      "nlos_stddev_material_extra_weight", "material_los_max_severity",
      "material_attenuation_default"};
}  // namespace

double UWBGazeboSystem::PathLossDb(const double _distanceM) const
{
  // Relative to 1 m; closer than that counts as 1 m (no negative loss).
  return this->pathLossExponent_ * 10.0 * std::log10(std::max(_distanceM, 1.0));
}

void UWBGazeboSystem::ComputeObstacleEffects(
    const std::unordered_map<std::string, double> &_materialLengthM,
    double &_lossDb,
    double &_biasCm) const
{
  bool crossesConductor = false;
  _lossDb = 0.0;
  _biasCm = 0.0;

  for (const auto &[materialName, lengthM] : _materialLengthM)
  {
    const auto it = this->materials_.find(materialName);
    if (it == this->materials_.end() || lengthM <= 0.0)
      continue;

    if (materialName == kConductorMaterial)
    {
      crossesConductor = true;
      continue;
    }

    _lossDb += it->second.attenuationDbPerM * lengthM;
    // Excess delay of propagating through a dielectric at c/sqrt(eps_r)
    // instead of c: beta = l (sqrt(eps_r) - 1). Only if enabled.
    if (this->enableDielectricDelay_)
      _biasCm += lengthM * (std::sqrt(it->second.permittivity) - 1.0) * 100.0;
  }

  // Applied once per link, however many metal obstacles it crosses: the
  // first path detours around the whole obstruction.
  if (crossesConductor)
  {
    _lossDb += this->metalDetourLossDb_;
    _biasCm += this->metalDetourBiasCm_;
  }
}

double UWBGazeboSystem::ComputeNoiseStddevCm(const double _lossDb) const
{
  // sigma = sigma_min (sigma_max / sigma_min)^ramp(L): exponential in dB,
  // i.e. a power law in distance.
  const double severity =
      normalizedRamp(_lossDb, this->noiseStartLossDb_, this->noiseEndLossDb_);
  return this->minNoiseStddevCm_ *
         std::pow(this->maxNoiseStddevCm_ / this->minNoiseStddevCm_, severity);
}

double UWBGazeboSystem::ComputeDropoutProbability(const double _lossDb) const
{
  if (_lossDb >= this->maxLinkLossDb_)
    return 1.0;

  // Independent floor (random packet loss) and link failure as the budget
  // runs out: p = 1 - (1 - p_min)(1 - logistic((L - L50) / w)).
  const double linkFailure =
      1.0 / (1.0 + std::exp(-(_lossDb - this->dropoutMidLossDb_) / this->dropoutWidthDb_));
  return 1.0 - ((1.0 - this->minDropoutProbability_) * (1.0 - linkFailure));
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

  if (this->pathLossExponent_ < 0.1)
  {
    gzwarn << "[UWBGazeboSystem] path_loss_exponent < 0.1. Clamping to 0.1.\n";
    this->pathLossExponent_ = 0.1;
  }

  auto clampNonNegative = [](double &_value, const char *_name)
  {
    if (_value < 0.0)
    {
      gzwarn << "[UWBGazeboSystem] " << _name << " < 0.0. Clamping to 0.0.\n";
      _value = 0.0;
    }
  };
  clampNonNegative(this->maxLinkLossDb_, "max_link_loss_db");
  clampNonNegative(this->noiseStartLossDb_, "noise_start_loss_db");
  clampNonNegative(this->noiseEndLossDb_, "noise_end_loss_db");
  clampNonNegative(this->dropoutMidLossDb_, "dropout_mid_loss_db");
  clampNonNegative(this->nlosMinLossDb_, "nlos_min_loss_db");
  clampNonNegative(this->bodyShadowMaxLossDb_, "body_shadow_max_loss_db");
  clampNonNegative(this->metalDetourLossDb_, "material_metal_detour_loss_db");
  clampNonNegative(this->metalDetourBiasCm_, "material_metal_detour_bias_cm");

  if (this->noiseEndLossDb_ < this->noiseStartLossDb_)
  {
    gzwarn << "[UWBGazeboSystem] noise_end_loss_db < noise_start_loss_db. Swapping values.\n";
    std::swap(this->noiseStartLossDb_, this->noiseEndLossDb_);
  }

  this->minNoiseStddevCm_ = std::max(this->minNoiseStddevCm_, kPositiveEpsilon);
  this->maxNoiseStddevCm_ = std::max(this->maxNoiseStddevCm_, kPositiveEpsilon);
  if (this->minNoiseStddevCm_ > this->maxNoiseStddevCm_)
  {
    gzwarn << "[UWBGazeboSystem] min_noise_stddev_cm > max_noise_stddev_cm. Swapping values.\n";
    std::swap(this->minNoiseStddevCm_, this->maxNoiseStddevCm_);
  }

  if (this->minDropoutProbability_ < 0.0 || this->minDropoutProbability_ > 1.0)
  {
    this->minDropoutProbability_ = std::clamp(this->minDropoutProbability_, 0.0, 1.0);
    gzwarn << "[UWBGazeboSystem] min_dropout_probability out of [0,1]. Clamped to "
           << this->minDropoutProbability_ << ".\n";
  }

  if (this->dropoutWidthDb_ < 0.01)
  {
    gzwarn << "[UWBGazeboSystem] dropout_width_db < 0.01. Clamping to 0.01.\n";
    this->dropoutWidthDb_ = 0.01;
  }

  // ITU-R P.2040's material coefficients are fitted over 1-100 GHz.
  if (this->carrierFrequencyGhz_ < 1.0 || this->carrierFrequencyGhz_ > 100.0)
  {
    this->carrierFrequencyGhz_ = std::clamp(this->carrierFrequencyGhz_, 1.0, 100.0);
    gzwarn << "[UWBGazeboSystem] carrier_frequency_ghz out of ITU-R P.2040's [1,100] GHz. Clamped to "
           << this->carrierFrequencyGhz_ << ".\n";
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
}

void UWBGazeboSystem::Configure(const gz::sim::Entity &,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &,
                                gz::sim::EventManager &)
{
  auto readDouble = [&](const char *_name, double &_value)
  {
    if (_sdf && _sdf->HasElement(_name))
      _value = _sdf->Get<double>(_name);
  };
  auto readBool = [&](const char *_name, bool &_value)
  {
    if (_sdf && _sdf->HasElement(_name))
      _value = _sdf->Get<bool>(_name);
  };
  auto readString = [&](const char *_name, std::string &_value)
  {
    if (_sdf && _sdf->HasElement(_name))
      _value = _sdf->Get<std::string>(_name);
  };

  readDouble("gaussian_noise_mean_cm", this->noiseMeanCm_);
  readDouble("bias_min_cm", this->biasMinCm_);
  readDouble("bias_max_cm", this->biasMaxCm_);
  readBool("apply_pair_bias", this->applyPairBias_);

  if (_sdf && _sdf->HasElement("random_seed"))
    this->randomSeed_ = _sdf->Get<unsigned int>("random_seed");
  if (this->randomSeed_ != 0)
    this->rng_.seed(this->randomSeed_);

  readDouble("path_loss_exponent", this->pathLossExponent_);
  readDouble("max_link_loss_db", this->maxLinkLossDb_);
  readDouble("noise_start_loss_db", this->noiseStartLossDb_);
  readDouble("noise_end_loss_db", this->noiseEndLossDb_);
  readDouble("min_noise_stddev_cm", this->minNoiseStddevCm_);
  readDouble("max_noise_stddev_cm", this->maxNoiseStddevCm_);
  readDouble("min_dropout_probability", this->minDropoutProbability_);
  readDouble("dropout_mid_loss_db", this->dropoutMidLossDb_);
  readDouble("dropout_width_db", this->dropoutWidthDb_);

  readBool("enable_nlos_dropout", this->enableNlosDropout_);
  readDouble("nlos_endpoint_margin_m", this->nlosEndpointMarginM_);
  readDouble("nlos_min_loss_db", this->nlosMinLossDb_);
  readDouble("body_shadow_max_loss_db", this->bodyShadowMaxLossDb_);
  readDouble("power_bias_cm_per_db", this->powerBiasCmPerDb_);
  readDouble("power_bias_ref_loss_db", this->powerBiasRefLossDb_);

  readString("topic", this->topic_);
  readString("ground_truth_topic", this->groundTruthTopic_);

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

  readString("tdoa_topic", this->tdoaTopic_);
  readString("tdoa_ground_truth_topic", this->tdoaGroundTruthTopic_);

  readBool("enable_material_attenuation", this->enableMaterialAttenuation_);
  readDouble("carrier_frequency_ghz", this->carrierFrequencyGhz_);
  readDouble("material_metal_detour_loss_db", this->metalDetourLossDb_);
  readDouble("material_metal_detour_bias_cm", this->metalDetourBiasCm_);
  readBool("enable_dielectric_delay", this->enableDielectricDelay_);

  for (const char *removed : kRemovedParameters)
  {
    if (_sdf && _sdf->HasElement(removed))
    {
      gzwarn << "[UWBGazeboSystem] " << removed << " is no longer used (the model is now a "
                "link budget; see README.md, \"Measurement model\"). Ignoring it.\n";
    }
  }

  this->ValidateAndNormalizeParameters();

  // Material table at the (validated) carrier frequency, then per-material
  // overrides: scalar "material_<name>_attenuation_db_per_m" /
  // "material_<name>_permittivity" parameters (neither for the conductor).
  // Add a material to materialModels() (and to gui/app.py's PARAM_GROUPS)
  // if a world needs one not listed.
  this->materials_.clear();
  this->warnedUnknownMaterials_.clear();
  for (const auto &[materialName, model] : materialModels())
  {
    auto props = materialAt(model, this->carrierFrequencyGhz_);
    if (materialName != kConductorMaterial)
    {
      const std::string prefix = "material_" + materialName;
      if (_sdf && _sdf->HasElement(prefix + "_attenuation_db_per_m"))
        props.attenuationDbPerM = _sdf->Get<double>(prefix + "_attenuation_db_per_m");
      if (_sdf && _sdf->HasElement(prefix + "_permittivity"))
        props.permittivity = _sdf->Get<double>(prefix + "_permittivity");
      if (_sdf && _sdf->HasElement(prefix + "_penetration_m"))
      {
        gzwarn << "[UWBGazeboSystem] " << prefix << "_penetration_m is no longer used "
                  "(replaced by " << prefix << "_attenuation_db_per_m). Ignoring it.\n";
      }

      if (props.attenuationDbPerM < 0.0)
      {
        gzwarn << "[UWBGazeboSystem] " << prefix << "_attenuation_db_per_m < 0.0. Clamping to 0.0.\n";
        props.attenuationDbPerM = 0.0;
      }
      if (props.permittivity < 1.0)
      {
        gzwarn << "[UWBGazeboSystem] " << prefix
               << "_permittivity < 1.0 (faster than light). Clamping to 1.0.\n";
        props.permittivity = 1.0;
      }
    }
    this->materials_[materialName] = props;
  }

  this->biasCmDist_ =
      std::uniform_real_distribution<double>(this->biasMinCm_, this->biasMaxCm_);
  this->pairBiasCm_.clear();
  this->publishers_.clear();
  this->groundTruthPublishers_.clear();

  std::ostringstream materialSummary;
  {
    std::vector<std::string> names;
    for (const auto &[name, props] : this->materials_)
      names.push_back(name);
    std::sort(names.begin(), names.end());
    for (size_t i = 0; i < names.size(); ++i)
    {
      const auto &props = this->materials_.at(names[i]);
      materialSummary << (i ? ", " : "") << names[i];
      if (names[i] == kConductorMaterial)
        materialSummary << ":{detour_loss_db=" << this->metalDetourLossDb_
                        << ", detour_bias_cm=" << this->metalDetourBiasCm_ << "}";
      else
        materialSummary << ":{dB_per_m=" << props.attenuationDbPerM
                        << ", eps_r=" << props.permittivity << "}";
    }
  }

  gzmsg << "[UWBGazeboSystem] Link-budget model: path_loss_exponent=" << this->pathLossExponent_
        << ", max_link_loss_db=" << this->maxLinkLossDb_
        << ", noise_loss_range_db=[" << this->noiseStartLossDb_ << ", " << this->noiseEndLossDb_ << "]"
        << ", noise_stddev_cm=[" << this->minNoiseStddevCm_ << ", " << this->maxNoiseStddevCm_ << "]"
        << ", min_dropout_probability=" << this->minDropoutProbability_
        << ", dropout_mid_loss_db=" << this->dropoutMidLossDb_
        << ", dropout_width_db=" << this->dropoutWidthDb_
        << ", enable_nlos_dropout=" << std::boolalpha << this->enableNlosDropout_
        << ", nlos_min_loss_db=" << this->nlosMinLossDb_
        << ", body_shadow_max_loss_db=" << this->bodyShadowMaxLossDb_
        << ", power_bias_cm_per_db=" << this->powerBiasCmPerDb_
        << ", power_bias_ref_loss_db=" << this->powerBiasRefLossDb_
        << ", nlos_endpoint_margin_m=" << this->nlosEndpointMarginM_
        << ", gaussian_mean_cm=" << this->noiseMeanCm_
        << ", apply_pair_bias=" << std::boolalpha << this->applyPairBias_
        << ", bias_cm=[" << this->biasMinCm_ << ", " << this->biasMaxCm_ << "]"
        << ", random_seed=" << this->randomSeed_
        << ", uwb_mode=" << (this->uwbMode_ == UwbMode::Tdoa ? "tdoa" : "twr")
        << ", topic=" << this->topic_
        << ", ground_truth_topic=" << this->groundTruthTopic_
        << ", enable_material_attenuation=" << std::boolalpha << this->enableMaterialAttenuation_
        << ", carrier_frequency_ghz=" << this->carrierFrequencyGhz_
        << ", enable_dielectric_delay=" << std::boolalpha << this->enableDielectricDelay_
        << ", materials={" << materialSummary.str() << "}\n";
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

// Slab test of the segment _start + t * _segment, t in [0, 1], against the
// box [_boxMin, _boxMax] in the same frame.
bool segmentIntersectsBox(const gz::math::Vector3d &_start,
                          const gz::math::Vector3d &_segment,
                          const gz::math::Vector3d &_boxMin,
                          const gz::math::Vector3d &_boxMax,
                          double &_tEnter,
                          double &_tExit)
{
  constexpr double kParallelEps = 1e-9;

  double tMin = 0.0;
  double tMax = 1.0;

  const auto &boxMin = _boxMin;
  const auto &boxMax = _boxMax;

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

// Parametric intervals [tEnter, tExit] of the segment inside a link's
// obstacle geometry. Box collisions are intersected exactly in their own
// (possibly rotated) frame: the segment is transformed into the box frame
// and slab-tested against +-size/2. If the link has any non-box collision,
// the whole link falls back to its world axis-aligned bounding box, which
// over-estimates rotated or non-box shapes. Returns false if the link has
// neither (no collision geometry and no AABB).
bool segmentLinkIntervals(const gz::math::Vector3d &_start,
                          const gz::math::Vector3d &_segment,
                          const gz::sim::Entity &_linkEntity,
                          gz::sim::EntityComponentManager &_ecm,
                          std::vector<std::pair<double, double>> &_intervals)
{
  const auto collisions =
      _ecm.ChildrenByComponents(_linkEntity, gz::sim::components::Collision());

  std::vector<std::pair<double, double>> boxIntervals;
  bool allBoxes = !collisions.empty();
  for (const auto &collision : collisions)
  {
    const auto geometry = _ecm.Component<gz::sim::components::Geometry>(collision);
    if (!geometry || geometry->Data().Type() != sdf::GeometryType::BOX ||
        geometry->Data().BoxShape() == nullptr)
    {
      allBoxes = false;
      break;
    }

    const gz::math::Pose3d boxPose = computeWorldPose(collision, _ecm);
    const gz::math::Vector3d half = geometry->Data().BoxShape()->Size() * 0.5;
    const gz::math::Vector3d localStart =
        boxPose.Rot().RotateVectorReverse(_start - boxPose.Pos());
    const gz::math::Vector3d localSegment = boxPose.Rot().RotateVectorReverse(_segment);

    double tEnter = 0.0;
    double tExit = 0.0;
    if (segmentIntersectsBox(localStart, localSegment, -half, half, tEnter, tExit))
      boxIntervals.emplace_back(tEnter, tExit);
  }

  if (allBoxes)
  {
    _intervals.insert(_intervals.end(), boxIntervals.begin(), boxIntervals.end());
    return true;
  }

  const auto worldBox = gz::sim::Link(_linkEntity).WorldAxisAlignedBox(_ecm);
  if (!worldBox.has_value())
    return false;

  double tEnter = 0.0;
  double tExit = 0.0;
  if (segmentIntersectsBox(_start, _segment, worldBox->Min(), worldBox->Max(), tEnter, tExit))
    _intervals.emplace_back(tEnter, tExit);
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
// into untagged obstacles (the generic material) and each tagged material,
// each later multiplied by its attenuation rate. _materials is
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

    std::vector<std::pair<double, double>> linkIntervals;
    if (!segmentLinkIntervals(_start, segment, linkEntity, _ecm, linkIntervals))
      continue;

    std::string materialName;
    if (_materials)
    {
      const auto it = _materials->find(linkEntity);
      if (it != _materials->end())
        materialName = it->second;
    }

    for (const auto &[tEnter, tExit] : linkIntervals)
    {
      const double clippedEnter = std::max(tEnter, tMinAllowed);
      const double clippedExit = std::min(tExit, tMaxAllowed);
      if (clippedExit > clippedEnter)
        intervalsByMaterial[materialName].emplace_back(clippedEnter, clippedExit);
    }
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
	// material-aware; robot bodyShadowLinks always use the generic rate
	// (capped), since a vehicle's own body isn't an NLOS "material" a world
	// author would tag. Obstacles left out of the map (untagged, or tagged
	// with a material not in materials_) are the generic material too.
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
					       << "). Treating it as generic.\n";
				}
				continue;
			}

			obstacleMaterials[linkEntity] = materialName;
		}
	}
	const std::unordered_map<gz::sim::Entity, std::string> *obstacleMaterialsPtr =
		this->enableMaterialAttenuation_ ? &obstacleMaterials : nullptr;

	// Geometry + link budget of one tag-anchor link: what the ray crosses,
	// the excess loss L = L_d + L_obs, and from it the LOS/NLOS regime, noise
	// stddev and dropout probability. Shared by TWR (one link per
	// measurement) and TDOA (two links per measurement). See README.md,
	// "Measurement model".
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

		// Untagged walls are the generic material.
		std::unordered_map<std::string, double> materialLengthM = wallPath.materialLengthM;
		if (wallPath.untaggedM > 0.0)
			materialLengthM[kGenericMaterial] += wallPath.untaggedM;

		double wallLossDb = 0.0;
		this->ComputeObstacleEffects(materialLengthM, wallLossDb, link.biasCm);
		const double bodyLossDb = std::min(
			this->materials_.at(kGenericMaterial).attenuationDbPerM * bodyPath.untaggedM,
			this->bodyShadowMaxLossDb_);

		link.pathLossDb = this->PathLossDb(link.distanceM);
		link.obstacleLossDb = wallLossDb + bodyLossDb;
		link.isNlos = link.obstacleLossDb >= this->nlosMinLossDb_;

		const double totalLossDb = link.pathLossDb + link.obstacleLossDb;
		link.biasCm += this->powerBiasCmPerDb_ * (totalLossDb - this->powerBiasRefLossDb_);
		link.noiseStddevCm = this->ComputeNoiseStddevCm(totalLossDb);
		link.dropoutProbability = this->ComputeDropoutProbability(
			this->enableNlosDropout_ ? totalLossDb : link.pathLossDb);
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
		return noiseCm + _link.biasCm;
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
					      << ", material_blocked_length_m=" << link.materialLengthM
					      << ", path_loss_db=" << link.pathLossDb
					      << ", obstacle_loss_db=" << link.obstacleLossDb
					      << ", bias_cm=" << link.biasCm
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
