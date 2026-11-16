#include "UWBGazeboSystem.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <exception>
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
#include <array>
#include <limits>
#include <map>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/sim/Util.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Mesh.hh>
#include <sdf/Sphere.hh>
#include <sdf/Collision.hh>
#include <sdf/Model.hh>
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

  std::string toLower(std::string _s)
  {
    std::transform(_s.begin(), _s.end(), _s.begin(),
                   [](const unsigned char c) { return std::tolower(c); });
    return _s;
  }

  // Fallback material tag: a "__material_<name>" suffix on a link or model
  // name, e.g. "wall_1__material_metal". Empty if there's none.
  std::string extractMaterialName(const std::string &_name)
  {
    static const std::string kMarker = "__material_";
    const auto pos = _name.rfind(kMarker);
    if (pos == std::string::npos)
      return "";
    return toLower(_name.substr(pos + kMarker.size()));
  }

  // Preferred material tag: a custom SDF element directly inside a
  // <collision> or <model>, e.g. <uwb:material>metal</uwb:material>. Custom
  // elements need a namespace prefix, declared on the <sdf> root as
  // xmlns:uwb="...". (SDF's own <material> is visual appearance only.)
  // Empty if there's none.
  const std::string kMaterialElement = "uwb:material";

  // Fixed offset of one transceiver, set on its link inside the model:
  // <link name="uwb_anchor_1"><uwb:bias_cm>-9.7</uwb:bias_cm>...</link>.
  // Empty if the link has none or the value isn't a number.
  const std::string kBiasElement = "uwb:bias_cm";
  std::optional<double> customLinkBiasCm(const sdf::ElementPtr &_model,
                                         const std::string &_linkName)
  {
    if (!_model || !_model->HasElement("link"))
      return std::nullopt;
    for (auto link = _model->GetElement("link"); link; link = link->GetNextElement("link"))
    {
      if (!link->HasAttribute("name") ||
          link->GetAttribute("name")->GetAsString() != _linkName ||
          !link->HasElement(kBiasElement))
        continue;
      const std::string text = link->FindElement(kBiasElement)->Get<std::string>();
      try
      {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (text.find_first_not_of(" \t\n\r", used) == std::string::npos && std::isfinite(value))
          return value;
      }
      catch (const std::exception &)
      {
      }
      gzwarn << "[UWBGazeboSystem] <" << kBiasElement << "> of link '" << _linkName
             << "' isn't a number ('" << text << "'). Ignoring it.\n";
      return std::nullopt;
    }
    return std::nullopt;
  }
  std::string customMaterialName(const sdf::ElementPtr &_element)
  {
    if (!_element || !_element->HasElement(kMaterialElement))
      return "";
    std::string material = _element->FindElement(kMaterialElement)->Get<std::string>();
    const auto first = material.find_first_not_of(" \t\n\r");
    const auto last = material.find_last_not_of(" \t\n\r");
    if (first == std::string::npos)
      return "";
    return toLower(material.substr(first, last - first + 1));
  }

  // Material of untagged obstacles (and of every obstacle with the material
  // model off).
  const std::string kGenericMaterial = "generic";

  // The one conductor in the material table: it isn't penetrated like a
  // dielectric (no attenuation rate, no excess-delay bias); the first path
  // detours around it instead (metalDetourLossDb_).
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
  // untagged obstacle's real material is unknown. See
  // docs/measurement_model.md.
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
    double biasCm{0.0};                // beta: excess delay + power bias
    bool isNlos{false};
    bool moving{false};                // the tag or the anchor moved
    double dropoutProbability{0.0};
    double noiseStddevCm{0.0};
  };

  // Allowed measurement rates. 75 Hz is the highest tag rate Eliko offers;
  // Bitcraze's LPS is slower. See docs/measurement_model.md.
  constexpr double kMinUpdateRateHz = 1.0;
  constexpr double kMaxUpdateRateHz = 75.0;
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
    _lossDb += this->metalDetourLossDb_;
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
  // runs out: p = 1 - (1 - p_min)(1 - s(L) / s(L_max)), with s the logistic
  // of (L - L_c) / w. Scaling by s(L_max) makes p reach 1 exactly at L_max,
  // so it rises continuously into blackout.
  const auto logistic = [this](const double _loss)
  {
    return 1.0 / (1.0 + std::exp(-(_loss - this->dropoutCenterLossDb_) / this->dropoutWidthDb_));
  };
  const double scale = logistic(this->maxLinkLossDb_);  // underflows if L_max << L_c
  const double linkFailure = scale > 0.0 ? logistic(_lossDb) / scale : 0.0;
  return 1.0 - ((1.0 - this->minDropoutProbability_) * (1.0 - linkFailure));
}

void UWBGazeboSystem::ValidateAndNormalizeParameters()
{
  if (!(this->updateRateHz_ >= kMinUpdateRateHz && this->updateRateHz_ <= kMaxUpdateRateHz))
  {
    const double clamped = std::isfinite(this->updateRateHz_)
        ? std::clamp(this->updateRateHz_, kMinUpdateRateHz, kMaxUpdateRateHz) : 10.0;
    gzwarn << "[UWBGazeboSystem] update_rate_hz=" << this->updateRateHz_ << " is outside ["
           << kMinUpdateRateHz << ", " << kMaxUpdateRateHz << "] Hz. Using " << clamped << ".\n";
    this->updateRateHz_ = clamped;
  }
  this->updatePeriod_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / this->updateRateHz_));

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
  clampNonNegative(this->dropoutCenterLossDb_, "dropout_center_loss_db");
  clampNonNegative(this->nlosMinLossDb_, "nlos_min_loss_db");
  clampNonNegative(this->bodyShadowMaxLossDb_, "body_shadow_max_loss_db");
  clampNonNegative(this->metalDetourLossDb_, "material_metal_detour_loss_db");
  clampNonNegative(this->movingOutlierProbability_, "moving_outlier_probability");
  clampNonNegative(this->movingOutlierMeanCm_, "moving_outlier_mean_cm");
  clampNonNegative(this->movingOutlierMinSpeedMps_, "moving_outlier_min_speed_mps");
  this->movingOutlierProbability_ = std::min(this->movingOutlierProbability_, 1.0);

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

  readDouble("update_rate_hz", this->updateRateHz_);
  readDouble("path_loss_exponent", this->pathLossExponent_);
  readDouble("max_link_loss_db", this->maxLinkLossDb_);
  readDouble("noise_start_loss_db", this->noiseStartLossDb_);
  readDouble("noise_end_loss_db", this->noiseEndLossDb_);
  readDouble("min_noise_stddev_cm", this->minNoiseStddevCm_);
  readDouble("max_noise_stddev_cm", this->maxNoiseStddevCm_);
  readDouble("min_dropout_probability", this->minDropoutProbability_);
  readDouble("dropout_center_loss_db", this->dropoutCenterLossDb_);
  readDouble("dropout_width_db", this->dropoutWidthDb_);

  readBool("enable_nlos_dropout", this->enableNlosDropout_);
  readDouble("moving_outlier_probability", this->movingOutlierProbability_);
  readDouble("moving_outlier_mean_cm", this->movingOutlierMeanCm_);
  readDouble("moving_outlier_min_speed_mps", this->movingOutlierMinSpeedMps_);
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
  readBool("enable_dielectric_delay", this->enableDielectricDelay_);

  this->ValidateAndNormalizeParameters();

  // Built-in material table at the (validated) carrier frequency. Its
  // entries aren't overridable: a user who needs different values defines a
  // custom material under a new name instead.
  this->materials_.clear();
  this->warnedUnknownMaterials_.clear();
  for (const auto &[materialName, model] : materialModels())
    this->materials_[materialName] = materialAt(model, this->carrierFrequencyGhz_);

  // Custom (user-defined, uncalibrated) materials: any other <name> in a
  // "material_<name>_{permittivity,loss_tangent}" parameter. Both are
  // required, and the rate is 90.96 f sqrt(eps_r) tan(delta) at
  // carrier_frequency_ghz, like the built-in plastic, cardboard and foam.
  // "material_<name>_attenuation_db_per_m" (a fixed rate, which wouldn't
  // follow the frequency) is not a parameter: it's only scanned for here to
  // warn that it's ignored, as is any of these keys on a built-in name.
  this->customMaterials_.clear();
  std::vector<std::string> customNames;
  {
    static const std::string kPrefix = "material_";
    static const std::vector<std::string> kSuffixes = {
        "_attenuation_db_per_m", "_loss_tangent", "_permittivity"};
    std::unordered_set<std::string> seen;
    for (auto child = _sdf ? _sdf->GetFirstElement() : nullptr; child; child = child->GetNextElement())
    {
      const std::string key = child->GetName();
      if (key.rfind(kPrefix, 0) != 0)
        continue;
      for (const auto &suffix : kSuffixes)
      {
        if (key.size() <= kPrefix.size() + suffix.size() ||
            key.compare(key.size() - suffix.size(), suffix.size(), suffix) != 0)
          continue;
        const std::string name =
            key.substr(kPrefix.size(), key.size() - kPrefix.size() - suffix.size());
        if (materialModels().count(name))
          gzwarn << "[UWBGazeboSystem] " << key << " is ignored: built-in materials can't be "
                 << "changed (see docs/measurement_model.md#material-table). Define a custom "
                 << "material under a new name instead"
                 << (name == kConductorMaterial ? ", or set material_metal_detour_loss_db" : "")
                 << ".\n";
        else if (suffix == "_attenuation_db_per_m")
          gzwarn << "[UWBGazeboSystem] " << key << " is ignored: a custom material's rate is "
                 << "computed from material_" << name << "_loss_tangent and material_" << name
                 << "_permittivity, so that it follows carrier_frequency_ghz.\n";
        else if (name != toLower(name))
          gzwarn << "[UWBGazeboSystem] " << key << " is ignored: custom material names must be "
                 << "lowercase (obstacle tags are matched in lowercase).\n";
        else if (seen.insert(name).second)
          customNames.push_back(name);
        break;
      }
    }
  }
  for (const auto &name : customNames)
  {
    const std::string prefix = "material_" + name;
    if (!_sdf->HasElement(prefix + "_loss_tangent") || !_sdf->HasElement(prefix + "_permittivity"))
    {
      gzwarn << "[UWBGazeboSystem] Custom material '" << name << "' is ignored (obstacles tagged "
             << "with it count as generic): it needs both " << prefix << "_loss_tangent and "
             << prefix << "_permittivity.\n";
      continue;
    }

    double permittivity = _sdf->Get<double>(prefix + "_permittivity");
    if (permittivity < 1.0)
    {
      gzwarn << "[UWBGazeboSystem] " << prefix
             << "_permittivity < 1.0 (faster than light). Clamping to 1.0.\n";
      permittivity = 1.0;
    }
    double tanDelta = _sdf->Get<double>(prefix + "_loss_tangent");
    if (tanDelta < 0.0)
    {
      gzwarn << "[UWBGazeboSystem] " << prefix << "_loss_tangent < 0.0. Clamping to 0.0.\n";
      tanDelta = 0.0;
    }
    const MaterialProperties props{
        90.96 * this->carrierFrequencyGhz_ * std::sqrt(permittivity) * tanDelta, permittivity};

    this->materials_[name] = props;
    this->customMaterials_.insert(name);
    gzmsg << "[UWBGazeboSystem] Custom material '" << name << "' (uncalibrated): tan_delta="
          << tanDelta << ", eps_r=" << permittivity << " -> " << props.attenuationDbPerM
          << " dB/m at " << this->carrierFrequencyGhz_ << " GHz.\n";
  }
  if (!this->customMaterials_.empty() && !this->enableMaterialAttenuation_)
    gzwarn << "[UWBGazeboSystem] Custom materials are defined but enable_material_attenuation is "
           << "false: every obstacle counts as generic.\n";

  this->geometryCache_ = std::make_shared<ObstacleGeometryCache>();

  this->biasCmDist_ =
      std::uniform_real_distribution<double>(this->biasMinCm_, this->biasMaxCm_);
  this->pairBiasCm_.clear();
  this->configuredBiasCm_.clear();
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
      if (this->customMaterials_.count(names[i]))
        materialSummary << "(custom)";
      if (names[i] == kConductorMaterial)
        materialSummary << ":{detour_loss_db=" << this->metalDetourLossDb_ << "}";
      else
        materialSummary << ":{dB_per_m=" << props.attenuationDbPerM
                        << ", eps_r=" << props.permittivity << "}";
    }
  }

  gzmsg << "[UWBGazeboSystem] update_rate_hz=" << this->updateRateHz_ << "\n";
  gzmsg << "[UWBGazeboSystem] Link-budget model: path_loss_exponent=" << this->pathLossExponent_
        << ", max_link_loss_db=" << this->maxLinkLossDb_
        << ", noise_loss_range_db=[" << this->noiseStartLossDb_ << ", " << this->noiseEndLossDb_ << "]"
        << ", noise_stddev_cm=[" << this->minNoiseStddevCm_ << ", " << this->maxNoiseStddevCm_ << "]"
        << ", min_dropout_probability=" << this->minDropoutProbability_
        << ", dropout_center_loss_db=" << this->dropoutCenterLossDb_
        << ", dropout_width_db=" << this->dropoutWidthDb_
        << ", enable_nlos_dropout=" << std::boolalpha << this->enableNlosDropout_
        << ", nlos_min_loss_db=" << this->nlosMinLossDb_
        << ", body_shadow_max_loss_db=" << this->bodyShadowMaxLossDb_
        << ", moving_outlier=[p=" << this->movingOutlierProbability_
        << ", mean_cm=" << this->movingOutlierMeanCm_
        << ", min_speed_mps=" << this->movingOutlierMinSpeedMps_ << "]"
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

namespace
{
using Intervals = std::vector<std::pair<double, double>>;

// Adds [_tEnter, _tExit] clipped to the segment, t in [0, 1].
void addClippedInterval(Intervals &_intervals, const double _tEnter, const double _tExit)
{
  const double tEnter = std::max(_tEnter, 0.0);
  const double tExit = std::min(_tExit, 1.0);
  if (tExit > tEnter)
    _intervals.emplace_back(tEnter, tExit);
}

// Roots of a t^2 + b t + c = 0, in order. False if there are none (or a ~ 0).
bool solveQuadratic(const double _a, const double _b, const double _c, double &_t1, double &_t2)
{
  if (std::abs(_a) < 1e-12)
    return false;
  const double disc = _b * _b - 4.0 * _a * _c;
  if (disc < 0.0)
    return false;
  const double sq = std::sqrt(disc);
  _t1 = (-_b - sq) / (2.0 * _a);
  _t2 = (-_b + sq) / (2.0 * _a);
  return true;
}

// Sphere of radius _radius centred at _center, in the segment's frame.
void segmentSphereIntervals(const gz::math::Vector3d &_start, const gz::math::Vector3d &_segment,
                            const gz::math::Vector3d &_center, const double _radius,
                            Intervals &_intervals)
{
  const gz::math::Vector3d rel = _start - _center;
  double t1 = 0.0;
  double t2 = 0.0;
  if (solveQuadratic(_segment.Dot(_segment), 2.0 * _segment.Dot(rel),
                     rel.Dot(rel) - _radius * _radius, t1, t2))
    addClippedInterval(_intervals, t1, t2);
}

// Ellipsoid with semi-axes _radii along the local axes: scaled to a unit sphere.
void segmentEllipsoidIntervals(const gz::math::Vector3d &_start, const gz::math::Vector3d &_segment,
                               const gz::math::Vector3d &_radii, Intervals &_intervals)
{
  if (_radii.X() <= 0.0 || _radii.Y() <= 0.0 || _radii.Z() <= 0.0)
    return;
  const gz::math::Vector3d inv(1.0 / _radii.X(), 1.0 / _radii.Y(), 1.0 / _radii.Z());
  segmentSphereIntervals(_start * inv, _segment * inv, gz::math::Vector3d::Zero, 1.0, _intervals);
}

// Cylinder of radius _radius around the local z axis, z in +-_length/2.
void segmentCylinderIntervals(const gz::math::Vector3d &_start, const gz::math::Vector3d &_segment,
                              const double _radius, const double _length, Intervals &_intervals)
{
  constexpr double kInf = std::numeric_limits<double>::infinity();
  double tMin = -kInf;
  double tMax = kInf;

  // Radial part: inside the infinite cylinder.
  const double a = _segment.X() * _segment.X() + _segment.Y() * _segment.Y();
  const double b = 2.0 * (_start.X() * _segment.X() + _start.Y() * _segment.Y());
  const double c = _start.X() * _start.X() + _start.Y() * _start.Y() - _radius * _radius;
  if (a < 1e-12)
  {
    if (c > 0.0)
      return;  // parallel to the axis and outside it
  }
  else if (!solveQuadratic(a, b, c, tMin, tMax))
  {
    return;
  }

  // Axial part: between the end caps.
  const double halfLength = 0.5 * _length;
  if (std::abs(_segment.Z()) < 1e-12)
  {
    if (std::abs(_start.Z()) > halfLength)
      return;
  }
  else
  {
    double z1 = (-halfLength - _start.Z()) / _segment.Z();
    double z2 = (halfLength - _start.Z()) / _segment.Z();
    if (z1 > z2)
      std::swap(z1, z2);
    tMin = std::max(tMin, z1);
    tMax = std::min(tMax, z2);
  }
  addClippedInterval(_intervals, tMin, tMax);
}

// Capsule: a cylinder plus a sphere on each end cap. The three overlap; the
// intervals are merged later, so the overlap isn't counted twice.
void segmentCapsuleIntervals(const gz::math::Vector3d &_start, const gz::math::Vector3d &_segment,
                             const double _radius, const double _length, Intervals &_intervals)
{
  segmentCylinderIntervals(_start, _segment, _radius, _length, _intervals);
  segmentSphereIntervals(_start, _segment, {0.0, 0.0, 0.5 * _length}, _radius, _intervals);
  segmentSphereIntervals(_start, _segment, {0.0, 0.0, -0.5 * _length}, _radius, _intervals);
}

}  // namespace

namespace custom
{
// A collision mesh in its collision frame (scale applied), with its
// vertices welded by position. Closed means every edge is used as often in
// one direction as in the other: the mesh is one or more closed, consistently
// oriented shells (parts may touch or overlap), turned to face outwards.
// Only then can the segment's crossings tell inside from outside.
struct TriangleMesh
{
  std::vector<gz::math::Vector3d> vertices;
  std::vector<std::array<unsigned int, 3>> triangles;
  gz::math::Vector3d min;
  gz::math::Vector3d max;
  bool closed{false};
};

struct ObstacleGeometryCache
{
  // Per collision entity; null if the mesh couldn't be loaded.
  std::unordered_map<gz::sim::Entity, std::shared_ptr<const TriangleMesh>> meshes;
  // Links already warned about falling back to their bounding box.
  std::unordered_set<gz::sim::Entity> warnedFallback;
};
}  // namespace custom

namespace
{
// Loads, welds and checks an SDF mesh. Null if it can't be loaded or has no
// triangles. _description says what was found, for the log.
std::shared_ptr<const custom::TriangleMesh> buildTriangleMesh(const sdf::Mesh &_meshSdf,
                                                              std::string &_description)
{
  const gz::common::Mesh *mesh = gz::sim::loadMesh(_meshSdf);
  if (!mesh)
  {
    _description = "could not be loaded";
    return nullptr;
  }

  auto shape = std::make_shared<custom::TriangleMesh>();
  // Weld vertices that share a position (to 1 um): exporters split them per
  // face for normals and texture coordinates, which would otherwise make
  // every edge look open.
  constexpr double kWeldM = 1e-6;
  std::map<std::array<long long, 3>, unsigned int> welded;
  auto weld = [&](const gz::math::Vector3d &_v)
  {
    const std::array<long long, 3> key = {std::llround(_v.X() / kWeldM), std::llround(_v.Y() / kWeldM),
                                          std::llround(_v.Z() / kWeldM)};
    const auto [it, inserted] = welded.emplace(key, static_cast<unsigned int>(shape->vertices.size()));
    if (inserted)
      shape->vertices.push_back(_v);
    return it->second;
  };

  const gz::math::Vector3d scale = _meshSdf.Scale();
  for (unsigned int s = 0; s < mesh->SubMeshCount(); ++s)
  {
    const auto subMesh = mesh->SubMeshByIndex(s).lock();
    if (!subMesh || subMesh->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES)
      continue;
    if (!_meshSdf.Submesh().empty() && subMesh->Name() != _meshSdf.Submesh())
      continue;
    // <center_submesh>: the submesh's bounding-box centre moves to the origin.
    gz::math::Vector3d offset = gz::math::Vector3d::Zero;
    if (!_meshSdf.Submesh().empty() && _meshSdf.CenterSubmesh())
      offset = -0.5 * (subMesh->Min() + subMesh->Max());

    const unsigned int count = subMesh->IndexCount() > 0 ? subMesh->IndexCount() : subMesh->VertexCount();
    for (unsigned int i = 0; i + 2 < count; i += 3)
    {
      std::array<unsigned int, 3> tri;
      for (unsigned int k = 0; k < 3; ++k)
      {
        const unsigned int index =
            subMesh->IndexCount() > 0 ? static_cast<unsigned int>(subMesh->Index(i + k)) : i + k;
        tri[k] = weld((subMesh->Vertex(index) + offset) * scale);
      }
      if (tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2])
        shape->triangles.push_back(tri);
    }
  }

  if (shape->triangles.empty())
  {
    _description = "has no triangles";
    return nullptr;
  }

  shape->min = shape->max = shape->vertices.front();
  for (const auto &v : shape->vertices)
  {
    shape->min.Min(v);
    shape->max.Max(v);
  }

  // Closed and consistently oriented: each edge is used as often in each
  // direction (an edge where two touching parts meet is used twice each way).
  std::map<std::pair<unsigned int, unsigned int>, int> edgeBalance;
  for (const auto &tri : shape->triangles)
    for (int k = 0; k < 3; ++k)
    {
      const unsigned int a = tri[k];
      const unsigned int b = tri[(k + 1) % 3];
      edgeBalance[{std::min(a, b), std::max(a, b)}] += a < b ? 1 : -1;
    }
  bool closed = true;
  for (const auto &[edge, balance] : edgeBalance)
  {
    if (balance != 0)
    {
      closed = false;
      break;
    }
  }

  // Outward-facing if the enclosed (signed) volume is positive.
  double volume = 0.0;
  for (const auto &tri : shape->triangles)
    volume += shape->vertices[tri[0]].Dot(shape->vertices[tri[1]].Cross(shape->vertices[tri[2]])) / 6.0;
  if (closed && volume < 0.0)
  {
    for (auto &tri : shape->triangles)
      std::swap(tri[1], tri[2]);
    volume = -volume;
  }
  shape->closed = closed && volume > 0.0;

  std::ostringstream description;
  description << shape->triangles.size() << " triangles, "
              << (shape->closed ? "closed (exact intersection)"
                                : "not closed (using its bounding box in the collision frame)");
  _description = description.str();
  return shape;
}

// Intervals of the segment inside a closed mesh, in the mesh's frame. Every
// triangle the segment's line crosses is an entry (facing against the
// segment) or an exit; walking them in order with a depth counter (the
// winding number) gives where the segment is inside material: wherever the
// depth isn't zero. Overlapping parts count once, and a cavity (an inward-
// facing inner shell) brings the depth back to zero.
void segmentMeshIntervals(const gz::math::Vector3d &_start, const gz::math::Vector3d &_segment,
                          const custom::TriangleMesh &_mesh, Intervals &_intervals)
{
  // Quick rejection against the mesh's own bounding box.
  constexpr double kPad = 1e-6;
  double boxEnter = 0.0;
  double boxExit = 0.0;
  if (!segmentIntersectsBox(_start, _segment, _mesh.min - kPad, _mesh.max + kPad, boxEnter, boxExit))
    return;

  // Moller-Trumbore on the segment's whole line (t unbounded): an endpoint
  // inside the mesh is then still bracketed by an entry and an exit.
  constexpr double kDetEps = 1e-14;
  constexpr double kBaryEps = 1e-9;
  std::vector<std::pair<double, int>> hits;  // (t, +1 entry / -1 exit)
  for (const auto &tri : _mesh.triangles)
  {
    const auto &v0 = _mesh.vertices[tri[0]];
    const gz::math::Vector3d e1 = _mesh.vertices[tri[1]] - v0;
    const gz::math::Vector3d e2 = _mesh.vertices[tri[2]] - v0;
    const gz::math::Vector3d p = _segment.Cross(e2);
    const double det = e1.Dot(p);
    if (std::abs(det) < kDetEps)
      continue;  // the line runs along the triangle's plane
    const double invDet = 1.0 / det;
    const gz::math::Vector3d s = _start - v0;
    const double u = s.Dot(p) * invDet;
    if (u < -kBaryEps || u > 1.0 + kBaryEps)
      continue;
    const gz::math::Vector3d q = s.Cross(e1);
    const double v = _segment.Dot(q) * invDet;
    if (v < -kBaryEps || u + v > 1.0 + kBaryEps)
      continue;
    const double t = e2.Dot(q) * invDet;
    // det = -segment . (e1 x e2), with e1 x e2 the outward normal: the
    // segment enters where it points against the normal.
    hits.emplace_back(t, det > 0.0 ? 1 : -1);
  }
  if (hits.empty())
    return;

  // Entries before exits at the same t, so grazing an edge or a vertex opens
  // and closes an empty interval instead of leaving the counter unbalanced.
  std::sort(hits.begin(), hits.end(),
            [](const auto &_a, const auto &_b)
            { return _a.first < _b.first || (_a.first == _b.first && _a.second > _b.second); });

  // A crossing through a shared edge or vertex is reported by each adjacent
  // triangle: keep one per (t, direction).
  constexpr double kSameT = 1e-9;
  int depth = 0;
  double tEnter = 0.0;
  double lastT = -std::numeric_limits<double>::infinity();
  int lastSign = 0;
  for (const auto &[t, sign] : hits)
  {
    if (sign == lastSign && t - lastT < kSameT)
      continue;
    lastT = t;
    lastSign = sign;
    const int before = depth;
    depth += sign;
    if (before == 0)
      tEnter = t;
    else if (depth == 0)
      addClippedInterval(_intervals, tEnter, t);
  }
}

// Parametric intervals [tEnter, tExit] of the segment inside a link's
// obstacle geometry, each collision intersected exactly in its own (possibly
// rotated) frame: boxes, spheres, ellipsoids, cylinders and capsules
// analytically, and meshes triangle by triangle (closed meshes only; an open
// one uses its bounding box in the collision frame). If the link has any
// other shape (plane, heightmap, polyline, cone) or a mesh that can't be
// loaded, the whole link falls back to its world axis-aligned bounding box,
// which over-estimates. Returns false if there's nothing to intersect.
bool segmentLinkIntervals(const gz::math::Vector3d &_start,
                          const gz::math::Vector3d &_segment,
                          const gz::sim::Entity &_linkEntity,
                          gz::sim::EntityComponentManager &_ecm,
                          custom::ObstacleGeometryCache &_cache,
                          Intervals &_intervals)
{
  const auto collisions =
      _ecm.ChildrenByComponents(_linkEntity, gz::sim::components::Collision());

  Intervals shapeIntervals;
  std::string unsupported = collisions.empty() ? "no collision" : "";
  for (const auto &collision : collisions)
  {
    const auto geometry = _ecm.Component<gz::sim::components::Geometry>(collision);
    if (!geometry)
    {
      unsupported = "collision without geometry";
      break;
    }
    const sdf::Geometry &geom = geometry->Data();

    const gz::math::Pose3d pose = computeWorldPose(collision, _ecm);
    const gz::math::Vector3d localStart = pose.Rot().RotateVectorReverse(_start - pose.Pos());
    const gz::math::Vector3d localSegment = pose.Rot().RotateVectorReverse(_segment);

    double tEnter = 0.0;
    double tExit = 0.0;
    if (geom.Type() == sdf::GeometryType::BOX && geom.BoxShape())
    {
      const gz::math::Vector3d half = geom.BoxShape()->Size() * 0.5;
      if (segmentIntersectsBox(localStart, localSegment, -half, half, tEnter, tExit))
        shapeIntervals.emplace_back(tEnter, tExit);
    }
    else if (geom.Type() == sdf::GeometryType::SPHERE && geom.SphereShape())
    {
      segmentSphereIntervals(localStart, localSegment, gz::math::Vector3d::Zero,
                             geom.SphereShape()->Radius(), shapeIntervals);
    }
    else if (geom.Type() == sdf::GeometryType::ELLIPSOID && geom.EllipsoidShape())
    {
      segmentEllipsoidIntervals(localStart, localSegment, geom.EllipsoidShape()->Radii(), shapeIntervals);
    }
    else if (geom.Type() == sdf::GeometryType::CYLINDER && geom.CylinderShape())
    {
      segmentCylinderIntervals(localStart, localSegment, geom.CylinderShape()->Radius(),
                               geom.CylinderShape()->Length(), shapeIntervals);
    }
    else if (geom.Type() == sdf::GeometryType::CAPSULE && geom.CapsuleShape())
    {
      segmentCapsuleIntervals(localStart, localSegment, geom.CapsuleShape()->Radius(),
                              geom.CapsuleShape()->Length(), shapeIntervals);
    }
    else if (geom.Type() == sdf::GeometryType::MESH && geom.MeshShape())
    {
      auto it = _cache.meshes.find(collision);
      if (it == _cache.meshes.end())
      {
        std::string description;
        auto mesh = buildTriangleMesh(*geom.MeshShape(), description);
        gzmsg << "[UWBGazeboSystem] Obstacle mesh '" << geom.MeshShape()->Uri() << "': "
              << description << ".\n";
        it = _cache.meshes.emplace(collision, std::move(mesh)).first;
      }
      if (!it->second)
      {
        unsupported = "mesh that could not be loaded";
        break;
      }
      if (it->second->closed)
        segmentMeshIntervals(localStart, localSegment, *it->second, shapeIntervals);
      else if (segmentIntersectsBox(localStart, localSegment, it->second->min, it->second->max,
                                    tEnter, tExit))
        shapeIntervals.emplace_back(tEnter, tExit);
    }
    else
    {
      unsupported = "geometry type " + std::to_string(static_cast<int>(geom.Type()));
      break;
    }
  }

  if (unsupported.empty())
  {
    _intervals.insert(_intervals.end(), shapeIntervals.begin(), shapeIntervals.end());
    return true;
  }

  // A link with no box yet, or nothing to bound, reports an infinite box,
  // which would read as blocking every segment.
  const auto worldBox = gz::sim::Link(_linkEntity).WorldAxisAlignedBox(_ecm);
  if (!worldBox.has_value() || !worldBox->Size().IsFinite())
    return false;

  if (_cache.warnedFallback.insert(_linkEntity).second)
  {
    const auto linkName = _ecm.Component<gz::sim::components::Name>(_linkEntity);
    const auto modelName =
        _ecm.Component<gz::sim::components::Name>(_ecm.ParentEntity(_linkEntity));
    gzwarn << "[UWBGazeboSystem] Obstacle link '" << (modelName ? modelName->Data() : "?")
           << "::" << (linkName ? linkName->Data() : "?") << "' has a " << unsupported
           << ": using the link's world axis-aligned bounding box, which over-estimates "
           << "the blocked length.\n";
  }

  double tEnter = 0.0;
  double tExit = 0.0;
  if (segmentIntersectsBox(_start, _segment, worldBox->Min(), worldBox->Max(), tEnter, tExit))
    _intervals.emplace_back(tEnter, tExit);
  return true;
}
}  // namespace

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

// Length of the tag-anchor segment that lies inside obstacles, split
// into untagged obstacles (the generic material) and each tagged material,
// each later multiplied by its attenuation rate. _materials is
// optional (nullptr when the material model is disabled) and maps an
// obstacle link to its material name, resolved once per PreUpdate call;
// links missing from it count as untagged. Intervals are merged *within*
// each group before summing, so overlapping obstacles of the same group (e.g. a
// wall split into several collision links) aren't double-counted, while
// different materials in series each contribute their own length. With
// _materials null, every link is untagged, which reduces to exactly the
// original single-thickness behaviour.
BlockedPath computeBlockedPath(const gz::math::Vector3d &_start,
                                   const gz::math::Vector3d &_end,
                                   const std::vector<gz::sim::Entity> &_allLinks,
                                   const gz::sim::Entity &_tagLinkEntity,
                                   const gz::sim::Entity &_anchorLinkEntity,
                                   const double _endpointMarginM,
                                   gz::sim::EntityComponentManager &_ecm,
                                   custom::ObstacleGeometryCache &_cache,
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
    if (!segmentLinkIntervals(_start, segment, linkEntity, _ecm, _cache, linkIntervals))
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

	// Updates are scheduled on a fixed grid of updatePeriod_, so the rate
	// stays exact when the period isn't a multiple of the physics step. If
	// the simulation falls behind by a whole period, it restarts from now.
	const auto now = info.simTime;
	if (now - this->lastUpdateTime_ < this->updatePeriod_) return;
	this->lastUpdateTime_ += this->updatePeriod_;
	if (now - this->lastUpdateTime_ >= this->updatePeriod_)
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
					      << "'. Tag ids must be globally unique. Skipping duplicate.\n";
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
					      << "'. Anchor ids must be globally unique. Skipping duplicate.\n";
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

	// Offsets configured on the transceivers' links, read once per link.
	auto configuredBiasCm = [&](const gz::sim::Entity &_link, const std::string &_linkName)
	{
		auto it = this->configuredBiasCm_.find(_link);
		if (it == this->configuredBiasCm_.end())
		{
			std::optional<double> value;
			const auto modelSdf = ecm.Component<gz::sim::components::ModelSdf>(ecm.ParentEntity(_link));
			if (modelSdf)
				value = customLinkBiasCm(modelSdf->Data().Element(), _linkName);
			if (value)
				gzmsg << "[UWBGazeboSystem] " << _linkName << ": configured bias " << *value << " cm.\n";
			it = this->configuredBiasCm_.emplace(_link, value).first;
		}
		return it->second;
	};

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
			// Most specific first: <uwb:material> on one of the link's
			// collisions, the link name's suffix, <uwb:material> on the model,
			// then the model name's suffix.
			std::string materialName;
			for (const auto &collision :
			     ecm.ChildrenByComponents(linkEntity, gz::sim::components::Collision()))
			{
				const auto element = ecm.Component<gz::sim::components::CollisionElement>(collision);
				if (element)
					materialName = customMaterialName(element->Data().Element());
				if (!materialName.empty())
					break;
			}

			if (materialName.empty())
			{
				const auto linkNameComp = ecm.Component<gz::sim::components::Name>(linkEntity);
				if (linkNameComp)
					materialName = extractMaterialName(linkNameComp->Data());
			}

			const auto parentEntity = ecm.ParentEntity(linkEntity);
			if (materialName.empty())
			{
				const auto modelSdf = ecm.Component<gz::sim::components::ModelSdf>(parentEntity);
				if (modelSdf)
					materialName = customMaterialName(modelSdf->Data().Element());
			}

			if (materialName.empty())
			{
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
					       << "). Treating it as generic. To define it, set material_" << materialName
					       << "_loss_tangent and material_" << materialName
					       << "_permittivity (see docs/measurement_model.md#custom-materials).\n";
				}
				continue;
			}

			obstacleMaterials[linkEntity] = materialName;
		}
	}
	const std::unordered_map<gz::sim::Entity, std::string> *obstacleMaterialsPtr =
		this->enableMaterialAttenuation_ ? &obstacleMaterials : nullptr;

	// Which tags and anchors moved since the last update (for the moving
	// outliers). UTIL's anchors never move, so its data can't tell tag motion
	// from relative motion: either end moving counts.
	std::unordered_map<gz::sim::Entity, bool> sensorMoving;
	for (const auto *sensors : {&tags, &anchors})
	{
		for (const auto &[sensorName, sensorEntity] : *sensors)
		{
			const gz::math::Vector3d sensorPos = computeWorldPose(sensorEntity, ecm).Pos();
			const auto last = this->lastSensorPosition_.find(sensorEntity);
			sensorMoving[sensorEntity] = last != this->lastSensorPosition_.end() &&
			                             sensorPos.Distance(last->second) > this->movingOutlierMinSpeedMps_ / this->updateRateHz_;
			this->lastSensorPosition_[sensorEntity] = sensorPos;
		}
	}

	// Geometry + link budget of one tag-anchor link: what the ray crosses,
	// the excess loss L = L_d + L_obs, and from it the LOS/NLOS regime, noise
	// stddev and dropout probability. Shared by TWR (one link per
	// measurement) and TDOA (two links per measurement). See
	// docs/measurement_model.md.
	auto evaluateLink = [&](const gz::sim::Entity &_tagEntity,
	                        const gz::math::Vector3d &_tagPos,
	                        const gz::sim::Entity &_anchorEntity,
	                        const gz::math::Vector3d &_anchorPos)
	{
		LinkState link;
		link.distanceM = _tagPos.Distance(_anchorPos);
		link.moving = sensorMoving.at(_tagEntity) || sensorMoving.at(_anchorEntity);

		const BlockedPath wallPath =
			computeBlockedPath(_tagPos, _anchorPos, obstacleLinks, _tagEntity, _anchorEntity,
			                   this->nlosEndpointMarginM_, ecm, *this->geometryCache_, obstacleMaterialsPtr);
		const BlockedPath bodyPath =
			computeBlockedPath(_tagPos, _anchorPos, bodyShadowLinks, _tagEntity, _anchorEntity,
			                   this->nlosEndpointMarginM_, ecm, *this->geometryCache_);

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
		// Occasional late first path, seen only on moving tags in UTIL's data.
		if (_link.moving && this->movingOutlierProbability_ > 0.0 &&
		    std::bernoulli_distribution(this->movingOutlierProbability_)(this->rng_))
		{
			noiseCm += std::exponential_distribution<double>(1.0 / this->movingOutlierMeanCm_)(this->rng_);
		}
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

				// r = d + n + beta + b_pair. b_pair is the anchor's plus the tag's
				// configured offset when either has one, else a random draw.
				double dist = groundTruthDistCm + drawLinkErrorCm(link);
				const auto anchorBias = configuredBiasCm(anchorEntity, anchorName);
				const auto tagBias = configuredBiasCm(tagEntity, tagName);
				if (anchorBias || tagBias)
					dist += anchorBias.value_or(0.0) + tagBias.value_or(0.0);
				else if (this->applyPairBias_)
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
				// The tag's own offset cancels in the difference.
				if (const auto bias = configuredBiasCm(refAnchorEntity, refAnchorName))
					refErrorCm += *bias;
				else if (this->applyPairBias_)
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
				if (const auto bias = configuredBiasCm(anchorEntity, anchorName))
					legErrorCm += *bias;
				else if (this->applyPairBias_)
					legErrorCm += persistentBiasCm("tdoa_a" + anchorId);

				publishTo(this->publishers_,
				          makePairTopic(this->tdoaTopic_, pairId),
				          groundTruthTdoaCm + legErrorCm - refErrorCm);
			}
		}
	}
}
