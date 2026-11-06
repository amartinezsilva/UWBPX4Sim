# Measurement model

The `UWBGazeboPlugin` system simulates UWB measurements between anchors and tags with a **link budget**. Every 100 ms of simulation time it computes, for each tag-anchor link, the excess loss $`L`$ (in dB) from free-space path loss plus every obstacle the line of sight crosses; $`L`$ sets the noise standard deviation $`\sigma`$ and the dropout probability $`p`$. It then publishes a ground truth and a simulated measurement: one link per TWR range, two links per TDOA value. Because distance and obstacles add up in the same dB budget, their relative influence follows from physical magnitudes: doubling the distance costs 6 dB, a 25 cm interior wall about 8 dB, and a 20 cm concrete wall about 20 dB (at 4 GHz).

Parameters are introduced below as `code` next to the symbol they set; all of them are listed in [Parameter reference](#parameter-reference). They live in a preset under `uwb_gazebo_plugin/presets/` (a plain `key: value` file; keys left out use the compiled-in defaults), which `./setup_simulator.sh` syncs into the plugin's block in PX4's `server.config` on every run -- pick one with `--params <name>`, in the setup GUI's "Plugin Parameters" tab, or from the terminal picker. The two shipped presets, `eliko_identification` (Eliko TWR, the default under `-y`) and `util_identification` (calibrated from the [UTIL](https://utiasdsl.github.io/util-uwb-dataset/) TDOA dataset; see [Calibration](#calibration)), are read-only from the GUI: tune one and save it under a new name.

## Ranging modes and topics

`uwb_mode` selects the ranging scheme for the whole simulation:

- **`twr`** (default) -- two-way ranging, one range per anchor-tag pair, published on `<topic>/aItJ` and `<ground_truth_topic>/aItJ` (anchor `I`, tag `J`; defaults `/uwb_gz_simulator/distances[_ground_truth]`). The offboard nodes re-package them into `eliko_messages/DistancesList`, the same format as the real Eliko driver.
- **`tdoa`** -- time difference of arrival against a reference anchor, as in Bitcraze's Loco Positioning System, published on `<tdoa_topic>/aIaKtJ` and `<tdoa_ground_truth_topic>/aIaKtJ` (reference `I`, anchor `K`, tag `J`; defaults `/uwb_gz_simulator/tdoa[_ground_truth]`). There's no `eliko_messages` equivalent, so these stay on GZ topics only.

## Link geometry

For a tag at $`\mathbf{p}_t`$ and an anchor at $`\mathbf{a}`$, the true range is $`d = \lVert \mathbf{p}_t - \mathbf{a} \rVert`$. The segment between them, minus `nlos_endpoint_margin_m` at each end (to ignore the vehicles' own mounts), is intersected with every obstacle -- every world model except `ground_plane` and the UWB-carrying vehicles: exactly with box collisions (in each box's own, possibly rotated, frame), and through the link's world-axis-aligned bounding box (AABB) for any other shape. Chord lengths are merged per material, so overlapping links of one wall aren't counted twice:

- $`\ell_m`$ -- chord length through obstacles of material $`m`$. Untagged obstacles are the `generic` material. With `enable_material_attenuation: true`, an obstacle is tagged by a `__material_<name>` suffix on its link or model name, e.g. `wall_1__material_concrete`; the known materials are in the [Material table](#material-table), and an unknown name logs a warning once and counts as `generic`.
- $`\ell_{\text{body}}`$ -- chord length through the vehicles' `base_link`s.

## Link budget

```math
L = \underbrace{10\, n \log_{10} \max(d, 1\,\mathrm{m})}_{L_d} \;+\; \underbrace{\sum_{m} \alpha_m \ell_m \;+\; L_{\text{metal}}\, \mathbf{1}[\ell_{\text{metal}} > 0] \;+\; \min\big(\alpha_{\text{generic}}\, \ell_{\text{body}},\ L_{\text{body}}\big)}_{L_{\text{obs}}}
```

- $`L_d`$ is free-space path loss relative to 1 m, with path-loss exponent $`n`$ = `path_loss_exponent` (2 in free space).
- $`\alpha_m`$ is material $`m`$'s attenuation rate in dB/m at the carrier frequency $`f`$ = `carrier_frequency_ghz`. For ITU-R P.2040 building materials (concrete, plasterboard, wood, glass), with relative permittivity $`\varepsilon_r = a`$ and conductivity $`\sigma_c = c\, f^{d}`$ S/m from that recommendation's Table 3, $`\alpha = 1636\, \sigma_c / \sqrt{\varepsilon_r}`$; for the other dielectrics, $`\alpha = 90.96\, f\, \sqrt{\varepsilon_r}\, \tan\delta`$ from their loss tangent. Override any rate as `material_<name>_attenuation_db_per_m`.
- Metal isn't penetrated: the first path detours around it, costing $`L_{\text{metal}}`$ = `material_metal_detour_loss_db` once per link however many metal obstacles it crosses.
- The vehicles' own bodies use the generic rate, capped at $`L_{\text{body}}`$ = `body_shadow_max_loss_db` (a partial blockage by a chassis never kills the link on its own).

A link is **NLOS** if $`L_{\text{obs}} \ge`$ `nlos_min_loss_db` (3 dB by default -- about 10 cm of generic wall at 4 GHz), and **LOS** otherwise. The NLOS regimes follow from where the total loss falls in the budget, so they depend on distance as well as on the obstacle:

| Regime | Condition | Effect |
| --- | --- | --- |
| LOS | $`L_{\text{obs}} <`$ `nlos_min_loss_db` | distance only |
| Soft NLOS | NLOS and $`L < L_{\sigma 0}`$ | $`\sigma`$, $`p`$ unchanged; folded (positive) noise and material bias only |
| Hard NLOS | NLOS and $`L_{\sigma 0} \le L < L_{\max}`$ | $`\sigma`$ grows, and $`p`$ too with `enable_nlos_dropout: true` |
| Blackout | $`L \ge L_{\max}`$ | no measurement (with `enable_nlos_dropout: false`, only when distance alone reaches $`L_{\max}`$) |

At the 3.7 m separation of the paper's NLOS experiment, generic walls of 0.25 m, 0.75 m and 1.2 m fall in Soft NLOS, Hard NLOS and Blackout respectively, as in that experiment; the same 0.25 m wall at 20 m is already Hard NLOS.

## Noise standard deviation and dropout probability

```math
\sigma = \sigma_{\min} \left(\frac{\sigma_{\max}}{\sigma_{\min}}\right)^{\mathrm{clamp}\left(\frac{L - L_{\sigma 0}}{L_{\sigma 1} - L_{\sigma 0}},\ 0,\ 1\right)}
\qquad
p =
\begin{cases}
1 & L \ge L_{\max} \\
1 - (1 - p_{\min}) \left(1 - \dfrac{s(L)}{s(L_{\max})}\right) & \text{otherwise}
\end{cases}
\qquad
s(L) = \frac{1}{1 + e^{-(L - L_c)/w}}
```

- $`\sigma`$ stays at $`\sigma_{\min}`$ = `min_noise_stddev_cm` up to $`L_{\sigma 0}`$ = `noise_start_loss_db`, then grows exponentially in dB (a power law in distance) to $`\sigma_{\max}`$ = `max_noise_stddev_cm` at $`L_{\sigma 1}`$ = `noise_end_loss_db`.
- $`p`$ has two independent causes, and a measurement survives only if it escapes both, which gives the $`(1 - p_{\min})(1 - \dots)`$ product:
  - **Random packet loss:** a fixed probability $`p_{\min}`$ = `min_dropout_probability`, whatever the loss.
  - **Link failure:** as the budget runs out, the signal gets too weak to decode. This follows the S-shaped logistic curve $`s(L)`$: near 0 at low loss, rising steeply, then flattening towards 1. $`L_c`$ = `dropout_center_loss_db` sets *where* the rise happens: it's the centre of the S, where $`s`$ = 0.5. $`w`$ = `dropout_width_db` sets *how abrupt* it is: most of the rise happens within $`\pm 2w`$ of $`L_c`$ (about $`\pm 3`$ dB in the presets).
  - **Closing the link:** at $`L_{\max}`$ = `max_link_loss_db` the link is closed, but the S alone would only have reached 0.81 there. Dividing by $`s(L_{\max})`$ stretches it to reach exactly 1, so $`p`$ rises into blackout without a jump. The stretch also means $`p`$ isn't 0.5 at $`L_c`$: with the shipped values it's 0.63 at $`L_c`$ (35.5 dB, 60 m) and 0.5 at about 34.9 dB (56 m).
- With `enable_nlos_dropout: false`, dropout uses $`L_d`$ only (obstacles don't drop measurements); noise still uses the full $`L`$.

![Noise standard deviation and dropout probability against the link budget](images/link_budget_curves.png)

The curves use the `eliko_identification` values. In both presets $`L_{\sigma 1}`$ and $`L_{\max}`$ are the same, so $`\sigma`$ reaches $`\sigma_{\max}`$ just as the link closes.

## Moving outliers

While either end of a link moves (more than 1 cm between updates), that link independently reads long with probability $`p_o`$ = `moving_outlier_probability`, by an exponentially distributed excess of mean $`m_o`$ = `moving_outlier_mean_cm`. The excess is added to the noise $`\tilde n`$ defined below. A link with both ends still is never affected, because UTIL's static recordings have almost no such samples while its flights do. UTIL's anchors never move, so applying this to moving anchors is an extrapolation. Off by default ($`p_o = 0`$). This is an empirical term: it reproduces the error distribution, but the physical cause isn't established.

## Deterministic bias

Each link also carries a deterministic range bias $`\beta`$ from two sources:

```math
\beta = \underbrace{\sum_{m \neq \text{metal}} \ell_m \left(\sqrt{\varepsilon_{r,m}} - 1\right)}_{\text{dielectric delay}} + \underbrace{k_P \left(L - L_P\right)}_{\text{received power}}
```

- **Dielectric delay** -- a dielectric slows the signal to $`c/\sqrt{\varepsilon_r}`$ ($`\varepsilon_{r,m}`$ = `material_<name>_permittivity`). **Off by default** (`enable_dielectric_delay: false`): it holds for solid slabs, but on UTIL's NLOS data, where the obstacles are mostly-hollow objects represented as solid boxes, it over-predicts their shifts. `generic` has $`\varepsilon_r = 1`$ (no delay) even when enabled, since an untagged obstacle's real material is unknown.
- **Received power** -- this is also how metal shifts a measurement: it has no bias of its own, because on UTIL's NLOS data the detour loss's power bias and NLOS folding already reproduce metal's shift (-34.2 cm predicted vs -35.2 cm real). The DW1000's timestamp has a bias that varies with received signal level, weaker signals reading longer (Decawave application note APS011, Section 3 and Table 2). It's modelled as linear in the loss, $`k_P`$ = `power_bias_cm_per_db` per dB relative to $`L_P`$ = `power_bias_ref_loss_db`; in TDOA, $`L_P`$ cancels between the two legs and only $`k_P`$ matters. Off by default; `util_identification` fits $`k_P`$ from UTIL's LOS distance test. APS011's curve saturates, so the linear form holds only over the fitted range (1-6.5 m).

## TWR measurement

For anchor $`j`$ and tag $`t`$, the ground truth $`d_{jt}`$ is always published. With probability $`p`$ no simulated range is published that cycle; otherwise

```math
\tilde r_{jt} = \max\left(0,\ d_{jt} + \tilde n + \beta + b_{jt}\right), \qquad
n \sim \mathcal{N}(\mu, \sigma^2), \qquad
\tilde n =
\begin{cases}
-n & \text{NLOS and } n < 0 \\
n & \text{otherwise}
\end{cases}
```

- $`\mu`$ = `gaussian_noise_mean_cm`.
- $`b_{jt} \sim \mathcal{U}(b_{\min}, b_{\max})`$, with $`b_{\min}, b_{\max}`$ = `bias_min_cm`, `bias_max_cm`, is drawn once per pair and kept for the whole run when `apply_pair_bias` is on (0 otherwise).
- Folding $`n`$ in NLOS models the first path arriving late, never early: for $`\mu = 0`$ it adds a mean of $`\sigma\sqrt{2/\pi} \approx 0.8\sigma`$ on top of the deterministic $`\beta`$.

## TDOA measurement

The reference anchor $`i`$ is the one with the lowest numeric id. Each leg $`k`$ (tag to anchor $`k`$) is evaluated as a link of its own, with error

```math
e_k = \tilde n_k + \beta_k + b_k, \qquad b_k \sim \mathcal{U}(b_{\min}, b_{\max}) \text{ once per anchor}
```

($`\tilde n_k`$ folded in NLOS as above). For every other anchor $`j`$, the ground truth (Zhao et al. 2023, Eq. 1) and the measurement are

```math
d_{ij} = \lVert \mathbf{p}_t - \mathbf{a}_j \rVert - \lVert \mathbf{p}_t - \mathbf{a}_i \rVert, \qquad
\tilde d_{ij} = d_{ij} + e_j - e_i .
```

- **The reference leg is shared.** $`e_i`$ is drawn once per tag per cycle and enters every $`\tilde d_{ij}`$ of that tag. If the reference leg drops out (probability $`p_i`$), all of that tag's TDOA values are lost that cycle; otherwise each $`\tilde d_{ij}`$ is additionally lost with probability $`p_j`$. Marginally, $`P(\text{lost}) = 1 - (1 - p_i)(1 - p_j)`$.
- **Errors are correlated across pairs**, as on real TDOA hardware: $`\mathrm{Cov}(\tilde d_{ij}, \tilde d_{ik}) = \mathrm{Var}(e_i)`$, a correlation of 0.5 when all legs have equal $`\sigma`$.
- **NLOS errors keep their sign.** NLOS on anchor $`j`$'s leg pushes $`\tilde d_{ij}`$ up; NLOS on the reference leg pulls *every* $`\tilde d_{ij}`$ of that tag down.
- $`\mu`$ and the tag's own antenna delay cancel in the difference, which is why the persistent bias is per anchor ($`b_j - b_i`$), not per pair.
- For LOS legs, $`\mathrm{Var}(\tilde d_{ij}) = \sigma_i^2 + \sigma_j^2`$, which is why `util_identification` derives its per-link $`\sigma`$ as the measured TDOA spread divided by $`\sqrt{2}`$. Both modes share the same per-link model, so switching modes needs no recalibration.

## Material table

Attenuation rates $`\alpha`$ (dB/m) at DW1000 channels 2 and 5, and relative permittivity:

| Material | $`\varepsilon_r`$ | $`\alpha`$, 4 GHz | $`\alpha`$, 6.5 GHz | Source |
| --- | ---: | ---: | ---: | --- |
| `generic` (untagged) | 1.00 | 31.0 | 48.8 | plasterboard rate, no excess delay |
| `plasterboard` | 2.73 | 31.0 | 48.8 | ITU-R P.2040 |
| `concrete` | 5.24 | 97.7 | 142.8 | ITU-R P.2040 |
| `wood` | 1.99 | 24.1 | 40.5 | ITU-R P.2040 |
| `glass` | 6.31 | 15.0 | 28.8 | ITU-R P.2040 |
| `plastic` | 2.50 | 5.8 | 9.3 | $`\tan\delta = 0.01`$ |
| `cardboard` | 1.20 | 1.2 | 1.9 | $`\tan\delta = 0.003`$ (mostly air) |
| `foam` | 1.05 | 0.2 | 0.3 | $`\tan\delta = 0.0005`$ |
| `metal` | -- | -- | -- | detour: 20 dB loss |

The plastic, cardboard and foam loss tangents and the 20 dB metal detour loss aren't from a standard: they're assumptions, tested on UTIL's NLOS objects and flights. Some resulting links with `eliko_identification`'s noise and dropout curves, with `enable_nlos_dropout: true` to show the full budget (no deterministic bias, since that preset has no power bias and the delay and detour bias are off):

| Link | $`L_{\text{obs}}`$ (dB) | $`L`$ (dB) | Regime | $`\sigma`$ (cm) | $`p`$ |
| --- | ---: | ---: | --- | ---: | ---: |
| LOS, 5 m / 40 m / 70 m | 0 | 14.0 / 32.0 / 36.9 | LOS | 12.0 / 28.7 / 47.0 | 0.020 / 0.112 / 0.908 |
| generic 0.25 m, 3.7 m | 7.7 | 19.1 | Soft NLOS | 12.0 | 0.020 |
| generic 0.25 m, 20 m | 7.7 | 33.8 | Hard NLOS | 34.1 | 0.298 |
| generic 0.5 m, 3.7 m | 15.5 | 26.8 | Hard NLOS | 16.9 | 0.022 |
| generic 0.75 m, 3.7 m | 23.2 | 34.6 | Hard NLOS | 37.1 | 0.439 |
| generic 1.0 m, 3.7 m | 31.0 | 42.4 | Blackout | -- | 1 |
| foam / cardboard 0.3 m, glass 6 mm, 5 m | < 1 | 14.0-14.3 | LOS | 12.0 | 0.020 |
| plastic 0.3 m, 5 m | 1.7 | 15.7 | LOS | 12.0 | 0.020 |
| wood 0.3 m, 5 m | 7.2 | 21.2 | Soft NLOS | 12.0 | 0.020 |
| concrete 0.2 m, 5 m | 19.5 | 33.5 | Hard NLOS | 33.3 | 0.255 |
| metal, 5 m | 20.0 | 34.0 | Hard NLOS | 34.9 | 0.330 |

The same wall matters more the farther apart the robots are, since it eats into a budget distance has already spent (compare the two 0.25 m rows). With `enable_nlos_dropout: false`, as in `util_identification`, $`\sigma`$ is as shown, but $`p`$ follows distance alone, so an obstacle never causes a blackout. Where $`L \ge L_{\max}`$ from obstacles, $`\sigma`$ stays at $`\sigma_{\max}`$.

## Limitations

- Box collisions are intersected exactly, including rotated ones; any other collision shape (meshes, cylinders, ...) falls back to its link's world-axis-aligned bounding box, which over-estimates the blocked length.
- One straight ray per link: no explicit multipath, and no diffraction around obstacle edges (the metal detour loss is a constant, not computed from geometry). The moving outliers are a statistical stand-in, the same for every anchor and uncorrelated in time.
- Overlapping obstacles of *different* materials each count their own chord length.
- The outlier-heavy behaviour UTIL shows behind some plastic and cabinet obstacles (mostly accurate readings with occasional large misses) isn't modelled.
- The TDOA reference anchor is fixed (lowest id), and clock synchronisation between anchors isn't simulated.

## Parameter reference

| Parameter | Symbol or role | Default |
| --- | --- | --- |
| **Ranging mode and topics** | | |
| `uwb_mode` | `twr` or `tdoa` | `twr` |
| `topic`, `ground_truth_topic` | TWR measurements, ground truth | `/uwb_gz_simulator/distances`, `/uwb_gz_simulator/distances_ground_truth` |
| `tdoa_topic`, `tdoa_ground_truth_topic` | TDOA measurements, ground truth | `/uwb_gz_simulator/tdoa`, `/uwb_gz_simulator/tdoa_ground_truth` |
| **Link budget** | | |
| `path_loss_exponent` | $`n`$ | `2.0` |
| **Noise** | | |
| `min_noise_stddev_cm`, `max_noise_stddev_cm` | $`\sigma_{\min}`$, $`\sigma_{\max}`$ | `12.0`, `50.0` |
| `noise_start_loss_db`, `noise_end_loss_db` | $`L_{\sigma 0}`$, $`L_{\sigma 1}`$ | `23.5`, `37.5` |
| `gaussian_noise_mean_cm` | $`\mu`$ | `0.0` |
| `moving_outlier_probability`, `moving_outlier_mean_cm` | $`p_o`$, $`m_o`$ | `0.0`, `30.0` |
| **Dropout** | | |
| `max_link_loss_db` | $`L_{\max}`$ | `37.5` |
| `min_dropout_probability` | $`p_{\min}`$ | `0.02` |
| `dropout_center_loss_db`, `dropout_width_db` | $`L_c`$, $`w`$ | `35.5`, `1.4` |
| `enable_nlos_dropout` | obstacles affect dropout | `true` |
| **Obstacles** | | |
| `nlos_endpoint_margin_m` | ignored length at each segment end | `0.02` |
| `nlos_min_loss_db` | NLOS threshold on $`L_{\text{obs}}`$ | `3.0` |
| `body_shadow_max_loss_db` | $`L_{\text{body}}`$ | `10.0` |
| **Materials** | | |
| `enable_material_attenuation` | `__material_<name>` tags on/off | `false` |
| `carrier_frequency_ghz` | $`f`$ (DW1000 channel 2) | `4.0` |
| `material_<name>_attenuation_db_per_m` | $`\alpha_m`$ override (any material but `metal`) | from $`f`$, see [Material table](#material-table) |
| `material_<name>_permittivity` | $`\varepsilon_{r,m}`$ override (any material but `metal`) | see [Material table](#material-table) |
| `material_metal_detour_loss_db` | $`L_{\text{metal}}`$ | `20.0` |
| `enable_dielectric_delay` | dielectric-delay term on/off | `false` |
| **Persistent bias** | | |
| `apply_pair_bias` | persistent bias on/off | `false` |
| `bias_min_cm`, `bias_max_cm` | $`b_{\min}`$, $`b_{\max}`$ | `-27.0`, `20.0` |
| `power_bias_cm_per_db`, `power_bias_ref_loss_db` | $`k_P`$, $`L_P`$ | `0.0`, `0.0` |
| `random_seed` | noise/bias seed; `0` = fixed default seed | `0` |

The per-material overrides aren't shown in the setup GUI (saving from it would freeze the frequency-derived rates); add them to a preset by hand.

## Calibration

### Eliko preset (`eliko_identification`)

The Eliko preset was measured in LOS at short range (σ ≈ 12 cm, near-zero bias, robots < 10 m apart) and completed with the datasheet's 75 m maximum range and ~50 cm noise near it (see the Robot Meets Ranging workshop paper, Section II-B and Table II). With $`n = 2`$, 15 m and 75 m are 23.5 dB and 37.5 dB, so the defaults reproduce those points exactly, and the link budget sets the shape in between:

| $`d`$ (m) | 5 | 10 | 15 | 30 | 45 | 60 | 70 | > 75 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| $`\sigma`$ (cm) | 12.0 | 12.0 | 12.0 | 22.2 | 31.8 | 41.0 | 47.0 | -- |
| $`p`$ | 0.020 | 0.020 | 0.020 | 0.037 | 0.201 | 0.641 | 0.908 | 1 |

$`L_c`$ and $`w`$ are a least-squares fit over 10-70 m to the published calibration's dropout, which rises from 2% at 10 m to 90% at 70 m (rms error 0.04, before the scaling by $`s(L_{\max})`$). The dataset doesn't cover NLOS. In the absence of an Eliko NLOS calibration, the preset uses the same obstacle and material parameters as `util_identification`. Its other parameters stay Eliko's, including no received-power bias and a 12 cm noise floor, so an obstacle's effect differs from UTIL's even with identical material values. Treat Eliko NLOS results as uncalibrated.

### UTIL preset (`util_identification`)

Calibrated and validated against the [UTIL](https://utiasdsl.github.io/util-uwb-dataset/) dataset [Zhao et al.]: static identification recordings first, then flights through the full pipeline (PX4 SITL, Gazebo, ROS 2, offboard control).

| Parameter | Value | Origin |
| --- | --- | --- |
| `min_noise_stddev_cm` | 4.0 | UTIL's LOS TDOA spread divided by $`\sqrt 2`$ |
| `power_bias_cm_per_db` | 0.98 | Fitted on the LOS distance test ($`\pm`$ 0.18 cm/dB) |
| `bias_min_cm`, `bias_max_cm` | -3.6, 3.6 | Scatter of the distance test's per-setup means |
| `material_metal_detour_loss_db` | 20 | Assumed; reproduces the static metal shift |
| `enable_dielectric_delay` | `false` | Over-predicted the static NLOS shifts |
| `enable_nlos_dropout` | `false` | Metal-blocked pairs keep 78-99 % of the unblocked sample rate in UTIL's flights |
| `moving_outlier_probability`, `moving_outlier_mean_cm` | 0.05, 25 | Fitted on the flights; checked on held-out flights |

#### Summary

Results compare real and simulated TDOA errors with the Wasserstein-1 distance (W1, in cm; lower is better). *Centred* removes each setup's or anchor pair's own offset, leaving the noise shape.

| Data | W1 raw | W1 centred |
| --- | ---: | ---: |
| Static, LOS | 2.2-5.8 | 0.6-1.2 |
| Static, obstacle on the tag link | 2.1-9.5 | -- |
| Flight, LOS | 3.07 | 1.75 |
| Flight, NLOS | 7.89 | 11.38 |

The simulation reproduces noise levels, the received-power bias in LOS, and the size of obstacle effects in static data. Flight data is harder: LOS matches well, while NLOS matches in shape but less well in size.

#### Static data

In LOS, the noise matches at every distance, at about 5-6 cm. The simulated means also follow the real drift with distance, from about +8 cm at 1 m to about -6 cm at 6.5 m. Individual setups aren't comparable, because the simulation draws random anchor offsets and the real means jump between neighbouring positions for reasons not established.

![Static LOS spread against distance](images/static_spread_vs_distance.png)

With an obstacle on the anchor-tag line (*anTag* in the figure), the simulation reproduces how much each material shifts the error, relative to LOS (cm):

| Obstacle | Shift, real / sim | W1 |
| --- | --- | ---: |
| Cardboard | +0.5 / -5.2 | 6.12 |
| Foam | -0.4 / -1.8 | 2.06 |
| Plastic | -3.5 / -8.5 | 5.42 |
| Wooden cabinet | -9.4 / -14.6 | 9.46 |
| Wooden shelf | -16.8 / -16.2 | 3.27 |
| Metal | -35.2 / -35.3 | 6.88 |

![Static NLOS shift by material](images/static_material_shift.png)

- **Heavier obstacles shift the error more**, in reality and in the simulation. Light materials are over-predicted by a few cm.
- **Obstacle effects vary with placement more than from sample to sample.** Moving the same metal obstacle changes the real shift by about 15 cm, while the spread at any one placement stays near 7 cm, against 5 cm in LOS. The simulated shift varies less between placements.
- **An obstacle between the anchors barely matters.** The real error stays within 1 cm of LOS, except for plastic at 3.5 cm. The simulation doesn't model this link.

#### Flight data

Flights reproduce UTIL's paths, not its timing: the UAV covered 89-100 % of each path with a median tracking error of 3-7 cm. Anchors were carried by `static: true` robots at their surveyed positions. About 0.7 % of the real samples are corrupted, with errors of up to 120 m, and were excluded. Repeating flights with the same seed moves W1 by about 0.3 cm.

**LOS.** The noise shape and the received-power bias both carry over from the static data.

![LOS flights: noise shape and received-power bias](images/flights_los.png)

*In both flight figures, the "pair mean" is the median error of an anchor pair; slope points use samples within 30 cm of it.*

- **The received-power bias holds:** 0.77 cm/dB real against 0.94 simulated.
- **Moving tags have outliers that static ones don't.** Errors over 30 cm from the pair's median are almost absent in static data and make up 1-10 % of LOS flight data. The moving outlier term gives a uniform 3 %. It improved W1 from 5.20 to 3.07 cm raw, and by 1.3-2.1 cm on flights not used to fit it.
- **Some anchors are much noisier than others in flight.** The cause isn't known, and the simulation treats all anchors alike, so its noise is too low for the worst ones.
- **Per-anchor offsets are larger in flight data:** about 4.5 cm against 2.1 cm simulated. Widening them fits flights better but worsens the static fit, so the static value was kept.

**NLOS.** With one metal and three wooden boxes:

| Link | Robust std, real / sim | Outliers, real / sim |
| --- | --- | --- |
| Unblocked | 13.3 / 7.3 cm | 10.4 / 3.0 % |
| Wood | 12.2 / 6.7 cm | 8.6 / 2.9 % |
| Metal | 34.7 / 29.3 cm | 41.9 / 27.8 % |

![NLOS flights with the final calibration](images/flights_nlos.png)

- **Metal is right in sign and most of its scatter, but too large.** A blocked leg reads 28 cm long in reality and 58 cm in the simulation, because simulated noise grows steeply with total loss. Reducing that effect also removed the scatter and worsened the overall fit, so it was rejected.
- **Obstacles don't stop measurements.** Blocked links keep 78-99 % of their sample rate, as the simulation does.
- **Wood adds bias but no measurable scatter**, in reality and in the simulation.
- **The received-power bias disappears in this data:** 0.04 cm/dB real against 0.98 simulated. Blocked legs lose it, because their error no longer follows distance, and the simulation still applies it. Part of the loss also appears without these obstacles, for reasons not known.
- **Overall noise is about twice the simulation's**, even on unblocked links.
- **Obstacle positions are approximate.** UTIL didn't survey them, so the simulated metal effect also depends on where they were assumed to be.

## References

- Decawave, *APS011 Application Note: Sources of Error in DW1000 Based Two-Way Ranging (TWR) Schemes*, v1.0, 2014.
- ITU-R Recommendation P.2040-3, *Effects of building materials and structures on radiowave propagation above about 100 MHz*.
- W. Zhao, A. Goudar, X. Qiao, A. P. Schoellig, "UTIL: An Ultra-Wideband Time-Difference-of-Arrival Indoor Localization Dataset", IJRR, 2024. Data: https://github.com/utiasDSL/util-uwb-dataset
- A. Martínez-Silva, D. Alejo, L. Merino, F. Caballero, "UWBPX4Sim: Ultra-Wideband simulator for multi-robot applications", Robot Meets Ranging workshop, ICRA 2026.
