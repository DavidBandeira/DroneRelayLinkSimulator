# UAV Relay Link Simulator

MATLAB simulation framework for evaluating UAV-based airborne communication relays in emergency scenarios. Developed as part of a final aerospace engineering project at the Universidade de Aveiro.

> **Associated report:** *Validation of the Feasibility of an Aerial Relay System with Drones for Restoring Communications in Emergency Areas* — David Dinis Bandeira, 2025.

---

## Overview

This simulator models the downlink performance of a fixed-wing UAV operating as a 5G NR or Wi-Fi 6 airborne relay, including:

- Phased array antenna gain with bank-induced boresight rotation
- Circular loiter trajectory with coordinated-turn attitude model
- Link budget, SNR, Shannon capacity, and MCS-based throughput estimation
- Terrain-aware propagation via the Longley–Rice model (GeoTIFF input)
- Coverage heatmap generation via MATLAB Antenna Toolbox Site Viewer
- Cinematic 3D video rendering of simulation results

---

## Repository Structure

```
.
├── uav_relay_simulator_v8.m      # Main simulation script
├── render_simulation_video_4.m   # Cinematic video renderer
├── merge_terrain_tiles.m         # MDT tile merger and GeoTIFF exporter
├── convert_tif_to_dted.m         # GeoTIFF → DTED1 converter for addCustomTerrain()
└── README.md
```

---

## Requirements

- MATLAB R2023b or later
- [Antenna Toolbox](https://www.mathworks.com/products/antenna.html)
- [Communications Toolbox](https://www.mathworks.com/products/communications.html)
- [Mapping Toolbox](https://www.mathworks.com/products/mapping.html) (for terrain modes)

---

## Quick Start

### 1. Basic simulation (no terrain data required)

Open `uav_relay_simulator_v8.m` and set:

```matlab
TERRAIN_MODE = 'siteviewer';
```

Then run the script. The simulator will use a fixed MSL altitude over Manteigas (Lat: 40.4017, Lon: −7.5433) with free-space baseline propagation.

### 2. Simulation with real terrain (GeoTIFF input)

Set:

```matlab
TERRAIN_MODE = 'tif';
```

Place your GeoTIFF DEM file (e.g. `pedrogao_terrain_wgs84.tif`) in the working directory. If you have raw MDT tiles (Portuguese IGP format `MDT-2m-XXXXXX-XX-XXXX_v01.tif`), use `merge_terrain_tiles.m` first to merge and export them.

The script will:
1. Convert the GeoTIFF to DTED1 format via `convert_tif_to_dted()`
2. Register the terrain with MATLAB's `addCustomTerrain()`
3. Set the UAV altitude automatically as a fixed AGL value above the terrain peak within the loiter radius

---

## Configuration

All key parameters are defined at the top of `uav_relay_simulator_v8.m`.

### Access link technology

```matlab
sim.accessTech = '5g';      % '5g' | 'wifi6'
```

### RF parameters

```matlab
cfg.fHz           = 3.8e9;   % Carrier frequency [Hz]
cfg.BHz           = 20e6;    % Bandwidth [Hz]
cfg.txPower_dBm   = 23;      % Transmit power [dBm]
cfg.NF_dB         = 7;       % Receiver noise figure [dB]
cfg.rxSens_dBm    = -100;    % Receiver sensitivity [dBm]
cfg.fadeMargin_dB = 10;      % Fade margin [dB]
```

### Trajectory

```matlab
gDT.type        = 'circle';
gDT.radius_m    = 1750;      % Loiter radius [m]
gDT.centerLat   = 40.4017;   % Centre latitude
gDT.centerLon   = -7.5433;   % Centre longitude
gDT.alt_m       = 2000;      % UAV altitude MSL [m] (siteviewer mode)
```

### Antenna arrays

Five phased arrays are configured by default, matching the conceptual design:

| Array | Function        | Size       | Az [°] | El [°] |
|-------|----------------|------------|--------|--------|
| 1     | Nadir (fore)   | 2×2        | 90     | −75    |
| 2     | Nadir (aft)    | 2×2        | 270    | −75    |
| 3     | Lateral centre | 4×4        | 180    | −20    |
| 4     | Lateral fwd    | 4×4        | 160    | −24    |
| 5     | Lateral aft    | 4×4        | 200    | −24    |

Azimuths follow the airframe convention: 0° = forward, 90° = starboard. Elevation is negative downward.

To use a different configuration, edit the `arrays_cfg` block:

```matlab
% Example: single nadir omnidirectional element
arrays_cfg = struct('type','preset','preset','omni','G_elem_dBi',2, ...
                    'boresight_az',0,'boresight_el',-90);
```

Supported antenna presets: `omni`, `dipole`, `patch`, `yagi`, `array`. A custom LUT (CSV with columns `[index, elevation_deg, gain_dBi]`) is also supported via `'type','lut'`.

---

## Outputs

Running the main script produces:

| Output | Description |
|--------|-------------|
| SNR per user plot | SNR over time for each ground user |
| Throughput plot | 5G/Wi-Fi 6 per-user and aggregate throughput |
| Array gain plot | Best panel TX gain per user over the orbit |
| Link margin plot | Margin above sensitivity threshold (0 dB line shown) |
| Link availability plot | Binary link status per user over time |
| Shannon capacity plot | Theoretical upper bound per user and aggregate |
| Distance plot | Slant range UAV–user over the orbit |
| Trajectory map | 2D geographic plot of UAV orbit and user positions |

Console output includes mean aggregate throughput, mean end-to-end throughput (backhaul-limited), and link availability percentage.

---

## Video Rendering

To generate a cinematic MP4 video of the simulation, call `render_simulation_video_4` after the main loop:

```matlab
render_simulation_video_4(simData, cfg, gDT, terrain_renderer, ...
    'OutputFile',  'uav_relay.mp4', ...
    'CameraMode',  'orbit_low', ...
    'NumLaps',     1, ...
    'ZExag',       2.5);
```

### Camera modes

| Mode | Description |
|------|-------------|
| `orbit_low` | Close orbit, wide FOV — recommended general view |
| `orbit` | Classic wider orbit |
| `dramatic_reveal` | Opens close to drone, pulls back to reveal terrain |
| `ground_human` | Ground-level perspective from a user's position |
| `fly_alongside` | Camera flies beside the drone |
| `follow` | Follows from behind |
| `top_down` | Zenith view, emphasises coverage heatmap |
| `cinematic_auto` | Automatic sequence cycling through modes |

To render all modes at once:

```matlab
render_all_views(simData, cfg, gDT, terrain_renderer, 'Prefix', 'uav_');
```

---

## Terrain Pipeline

If you have Portuguese IGP MDT-2m tiles:

1. Place all `.tif` files in a subfolder (e.g. `pedrogao/`)
2. Run `merge_terrain_tiles.m` — this merges tiles, reprojects to WGS84, and exports:
   - `pedrogao_terrain_light.mat` — subsampled renderer grid
   - `pedrogao_terrain_wgs84.tif` — GeoTIFF for `addCustomTerrain()`
3. Set `TERRAIN_MODE = 'tif'` in the main script

The `convert_tif_to_dted()` function handles the GeoTIFF → DTED1 binary conversion required by MATLAB's terrain engine, including header patching for the correct cell coordinates.

---

## Physical Layer Models

### 5G NR throughput

MCS selection follows 3GPP TS 38.214. Effective throughput is:

$$T_\text{eff} = \eta_\text{MCS} \cdot N_\text{PRB} \cdot N_\text{RE} \cdot (1 - \delta) \cdot N_\text{layers} \cdot f_s \cdot (1 - \text{BLER})$$

BLER is approximated by a logistic function of SNR distance to the MCS threshold.

### Wi-Fi 6 throughput

MCS tables from IEEE 802.11ax. Throughput scaled by bandwidth, guard interval, and spatial stream count.

### Antenna gain model

Array gain uses the raised-cosine element pattern with scan loss:

$$G(\psi) = G_\text{elem} \cdot N_x N_y \cdot \eta \cdot \cos^n\psi$$

where $\psi$ is the angle between the bank-corrected boresight and the user direction, and $n$ is derived from the element half-power beamwidth.

---

## Simulation Parameters — Default Concept Design

| Parameter | Value |
|-----------|-------|
| Carrier frequency | 3.8 GHz |
| Bandwidth | 20 MHz |
| Subcarrier spacing | 30 kHz |
| Transmit power | 23 dBm EIRP |
| Receiver noise figure | 7 dB |
| Fade margin | 10 dB |
| UAV altitude | 1 000 m AGL / 2 000 m MSL |
| Loiter radius | 1 750 m |
| Loiter speed | 165 km/h (simulation) / 108 km/h (design point) |
| Number of users | 4 |
| Backhaul capacity (UL) | 15 Mbps (Starlink Mini) |

---

## Licence

MIT — free to use and modify with attribution.

---

## Citation

If you use this simulator in academic work, please cite the associated report:

```
D. D. Bandeira, "Validation of the Feasibility of an Aerial Relay System
with Drones for Restoring Communications in Emergency Areas,"
Final Project Report, Universidade de Aveiro, 2025.
```