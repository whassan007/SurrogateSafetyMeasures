# SurrogateSafetyMeasures

> Quantifiable metrics for evaluating the relative safety of traffic facilities by analyzing observable **conflict events** (near-misses) rather than relying solely on historical crash data.

---

## Overview

Traditional traffic safety analysis depends on crash records — data that is inherently rare, randomly distributed, often underreported, and available only after harm has already occurred. **Surrogate Safety Measures (SSMs)** offer a proactive alternative: by systematically measuring near-miss interactions between road users, engineers and researchers can assess risk *before* crashes happen.

This repository provides tools and implementations for computing, analyzing, and visualizing SSMs from traffic observation data, supporting safety assessments at intersections, pedestrian crossings, road segments, and other traffic facilities.

---

## Background

The surrogate safety paradigm is rooted in Hydén's **Safety Hierarchy** (1987), which describes a continuum of road-user interactions — from smooth, uneventful passages down through conflicts, near-misses, and ultimately collisions. The key insight is that near-miss events are far more frequent than crashes and carry statistically meaningful safety signals.

A **traffic conflict** is defined as an observable interaction between two or more road users that is close enough in time and space that a collision *would* occur if their movements remained unchanged. By quantifying the severity and frequency of these conflict events, SSMs make it possible to:

- Identify hazardous locations before crashes accumulate
- Evaluate the safety impact of infrastructure designs *before construction*
- Compare alternative traffic management strategies
- Monitor safety changes over time using consistent, repeatable metrics

---

## Key Metrics

### Time-Based Measures

| Metric | Abbreviation | Description |
|--------|-------------|-------------|
| **Time to Collision** | TTC | Time remaining until a collision would occur if the current course and speed differential are maintained. Lower TTC → higher severity. |
| **Post-Encroachment Time** | PET | Time difference between when an offending road user *leaves* a potential collision zone and when the right-of-way road user *enters* it. Captures near-misses without evasive action. |
| **Time Advantage** | TA | Difference between TTC and the time needed to clear the conflict zone; positive values indicate safety margin. |
| **Gap Acceptance** | GA | Time gap accepted by a road user when entering a stream of traffic; critical for unsignalized intersections. |

### Kinematic Measures

| Metric | Abbreviation | Description |
|--------|-------------|-------------|
| **Deceleration Rate to Avoid Crash** | DRAC | Required deceleration to avoid a collision; compared to the maximum available deceleration. |
| **Crash Potential Index** | CPI | Probability that required deceleration (DRAC) exceeds available deceleration (MADR). |
| **Delta-V** | ΔV | Change in velocity at the moment of a hypothetical impact; correlates with injury severity. |
| **Conflicting Speed of Vehicles** | CSV | Speed of the conflicting vehicle at the moment of the conflict event. |

### Spatial Measures

| Metric | Abbreviation | Description |
|--------|-------------|-------------|
| **Encroachment Time** | ET | Duration for which one road user occupies the path of another. |
| **Proportion of Stopping Distance** | PSD | Ratio of available stopping distance to the distance required to avoid a collision. |

---

## Conflict Event Types

Conflict events are classified by the type of interaction between road users:

- **Rear-end conflicts** — A following vehicle approaches a leading vehicle faster than it can stop.
- **Lane-change conflicts** — A vehicle moves laterally into the path of an adjacent road user.
- **Crossing conflicts** — Road users' paths cross, such as at intersections or pedestrian crossings.
- **Merging conflicts** — Road users converge from different lanes toward a single lane.
- **Head-on conflicts** — Opposing road users approach each other on a shared path.

---

## Methodology

The **Surrogate Safety Assessment Methodology (SSAM)** follows a structured pipeline:

```
Raw Trajectory Data
        │
        ▼
Conflict Detection  ←─── Define conflict threshold (e.g., TTC < 1.5s)
        │
        ▼
SSM Computation     ←─── TTC, PET, DRAC, CPI, ΔV, etc.
        │
        ▼
Conflict Classification  ←─── By type, severity, road-user class
        │
        ▼
Aggregation & Analysis   ←─── Frequency, severity distribution, hotspot maps
        │
        ▼
Safety Assessment        ←─── Compare to benchmarks or alternative designs
```

### Data Sources

SSMs can be derived from multiple data collection approaches:

- **Video-based observation** — Fixed cameras or UAV/drone footage processed with computer vision
- **Traffic simulation** — Microsimulation outputs (VISSIM, SUMO, AIMSUN) with trajectory export
- **GPS / probe vehicle data** — Onboard sensors and fleet telematics
- **Roadside sensors** — Radar, LiDAR, or loop detector arrays

---

## Features

- Computation of core SSMs: TTC, PET, DRAC, CPI, ΔV
- Support for vehicle–vehicle and vehicle–pedestrian interactions
- Conflict event detection with configurable severity thresholds
- Conflict classification by type (rear-end, crossing, merging, etc.)
- Statistical summaries and severity distribution analysis
- Visualization: time-space diagrams, conflict maps, severity histograms

---

## Getting Started

### Prerequisites

```bash
Python >= 3.9
numpy
pandas
matplotlib
scipy
```

### Installation

```bash
git clone https://github.com/whassan007/SurrogateSafetyMeasures.git
cd SurrogateSafetyMeasures
pip install -r requirements.txt
```

### Basic Usage

```python
from ssm import compute_ttc, compute_pet, detect_conflicts

# Load trajectory data (columns: time, vehicle_id, x, y, speed, heading)
import pandas as pd
trajectories = pd.read_csv("data/trajectories.csv")

# Detect conflicts
conflicts = detect_conflicts(trajectories, ttc_threshold=1.5)

# Compute SSMs for each conflict
conflicts["TTC"]  = compute_ttc(conflicts)
conflicts["PET"]  = compute_pet(conflicts)

print(conflicts[["conflict_type", "TTC", "PET"]].describe())
```

---

## Applications

SSMs are applicable across a wide range of traffic safety contexts:

- **Intersection safety assessment** — Signalized and unsignalized, urban and rural
- **Pedestrian and cyclist safety** — Active transport conflict analysis at crossings and shared paths
- **Freeway / motorway analysis** — Merging zones, weave sections, lane drops
- **Before-after studies** — Quantifying safety impact of infrastructure changes
- **Proactive safety monitoring** — Real-time conflict detection for connected intersections
- **Simulation-based design evaluation** — Testing designs that don't yet exist in the physical world

---

## References

- Hydén, C. (1987). *The Development of a Method for Traffic Safety Evaluation: The Swedish Traffic Conflicts Technique.* Lund University.
- Hayward, J.C. (1972). *Near-miss determination through use of a scale of danger.* Highway Research Record, 384.
- Gettman, D. & Head, L. (2003). [Surrogate Safety Measures from Traffic Simulation Models.](https://journals.sagepub.com/doi/10.3141/1840-12) *Transportation Research Record*, 1840.
- FHWA (2010). [Surrogate Safety Measures From Traffic Simulation Models — Final Report.](https://www.fhwa.dot.gov/publications/research/safety/03050/) FHWA-RD-03-050.
- Tarko, A.P. (2018). [Surrogate Measures of Safety.](https://ictct.net/wp-content/uploads/SMoS_Library/LIB_Tarko_2018.pdf) Chapter 17, *Safe Mobility: Challenges, Methodology and Solutions.*
- Chen, P. et al. (2017). [Surrogate Safety Analysis of Pedestrian–Vehicle Conflict at Intersections Using UAV Videos.](https://onlinelibrary.wiley.com/doi/10.1155/2017/5202150) *Journal of Advanced Transportation.*
- Zheng, L. et al. (2022). [Learning the representation of surrogate safety measures to identify traffic conflict.](https://www.sciencedirect.com/science/article/abs/pii/S0001457522001919) *Accident Analysis & Prevention.*

---

## License

This project is licensed under the **GNU Affero General Public License v3.0 (AGPL-3.0)**. See the [LICENSE](LICENSE) file for details.

---

## Contributing

Contributions are welcome. Please open an issue to discuss proposed changes before submitting a pull request. For major additions (new metrics, new data format support), include a reference to the relevant literature.

---

## Author

**Waël Hassan** — [github.com/whassan007](https://github.com/whassan007)
