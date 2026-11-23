# PIH-MJFit 0.05 mm Clearance Peg-in-Hole Models

Standalone MuJoCo MJCF and Wavefront OBJ assets for the 6 peg-and-hole geometries used in PIH-MJFit at **0.05 mm** per-side radial/edge clearance (`clearance = 5e-5 m`).

## Specifications

| Parameter | Value | Notes |
|---|---|---|
| Per-side clearance | `0.05 mm` (`0.00005 m`) | Exact edge-to-edge offset via `mjfit.peg(..., clearance=5e-5)` |
| Nominal cross-section size | `35 mm` (`0.035 m`) | See shape table below |
| Peg (shaft) length | `50 mm` (`0.050 m`) | Convex prism (`peg_0.obj` + `peg_visual.obj`) |
| Plate thickness (through-bore depth) | `40 mm` (`0.040 m`) | Radial convex-wedge decomposition around bore |
| Plate outer footprint | `110 mm × 110 mm` (`0.11 m`) | Centered at `(0, 0)` |
| Curve approximation `max_error` | `0.005 mm` (`5e-6 m`) | `min(clearance / 10, 0.05 mm)` |
| Sliding friction | `0.45` | `friction="0.45 0.01 0.001"`, `condim="4"` |
| Auxiliary end-face points | `64` | Stored in `geometry.json` (`auxiliary_points_xy_m`) |

> Note: In the PIH-MJFit full robot simulation, `pih_mjfit/scene.py` overrides `solref` on the peg/plate geoms to `"0.015 1.5"` for the heavier Estun iER15-1430-MI arm; the standalone `model.xml` files here use `mjfit` stiff contact defaults (`solref="0.002 1.0"`, `solimp="0.99 0.999 0.001 0.5 2.0"`).

## Included Shapes (6)

| Shape | Bore Size (X × Y mm) | Peg Size (X × Y mm) | Symmetry (`N`-fold) | Bore Vertices | Plate Collision OBJs |
|---|---:|---:|---:|---:|---:|
| `circle` | `35.010 × 35.010` | `34.910 × 34.910` | `0` (axisymmetric) | 132 | 132 |
| `square` | `35.000 × 35.000` | `34.900 × 34.900` | `4` | 4 | 4 |
| `hexagon` | `35.000 × 40.415` | `34.900 × 40.299` | `6` | 6 | 6 |
| `rectangle` | `35.000 × 24.500` | `34.900 × 24.400` | `2` | 4 | 4 |
| `stadium` | `35.010 × 22.750` | `34.910 × 22.650` | `2` | 106 | 106 |
| `d` | `25.205 × 35.010` | `25.105 × 34.910` | `1` | 87 | 87 |

## Directory Structure

```text
.
├── README.md
├── manifest.json
└── <shape>/
    ├── model.xml        # Standalone MuJoCo MJCF (plate + free-joint peg + sites)
    ├── geometry.json    # Exact 2D bore/peg outlines, 64 end-face points, bounding boxes
    └── meshes/
        ├── plate_visual.obj   # Single watertight visual mesh of the bored plate
        ├── plate_0.obj ...    # Convex collision prisms decomposing the plate
        ├── peg_visual.obj     # Single watertight visual mesh of the peg
        └── peg_0.obj          # Convex collision prism of the peg
```

### Sites & Frames in `model.xml`
- `plate` body center sits at `(0, 0, 0.02)` (plate bottom at `z = 0`, top mouth at `z = 0.04`).
  - `bore_mouth` site: `(0, 0, +0.02)` in `plate` frame (bore mouth center).
- `peg` body starts at `(0, 0, 0.07)` (5 mm above the plate mouth) with `freejoint="peg_joint"`.
  - `peg_tip` site: `(0, 0, -0.025)` in `peg` frame (center of peg bottom face).

## Quick Verification in MuJoCo

```python
import mujoco

model = mujoco.MjModel.from_xml_path("hexagon/model.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
```

## Source
Exported from [`Wangmili/PIH-MJFit`](https://github.com/Wangmili/PIH-MJFit) using `pih_mjfit.scene.Recipe(..., clearance=0.00005)` and `mjfit.peg(...)`.
