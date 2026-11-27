# Third-party notices — FANUC CRX-5iA with laser head USD asset

This directory contains license texts and provenance information for third-party
source assets used to construct the USD package in its parent directory.

## FANUC CRX-5iA source material

- Upstream repository (official FANUC ROS 2 robot descriptions):
  https://github.com/FANUC-CORPORATION/fanuc_description
- Package: `fanuc_crx_description`, version 2.1.0
- Source files: the `crx5ia` model, i.e. its URDF/xacro description, the
  visual meshes `meshes/crx5ia/visual/*.dae` and the collision meshes
  `meshes/crx5ia/collision/*.stl`
- Copyright: 2025-2026 FANUC America Corporation, 2025-2026 FANUC CORPORATION
- License: Apache-2.0
- Local license text:
  `FANUC-fanuc_description-Apache-2.0-LICENSE` (copy of the upstream
  `LICENSES/Apache-2.0.txt`)
- Local derived file: `crx5ia_laser_head.usd`
- Local modifications:
  Converted from URDF to USD and flattened into a single layer, with a laser
  head on `ee_link` (emitter frame 0.18 m along `fanuc_flange` +Z) and joint
  drive settings for the laser-cleaning cell. No endorsement by FANUC is implied.
