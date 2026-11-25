# Third-party notices — FANUC CRX-5iA with Robotiq 2F-140 USD asset

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
- Local derived files:
  `crx5ia.usd`, `crx5ia_base.usd`, `crx5ia_robot.usd`,
  `crx5ia_physics.usd`, `crx5ia_sensor.usd`,
  `crx5ia_ee_link_robotiq_2f_140.usd`
- Local modifications:
  Converted from URDF to USD, with composition references repaired for
  portable, repository-relative asset resolution and
  the laser head end effector replaced by a Robotiq 2F-140. No endorsement by
  FANUC is implied.

## Robotiq 2F-140 gripper

The gripper package lives in `../../grippers/robotiq_2f_140/`, shared with the
other gripper robots; its license text and notice are in that folder's
`LICENSES/`. This package only contains the robot-specific mounting layer.

## Scope

License applicability follows the provenance of each redistributed source asset.
This notice does not grant rights beyond the accompanying upstream licenses,
nor does it replace additional license terms attached to meshes, textures,
CAD data, materials, or NVIDIA assets.
