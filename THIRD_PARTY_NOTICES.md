# Third-Party Notices

## Scope of the root MIT License

The root MIT License applies only to original copyrightable material authored by Institut für Unternehmenskybernetik e.V. an der RWTH Aachen for this repository, except where a file or package states a different license.

It does not replace, modify, supersede, or grant rights beyond the license terms, copyright notices, attribution requirements, or redistribution conditions that apply to third-party assets included in this repository.

Third-party license texts and notices must remain distributed with their associated asset packages.

## GearBox CAD-derived USD assets

| Field | Value |
|---|---|
| Repository path | `assets/workpiece/GearBox/` |
| Files covered | `GearBox_2S_Reduction.usda` and `subUSDs/**` |
| Origin | Modified derivative of a CAD model obtained from the GrabCAD Library |
| Original model | `Two stage reduction gearbox` |
| Original creator | `Sukanta Nath` |
| Original source | `https://grabcad.com/library/two-stage-reduction-gearbox-5` |
| MIT status | Explicitly excluded from the root MIT License |
| Repository purpose | Visualization of this repository's educational demonstration only |
| Rights granted by repository maintainers | None for the GearBox CAD/USD assets, including no licence, sublicence, reuse, modification, redistribution, or commercial-use permission |

The GearBox-derived CAD/USD assets are third-party-derived materials. Their
inclusion in this repository is not a grant of rights to end users. Any use
other than visualization of this repository's educational demonstration
requires the user to independently obtain and verify the required permissions
from the applicable upstream rights holder(s) and under the applicable
upstream terms. The rights and terms of upstream rightsholder(s) control.

## Robot, sensor and workcell assets

| Repository path | Upstream | Notice |
|---|---|---|
| `assets/robots/ur5e_with_robotiq_2f_140/` | Universal Robots ROS 2 description | `LICENSES/NOTICE.md` |
| `assets/robots/crx5ia_with_robotiq_2f_140/` | FANUC `fanuc_description` (Apache-2.0) | `LICENSES/NOTICE.md` |
| `assets/grippers/robotiq_2f_140/` | Robotiq 2F-140 (ros-industrial-attic), mounted by both robots above | `LICENSES/NOTICE.md` |
| `assets/robots/crx5ia_with_laser_head/` | FANUC `fanuc_description` (Apache-2.0) | `LICENSES/NOTICE.md` |
| `assets/sensors/zivid/` | Zivid Isaac Sim extension (BSD-3-Clause) | `LICENSES/NOTICE.md` |

## FANUC CRX-5iA robot models

| Field | Value |
|---|---|
| Repository paths | `assets/robots/crx5ia_with_robotiq_2f_140/`, `assets/robots/crx5ia_with_laser_head/` |
| Origin | Converted to USD from the official FANUC ROS 2 description, package `fanuc_crx_description` 2.1.0, model `crx5ia` (URDF/xacro and meshes) |
| Upstream | https://github.com/FANUC-CORPORATION/fanuc_description |
| Copyright | 2025-2026 FANUC America Corporation, 2025-2026 FANUC CORPORATION |
| License | Apache-2.0; the license text is in each package's `LICENSES/` folder |
| Modifications | Listed in each package's `LICENSES/NOTICE.md`. No endorsement by FANUC is implied |

## NVIDIA Isaac Sim conveyor (not redistributed)

`scenes/laser_cleaning.usda` loads `Isaac/Props/Conveyors/ConveyorBelt_A05.usd`
directly from NVIDIA's Isaac Sim asset server. The asset is not part of this
repository and remains subject to NVIDIA's terms for Isaac Sim assets.
