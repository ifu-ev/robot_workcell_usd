# Laser-cleaning simulation

Two Isaac Sim scripts that simulate laser rust removal in the laser-cleaning cell (`scenes/laser_cleaning_flywheel.usda`):

| Script | Purpose |
|---|---|
| `laser_cleaning_raycasting.py` | Raycasts the laser's square scan pattern from the laser head onto the workpiece every tick, integrates the delivered dose per mesh face in simulation time and colours the faces: rust → clean → overexposed |
| `env_generation.py` | Puts a new workpiece at `/World/targets/part` (the path the raycast script expects), with unit correction and optional placement under the laser |

The scripts only model the laser. Moving the robot along a cleaning path is up to your robot program (for example MoveIt through the scene's topic-based joint control).

## Requirements

- Isaac Sim 6.1 with the scene's extensions: Warp (bundled), the debug-draw extension for the beam line (optional), and the ROS 2 bridge when the laser is switched over ROS
- The laser-cleaning scene open and playing

## Quick start

1. Open `scenes/laser_cleaning_flywheel.usda` and press **Play**.
2. Open **Window → Script Editor**, load `scripts/laser_cleaning/laser_cleaning_raycasting.py` and press **Run**. The script builds a raycast mesh of the workpiece, prints an aiming report and opens the **Laser Raycast Test Control** window:

   ```text
   [bvh] Built LOCAL-space BVH: 1 sub-mesh(es), 26917 verts, 28080 tris <- /World/targets/part
   [dose] Listening to /laser/enable for laser enable.
   [raycast] Loop started at ~25Hz on /World/targets/part
   ```

3. Switch the laser on:

   ```bash
   ros2 topic pub --once --qos-durability transient_local /laser/enable std_msgs/msg/Bool "{data: true}"
   ```

   Without ROS, set `LASER_ENABLE_TOPIC = ""` in the script: the laser is then always on.
4. Point the laser head at the part, so that the surface lies near the focus (254 mm from the lens), and move it over the surface. Faces inside the scan pattern turn from rust to clean as they receive the layer energy.
5. Read the result with **Dose Report** in the window, or with `laser_test.report()` in the Script Editor. For more detail, load the script as a module (below) and call `dose_summary()`; with `centre_xy=(x, y)` it also splits the result into radius bands around that point.

The beam line shows the laser state: red = miss, yellow = hit outside the working distance, green = hit inside it, grey = laser off.

### Loading as a module

To call the functions from other scripts (e.g. an experiment driver), load the file as a module instead of running it in the Script Editor:

```python
import importlib.util, sys
spec = importlib.util.spec_from_file_location("laser_cleaning_raycasting", "<repo>/scripts/laser_cleaning/laser_cleaning_raycasting.py")
m = importlib.util.module_from_spec(spec); sys.modules["laser_cleaning_raycasting"] = m; spec.loader.exec_module(m)
m.dose_summary()            # cleanable area and clean / partial / untouched / overexposed shares
                            # (empty until the laser has fired once)
m.laser_test.beam_on_s      # simulated seconds with the laser on
```

Running the script again tears down the previous instance (update loop, window, ROS subscriber) first.

## Dose model

Per tick, for every mesh face that is the first surface along the beam and whose centre lies inside the square scan pattern:

```text
irradiance  I   = LASER_POWER_W / pattern area                 1000 W / (5 cm x 5 cm) = 40 W/cm2
fluence     F   = F0 * cos(theta) / (1 + (dz / zR)^2)          dz: distance from the focus along the beam
efficiency  eta = clip((F - F_th) / (F0 - F_th), 0, 1)         0 beyond MAX_INCIDENCE_DEG
dose       += I * dt_sim * eta
```

A face is **clean** once its dose reaches `LAYER_ENERGY_J_CM2` and **overexposed** when the dose exceeds the layer energy by more than `OVEREXPOSE_J_CM2`. The dose is integrated in simulation time, so pausing the simulation stops the cleaning and a slower simulation does not change the result.

Typical layer energies:

| Rust | `LAYER_ENERGY_J_CM2` | Feed at full efficiency (1 kW, 50 mm pattern) |
|---|---|---|
| light | 25 | 80 mm/s |
| medium | 40 | 50 mm/s |
| heavy | 120 (default) | 16.7 mm/s |

The feed at full efficiency is `LASER_POWER_W / (pattern width x layer energy)`; any defocus or incidence slows the cleaning down from there.

## Settings

The **USER CONFIG** block at the top of `laser_cleaning_raycasting.py`:

| Setting | Default | Meaning |
|---|---|---|
| `TARGET_ROOT_PATH` | `/World/targets/part` | Workpiece root; everything below it is raycast |
| `EMITTER_PATH` | `.../Fanuc_Laser_Head/emitter` | Laser origin; the beam points along its +Z |
| `FOOTPRINT_SIZE_M` | 0.05 | Edge of the square scan pattern |
| `GRID_N` | 21 | Rays per pattern edge (21 × 21 rays, 2.5 mm apart) |
| `DOSE_MODEL` | `True` | `False`: paint instantly on hit (preview mode, no dose) |
| `LASER_POWER_W` | 1000 | Average laser power |
| `FOCAL_DIST_M` | 0.254 | Lens to focus |
| `F0_J_CM2`, `ZR_M`, `F_TH_J_CM2` | 3.0, 0.0365, 1.5 | Peak fluence, defocus at which it halves, ablation threshold |
| `MAX_INCIDENCE_DEG` | 70 | Steeper faces receive no dose |
| `LAYER_ENERGY_J_CM2` | 120 | Energy to remove the rust layer (see the table above) |
| `OVEREXPOSE_J_CM2` | 150 | Dose beyond the layer energy counted as overexposure |
| `LASER_ENABLE_TOPIC` | `/laser/enable` | `std_msgs/Bool` that switches the laser; `""` = always on |
| `WORKING_DIST_MIN/MAX` | 0.10 / 0.50 m | Distance range shown as an in-gate hit (beam colour, preview mode) |
| `UNBIND_TARGET_MATERIAL` | `True` | Unbind the part's material so the dose colours show |

**Move only the workpiece root prim** (`TARGET_ROOT_PATH`) while the script runs. The raycast mesh is built once in the root's local space; moving a child prim below it makes the mesh stale (a guard detects this and rebuilds).

## New workpiece

1. Create a scene that sublayers `scenes/laser_cleaning.usda`, like `scenes/laser_cleaning_flywheel.usda`, and open it. Do not run the script on `laser_cleaning.usda` itself; keep that file free of workpieces.
2. In `env_generation.py`, set `ASSET_PATH` to the part's USD (relative to the scene file, or absolute) and run it in the Script Editor. It references the part at `/World/targets/part` with a (translate, rotateX, scale) transform, scales parts authored in millimetres to metres, can lay flat parts down (`AUTO_STAND_UP`) and can place the part under the laser (`AUTO_PLACE_UNDER_EMITTER`).
3. Move the part where you want it (e.g. onto the rollers, top at z = 0.7674 m), save the scene and run `laser_cleaning_raycasting.py`.
