# Robot Workcell USD

Portable Isaac Sim USD assets for a gearbox-disassembly workcell with a UR5e + Robotiq 2F-140, Zivid camera, fixed workcell, and gearbox.

## Workcell

![Isaac Sim gearbox-disassembly workcell with UR5e, Robotiq 2F-140 gripper, Zivid camera, enclosure and gearbox](docs/images/workcell.png)

## Gearbox

![Two-stage reduction gearbox, showing its housing, gears, shafts, bearings etc.](docs/images/gearbox.png)


## Repository layout

```text
├── assets
│   ├── grippers
│   ├── robots
│   │   └── ur5e_with_robotiq_2f_140
│   ├── sensors
│   │   └── zivid
│   ├── textures
│   │   └── fabric_0032_normal_opengl_1k.png
│   ├── workcells
│   │   └── racpro-isaac
│   └── workpiece
│       └── GearBox
├── docs
│   └── images
│       ├── gearbox.jpg
│       └── workcell.jpg
├── LICENSE.txt
├── README.md
├── scenes
│   └── gearbox_disassembly.usda
├── scripts
│   └── add_pickit_and_ros2_omnigraph.py
└── THIRD_PARTY_NOTICES.md


```
## Requirements

- NVIDIA Isaac Sim **5.1**.
- ROS 2 Jazzy (Only for ROS2 topic based controller or point cloud publisher)

Set your Isaac Sim location once per terminal:

```bash
export ISAACSIM_ROOT="$HOME/isaacsim"
```

Source the Isaac Sim supported ROS 2 distribution:

```bash
source /opt/ros/<your_ros_distro>/setup.bash
```

## Clone the repository

```bash
git clone <your-repository-url> robot_workcell_usd
cd robot_workcell_usd
git checkout isaacsim_5.1

```

## Open the base scene

Start Isaac Sim 5.1 and open:

```text
<repository-root>/scenes/gearbox_disassembly.usda
```

Open `scenes/gearbox_disassembly.usda` in Isaac Sim 5.1. Before adding ROS graphs, press Play and confirm that the robot, gripper, workcell, and gearbox are stable.

## Adding ROS2 Omnigraphs
### Option A: Standalone ROS builder

The standalone builder opens the scene, validates the required prims, creates selected graphs, saves the root layer, and exits.

```bash
export ISAACSIM_ROOT="$HOME/isaacsim"
"$ISAACSIM_ROOT/python.sh" \
  scripts/add_pickit_and_ros2_omnigraph.py \
  --root "$PWD" \
  --headless
```

```bash
# Controller, joint state, and clock only.
"$ISAACSIM_ROOT/python.sh" scripts/add_pickit_and_ros2_omnigraph.py \
  --root "$PWD" --no-pickit --headless

# Zivid publishers and TF only.
"$ISAACSIM_ROOT/python.sh" scripts/add_pickit_and_ros2_omnigraph.py \
  --root "$PWD" --no-moveit --headless

# Replace only requested existing graph paths.
"$ISAACSIM_ROOT/python.sh" scripts/add_pickit_and_ros2_omnigraph.py \
  --root "$PWD" --overwrite --headless
```

The full builder creates:

```text
/World/ROS2/Topic_Based_Controller
/World/ROS2/Camera_Publisher
/World/ROS2/TF_Publisher
```

### Option B: Script Editor GUI builder

Use this option when the scene is already open in Isaac Sim. The complete GUI code is committed as:

```python
# Isaac Sim 5.1 Script Editor tool.
# Run only with a saved gearbox_disassembly.usda stage already open.
# This script creates selected ROS 2 OmniGraphs and can save the root USD layer.

from pxr import Sdf, Usd
import omni.graph.core as og
import omni.kit.app
import omni.ui as ui
import omni.usd
import carb
import traceback

LOG_PATH = "/tmp/isaacsim_ros2_omnigraph_builder.log"

ROBOT_PRIM = "/World/ur5e_"
JOINT_STATE_PRIM = "/World/ur5e_/root_joint"
CAMERA_PRIM = "/World/Zivid/projector"
TF_PARENT_PRIM = "/World/ur5e_/world"
ROS_ROOT = "/World/ROS2"
CTRL_GRAPH = f"{ROS_ROOT}/Topic_Based_Controller"
CAM_GRAPH = f"{ROS_ROOT}/Camera_Publisher"
TF_GRAPH = f"{ROS_ROOT}/TF_Publisher"


def log_event(level, message):
    line = f"[ROS2_GRAPH_BUILDER] {message}"
    print(line)
    with open(LOG_PATH, "a", encoding="utf-8") as stream:
        stream.write(line + "\\n")
    if level == "error":
        carb.log_error(line)
    elif level == "warning":
        carb.log_warn(line)
    else:
        carb.log_info(line)


def ensure_ros2_bridge():
    manager = omni.kit.app.get_app().get_extension_manager()
    extension = "isaacsim.ros2.bridge"
    if not manager.is_extension_enabled(extension):
        manager.set_extension_enabled_immediate(extension, True)
        log_event("info", f"Enabled extension: {extension}")


def edit_graph(path, nodes, connections, values=()):
    keys = og.Controller.Keys
    commands = {
        keys.CREATE_NODES: nodes,
        keys.CONNECT: connections,
    }
    if values:
        commands[keys.SET_VALUES] = list(values)
    log_event("info", f"Creating graph {path}: {len(nodes)} nodes, {len(connections)} connections")
    return og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        commands,
    )


def set_rel(stage, prim_path, rel_name, targets):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Missing graph prim: {prim_path}")
    relationship = prim.GetRelationship(rel_name)
    if not relationship:
        raise RuntimeError(f"Missing relationship: {prim_path}.{rel_name}")
    relationship.SetTargets([Sdf.Path(target) for target in targets])


def create_controller(stage):
    edit_graph(
        CTRL_GRAPH,
        [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_publish_joint_state", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("ros2_subscribe_joint_state", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("articulation_controller", "isaacsim.core.nodes.IsaacArticulationController"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ros2_qos_profile", "isaacsim.ros2.bridge.ROS2QoSProfile"),
            ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ros2_publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        [
            ("on_playback_tick.outputs:tick", "ros2_publish_joint_state.inputs:execIn"),
            ("on_playback_tick.outputs:tick", "ros2_subscribe_joint_state.inputs:execIn"),
            ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
            ("on_playback_tick.outputs:tick", "ros2_publish_clock.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_publish_joint_state.inputs:context"),
            ("ros2_context.outputs:context", "ros2_subscribe_joint_state.inputs:context"),
            ("ros2_context.outputs:context", "ros2_publish_clock.inputs:context"),
            ("ros2_qos_profile.outputs:qosProfile", "ros2_publish_joint_state.inputs:qosProfile"),
            ("ros2_qos_profile.outputs:qosProfile", "ros2_subscribe_joint_state.inputs:qosProfile"),
            ("ros2_qos_profile.outputs:qosProfile", "ros2_publish_clock.inputs:qosProfile"),
            ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_joint_state.inputs:timeStamp"),
            ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_clock.inputs:timeStamp"),
            ("ros2_subscribe_joint_state.outputs:effortCommand", "articulation_controller.inputs:effortCommand"),
            ("ros2_subscribe_joint_state.outputs:jointNames", "articulation_controller.inputs:jointNames"),
            ("ros2_subscribe_joint_state.outputs:positionCommand", "articulation_controller.inputs:positionCommand"),
            ("ros2_subscribe_joint_state.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
        ],
        [
            ("isaac_read_simulation_time.inputs:resetOnStop", True),
            ("articulation_controller.inputs:robotPath", JOINT_STATE_PRIM),
        ],
    )
    set_rel(stage, f"{CTRL_GRAPH}/ros2_publish_joint_state", "inputs:targetPrim", [JOINT_STATE_PRIM])
    set_rel(stage, f"{CTRL_GRAPH}/articulation_controller", "inputs:targetPrim", [ROBOT_PRIM])


def create_pickit(stage, width, height):
    edit_graph(
        CAM_GRAPH,
        [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("isaac_create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ros2_qos_profile", "isaacsim.ros2.bridge.ROS2QoSProfile"),
            ("ros2_camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("ros2_camera_helper_02", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("ros2_camera_info_helper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ],
        [
            ("on_playback_tick.outputs:tick", "isaac_create_render_product.inputs:execIn"),
            ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
            ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper_02.inputs:execIn"),
            ("isaac_create_render_product.outputs:execOut", "ros2_camera_info_helper.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_camera_helper.inputs:context"),
            ("ros2_context.outputs:context", "ros2_camera_helper_02.inputs:context"),
            ("ros2_qos_profile.outputs:qosProfile", "ros2_camera_helper.inputs:qosProfile"),
            ("ros2_qos_profile.outputs:qosProfile", "ros2_camera_helper_02.inputs:qosProfile"),
            ("ros2_qos_profile.outputs:qosProfile", "ros2_camera_info_helper.inputs:qosProfile"),
            ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper.inputs:renderProductPath"),
            ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper_02.inputs:renderProductPath"),
            ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_info_helper.inputs:renderProductPath"),
        ],
        [
            ("isaac_create_render_product.inputs:width", width),
            ("isaac_create_render_product.inputs:height", height),
            ("ros2_qos_profile.inputs:depth", 5),
            ("ros2_qos_profile.inputs:durability", "transientLocal"),
            ("ros2_qos_profile.inputs:reliability", "bestEffort"),
            ("ros2_camera_helper.inputs:frameId", "ZividCamera"),
            ("ros2_camera_helper.inputs:queueSize", 10),
            ("ros2_camera_helper.inputs:topicName", "ZividCamera/pointcloud"),
            ("ros2_camera_helper.inputs:type", "depth_pcl"),
            ("ros2_camera_helper.inputs:useSystemTime", False),
            ("ros2_camera_helper_02.inputs:frameId", "ZividCamera"),
            ("ros2_camera_helper_02.inputs:queueSize", 10),
            ("ros2_camera_helper_02.inputs:topicName", "ZividCamera/rgb"),
            ("ros2_camera_helper_02.inputs:type", "rgb"),
            ("ros2_camera_helper_02.inputs:useSystemTime", False),
            ("ros2_camera_info_helper.inputs:frameId", "ZividCamera"),
            ("ros2_camera_info_helper.inputs:topicName", "ZividCamera/camera_info"),
        ],
    )
    set_rel(stage, f"{CAM_GRAPH}/isaac_create_render_product", "inputs:cameraPrim", [CAMERA_PRIM])

    edit_graph(
        TF_GRAPH,
        [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_publish_transform_tree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ros2_qos_profile", "isaacsim.ros2.bridge.ROS2QoSProfile"),
        ],
        [
            ("on_playback_tick.outputs:tick", "ros2_publish_transform_tree.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_publish_transform_tree.inputs:context"),
            ("ros2_qos_profile.outputs:qosProfile", "ros2_publish_transform_tree.inputs:qosProfile"),
            ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_transform_tree.inputs:timeStamp"),
        ],
    )
    set_rel(stage, f"{TF_GRAPH}/ros2_publish_transform_tree", "inputs:parentPrim", [TF_PARENT_PRIM])
    set_rel(stage, f"{TF_GRAPH}/ros2_publish_transform_tree", "inputs:targetPrims", [CAMERA_PRIM])


class Builder:
    def __init__(self):
        self.pickit = ui.SimpleBoolModel(True)
        self.moveit = ui.SimpleBoolModel(True)
        self.overwrite = ui.SimpleBoolModel(False)
        self.save_after_create = ui.SimpleBoolModel(True)
        self.width = ui.SimpleIntModel(972)
        self.height = ui.SimpleIntModel(600)
        self.window = ui.Window("ROS2 OmniGraph Builder", width=530, height=390)

        with self.window.frame:
            with ui.VStack(spacing=8):
                ui.Label("ROS2 OmniGraph Builder", style={"font_size": 20})
                ui.Label("Open and save gearbox_disassembly.usda before creating graphs.", word_wrap=True)
                with ui.HStack(height=24):
                    ui.CheckBox(model=self.pickit, width=24)
                    ui.Label("Pickit: Zivid camera publishers + TF publisher")
                with ui.HStack(height=24):
                    ui.CheckBox(model=self.moveit, width=24)
                    ui.Label("MoveIt2: topic controller + joint state + clock")
                ui.Label("Camera render-product resolution")
                with ui.HStack(height=28):
                    ui.Label("Width", width=75)
                    ui.IntField(model=self.width, width=130)
                    ui.Label("Height", width=55)
                    ui.IntField(model=self.height, width=130)
                with ui.HStack(height=24):
                    ui.CheckBox(model=self.overwrite, width=24)
                    ui.Label("I confirm overwrite of selected existing graphs")
                with ui.HStack(height=24):
                    ui.CheckBox(model=self.save_after_create, width=24)
                    ui.Label("Save scene after creation")
                ui.Button("Create Graphs", height=34, clicked_fn=self.create)
                self.status = ui.Label("Select graphs and press Create Graphs.", word_wrap=True, height=70)

    def message(self, text, level="info"):
        self.status.text = text
        log_event(level, text)

    def create(self):
        pickit = self.pickit.get_value_as_bool()
        moveit = self.moveit.get_value_as_bool()
        width = self.width.get_value_as_int()
        height = self.height.get_value_as_int()

        if not pickit and not moveit:
            self.message("Select Pickit and/or MoveIt2.", "warning")
            return
        if width <= 0 or height <= 0:
            self.message("Camera width and height must be positive.", "warning")
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None or stage.GetRootLayer().anonymous:
            self.message("Open and save a USD stage before creating graphs.", "error")
            return

        required = ([] if not moveit else [ROBOT_PRIM, JOINT_STATE_PRIM])
        required += ([] if not pickit else [CAMERA_PRIM, TF_PARENT_PRIM])
        missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
        if missing:
            self.message("Missing required prim(s): " + ", ".join(missing), "error")
            return

        requested = ([] if not moveit else [CTRL_GRAPH])
        requested += ([] if not pickit else [CAM_GRAPH, TF_GRAPH])
        existing = [path for path in requested if stage.GetPrimAtPath(path).IsValid()]
        if existing and not self.overwrite.get_value_as_bool():
            self.message("Existing graph(s) found. Check overwrite and click again: " + ", ".join(existing), "warning")
            return

        try:
            ensure_ros2_bridge()
            root_layer = stage.GetRootLayer()
            with Usd.EditContext(stage, Usd.EditTarget(root_layer)):
                if not stage.GetPrimAtPath(ROS_ROOT).IsValid():
                    stage.DefinePrim(ROS_ROOT, "Scope")
                for path in existing:
                    stage.RemovePrim(path)
                if moveit:
                    create_controller(stage)
                if pickit:
                    create_pickit(stage, width, height)
                if self.save_after_create.get_value_as_bool():
                    root_layer.Save()

            made = ([] if not moveit else ["Topic_Based_Controller"])
            made += ([] if not pickit else ["Camera_Publisher", "TF_Publisher"])
            saved = " Scene saved." if self.save_after_create.get_value_as_bool() else " Save the scene manually with Ctrl+S."
            self.message("Created: " + ", ".join(made) + "." + saved)
        except Exception as exc:
            details = traceback.format_exc()
            log_event("error", f"Creation failed: {exc!r}\\n{details}")
            self.message(f"Creation failed: {exc!r}. Full traceback: {LOG_PATH}", "error")


try:
    ROS2_OMNIGRAPH_BUILDER.window.visible = False
except Exception:
    pass
ROS2_OMNIGRAPH_BUILDER = Builder()

```

1. Open and save `scenes/gearbox_disassembly.usda`.
2. Open **Window → Script Editor**.
3. Copy the **entire** contents into a new Python tab, and press **Run**.
4. The **ROS2 OmniGraph Builder** window appears.
5. Select one or both graph groups:
<br>
    a. Pickit: Enable to create ROS2 point cloud publisher
<br>
    b. Moveit: Enable to create `/joint_states` publisher and `/joint_command` subscriber
6. Set the desired **camera resolution** for Pickit
7. Press **Create Graphs**.

The GUI exposes these controls:

| Control | Result |
|---|---|
| Pickit | Creates Zivid RGB, point-cloud, camera-info, and TF graphs |
| MoveIt2 | Creates controller, joint-state, and clock graph |
| Width / Height | Zivid render-product resolution; default 972 × 600 |
| I confirm replacement of selected existing graphs | Permits replacement of only selected graph paths |
| Save scene after graph creation | Saves `gearbox_disassembly.usda` after successful creation |


## OmniGraph details

The builder validates these scene paths before it writes anything:

```text
Robot wrapper:           /World/ur5e_
Articulation root joint: /World/ur5e_/root_joint
Zivid camera:            /World/Zivid/projector
TF parent:               /World/ur5e_/world
```

The Isaac Sim 5.1 controller graph uses `Isaac Read Simulation Time` as the timestamp source for the ROS 2 joint-state and clock publishers:

```text
Isaac Read Simulation Time
  → ROS2 Publish Joint State.inputs:timeStamp
  → ROS2 Publish Clock.inputs:timeStamp
```

The Isaac Sim 5.1 TF graph directly configures the ROS 2 Transform Tree publisher with USD relationships:

```text
ROS2 Publish Transform Tree.inputs:parentPrim
  = /World/ur5e_/world

ROS2 Publish Transform Tree.inputs:targetPrims
  = /World/Zivid/projector
```

The TF publisher is executed from `On Playback Tick` and timestamped with `Isaac Read Simulation Time`.

The default Zivid ROS 2 output is:

```text
Frame ID:     ZividCamera
RGB topic:    ZividCamera/rgb
Point cloud:  ZividCamera/pointcloud
Camera info:  ZividCamera/camera_info
```

## Validation

After graph creation:

1. Confirm selected graph prims under `/World/ROS2`.
2. Press Play and verify the UR5e remains stable.
3. Confirm the Zivid render product initializes.
4. Source the workstation ROS 2 distribution and run `ros2 topic list`.
5. Verify gearbox, plank, and Zelle visibility in RGB and point-cloud output.

Before commit:

```bash
git status
git diff --check
git diff --stat
```

Do not author physics, materials, colliders, or transforms below composed robot collision instance-proxy paths such as `/World/ur5e_/wrist_3_link/collisions/...`. Keep project-specific USD dependencies repository-relative. 

## License

The **MIT License** applies only to original copyrightable material authored for this repository, except where a file, directory, or accompanying notice states otherwise. Third-party assets remain subject to their applicable rights, terms, notices, and redistribution conditions; see `THIRD_PARTY_NOTICES.md`.

#### GearBox workpiece

assets/workpiece/GearBox/ contains modified and OpenUSD-converted assets derived from a CAD model obtained from the GrabCAD Library. The GearBox files are supplied **only to allow visualization of this repository's educational demonstration**. They are not licensed under the repository's MIT License.

The repository maintainers grant no licence, sublicence, reuse permission, modification permission, redistribution permission, or commercial-use permission for the GearBox CAD/USD assets. For any use other than visualization, the users must obtain and verify any necessary rights directly from the applicable upstream rights holder(s) and under the applicable upstream terms. See `assets/workpiece/GearBox/NOTICE.md`.