# Robot Workcell USD

Portable Isaac Sim USD scenes and assets for three robot cells around a fixed workcell with a Zivid camera:

| Scene | Robot | Task |
|---|---|---|
| `scenes/gearbox_disassembly_ur5e.usda` | UR5e + Robotiq 2F-140 | Gearbox disassembly (tilted Zivid) |
| `scenes/gearbox_disassembly_crx5ia.usda` | FANUC CRX-5iA + Robotiq 2F-140 | Gearbox disassembly (recentred Zivid) |
| `scenes/laser_cleaning_flywheel.usda` | FANUC CRX-5iA + laser head | Laser cleaning of a rusted flywheel on a roller conveyor |

All scenes include ROS 2 OmniGraphs (topic-based joint control, joint states, clock, Zivid camera, TF) and keep `/clock`, `/tf` and `/joint_states` monotonic across Stop/Play.

## Workcell overview

UR5e + Robotiq 2F-140 (`scenes/gearbox_disassembly_ur5e.usda`):

![Isaac Sim gearbox-disassembly workcell with UR5e, Robotiq 2F-140 gripper, Zivid camera, enclosure and gearbox](docs/images/workcell.png)

FANUC CRX-5iA + Robotiq 2F-140 (`scenes/gearbox_disassembly_crx5ia.usda`):

![Isaac Sim gearbox-disassembly workcell with FANUC CRX-5iA, Robotiq 2F-140 gripper, Zivid camera, enclosure and gearbox](docs/images/workcell_crx5ia.png)

## Gearbox workpiece

![Two-stage reduction gearbox, showing its housing, gears, shafts, bearings etc.](docs/images/gearbox.png)


## Repository layout

```text
├── assets
│   ├── grippers
│   │   └── robotiq_2f_140             # shared by both gripper robots
│   ├── robots
│   │   ├── ur5e_with_robotiq_2f_140
│   │   ├── crx5ia_with_robotiq_2f_140
│   │   └── crx5ia_with_laser_head
│   ├── sensors
│   │   └── zivid
│   ├── textures
│   │   └── fabric_0032_normal_opengl_1k.png
│   ├── workcells
│   │   └── racpro-isaac
│   └── workpiece
│       ├── GearBox
│       └── FlyWheel
├── docs
│   └── images
│       ├── gearbox.png
│       ├── workcell.png
│       └── workcell_crx5ia.png
├── LICENSE.txt
├── README.md
├── scenes
│   ├── gearbox_disassembly_ur5e.usda
│   ├── gearbox_disassembly_crx5ia.usda
│   ├── laser_cleaning.usda              # laser cell without a workpiece
│   └── laser_cleaning_flywheel.usda     # laser cell + flywheel
├── scripts
│   ├── add_pickit_and_ros2_omnigraph.py
│   └── laser_cleaning                 # laser simulation (see its README.md)
└── THIRD_PARTY_NOTICES.md
```

## Requirements

- NVIDIA Isaac Sim **5.1** (tested with 5.1.0). This branch has the same scenes and assets as `isaacsim_6.1`, with the ROS 2 graphs built from the Isaac Sim 5.1 nodes (see [OmniGraph details](#omnigraph-details)).
- Internet access for the laser cell: its conveyor is loaded from NVIDIA's Isaac Sim asset server.
- ROS 2 Jazzy (Only for ROS2 topic based controller or point cloud publisher)

Set your Isaac Sim location once per terminal:

```bash
export ISAACSIM_ROOT="$HOME/isaacsim"
```

Start Isaac Sim 5.1 with its bundled ROS 2 Jazzy libraries. Isaac Sim 5.1 runs Python 3.11, and a system ROS 2 Jazzy (Python 3.12) sourced in the same terminal stops its Python ROS 2 API (`rclpy`) from loading: the OmniGraphs still publish, but Python ROS 2 code such as the laser script's `/laser/enable` subscriber fails. In the terminal that starts Isaac Sim, do not source `/opt/ros/jazzy` (if your `~/.bashrc` sources it, start from a clean shell, e.g. `env -i HOME="$HOME" PATH=/usr/bin:/bin DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" bash --norc`, and export `ISAACSIM_ROOT` and `ROS_DOMAIN_ID` there); instead:

```bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ISAACSIM_ROOT/exts/isaacsim.ros2.bridge/jazzy/lib"
"$ISAACSIM_ROOT/isaac-sim.sh"
```

Run other ROS 2 tools (`ros2 topic`, MoveIt) in terminals with `/opt/ros/jazzy/setup.bash` sourced and the same `ROS_DOMAIN_ID`.

## Clone the repository

```bash
git clone https://github.com/ifu-ev/robot_workcell_usd.git robot_workcell_usd
cd robot_workcell_usd
git checkout isaacsim_5.1

```

## Open a scene

Start Isaac Sim 5.1 and open one of the scenes, e.g.:

```text
<repository-root>/scenes/gearbox_disassembly_ur5e.usda
```

Press Play and confirm that the robot, gripper (or laser head), workcell and workpiece are stable. The ROS 2 graphs are already part of the scenes; the builder below recreates them for the UR5e scene.

## Laser-cleaning cell

`scenes/laser_cleaning.usda` is the cell without a workpiece: CRX-5iA with laser head (emitter 0.18 m along `fanuc_flange` +Z), Zivid camera, roller conveyor, ROS 2 graphs and physics, plus an empty `/World/targets` group. `scenes/laser_cleaning_flywheel.usda` sublayers it and adds the rusted flywheel at `/World/targets/part`, resting on the rollers under the Zivid axis. To use another workpiece, add a scene that sublayers `laser_cleaning.usda` and references the part at `/World/targets/part`; keep `laser_cleaning.usda` itself free of workpieces so that every target scene picks up changes to the cell.

The laser itself is simulated by the scripts in `scripts/laser_cleaning/`: a raycast of the scan pattern onto the workpiece with a dose model that colours rust, cleaned and overexposed surface. See [scripts/laser_cleaning/README.md](scripts/laser_cleaning/README.md).

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
# Paste this complete file into Isaac Sim 5.1 Window > Script Editor and press Run.
from pxr import Sdf, Usd
import omni.graph.core as og
import omni.kit.app
import omni.ui as ui
import omni.usd
import traceback

# Set the following to your specific scene
ROBOT="/World/ur5e_"
JOINT=f"{ROBOT}/root_joint" 
CAMERA="/World/Zivid/projector" 
PARENT=f"{ROBOT}/world"
ROOT="/World/ROS2" 
CTRL=f"{ROOT}/Topic_Based_Controller" 
CAM=f"{ROOT}/Camera_Publisher" 
TF=f"{ROOT}/TF_Publisher"

def edit(path,nodes,links,values=()):
    k=og.Controller.Keys; d={k.CREATE_NODES:nodes,k.CONNECT:links}
    if values: d[k.SET_VALUES]=list(values)
    og.Controller.edit({"graph_path":path,"evaluator_name":"execution"},d)

def rel(stage,path,name,targets):
    r=stage.GetPrimAtPath(path).GetRelationship(name)
    if not r: raise RuntimeError(f"Missing relationship: {path}.{name}")
    r.SetTargets([Sdf.Path(x) for x in targets])

def controller(s):
    edit(CTRL,[("tick","omni.graph.action.OnPlaybackTick"),("pub","isaacsim.ros2.bridge.ROS2PublishJointState"),("sub","isaacsim.ros2.bridge.ROS2SubscribeJointState"),("ctrl","isaacsim.core.nodes.IsaacArticulationController"),("ctx","isaacsim.ros2.bridge.ROS2Context"),("qos","isaacsim.ros2.bridge.ROS2QoSProfile"),("time","isaacsim.core.nodes.IsaacReadSimulationTime"),("clock","isaacsim.ros2.bridge.ROS2PublishClock")],[("tick.outputs:tick","pub.inputs:execIn"),("tick.outputs:tick","sub.inputs:execIn"),("tick.outputs:tick","ctrl.inputs:execIn"),("tick.outputs:tick","clock.inputs:execIn"),("ctx.outputs:context","pub.inputs:context"),("ctx.outputs:context","sub.inputs:context"),("ctx.outputs:context","clock.inputs:context"),("qos.outputs:qosProfile","pub.inputs:qosProfile"),("qos.outputs:qosProfile","sub.inputs:qosProfile"),("qos.outputs:qosProfile","clock.inputs:qosProfile"),("time.outputs:simulationTime","pub.inputs:timeStamp"),("time.outputs:simulationTime","clock.inputs:timeStamp"),("sub.outputs:jointNames","ctrl.inputs:jointNames"),("sub.outputs:positionCommand","ctrl.inputs:positionCommand"),("sub.outputs:velocityCommand","ctrl.inputs:velocityCommand"),("sub.outputs:effortCommand","ctrl.inputs:effortCommand")],[("time.inputs:resetOnStop",True),("ctrl.inputs:robotPath",JOINT)])
    rel(s,f"{CTRL}/pub","inputs:targetPrim",[JOINT]); rel(s,f"{CTRL}/ctrl","inputs:targetPrim",[ROBOT])

def camera_and_tf(s,w,h):
    edit(CAM,[("tick","omni.graph.action.OnPlaybackTick"),("rp","isaacsim.core.nodes.IsaacCreateRenderProduct"),("ctx","isaacsim.ros2.bridge.ROS2Context"),("qos","isaacsim.ros2.bridge.ROS2QoSProfile"),("pcl","isaacsim.ros2.bridge.ROS2CameraHelper"),("rgb","isaacsim.ros2.bridge.ROS2CameraHelper"),("info","isaacsim.ros2.bridge.ROS2CameraInfoHelper")],[("tick.outputs:tick","rp.inputs:execIn"),("rp.outputs:execOut","pcl.inputs:execIn"),("rp.outputs:execOut","rgb.inputs:execIn"),("rp.outputs:execOut","info.inputs:execIn"),("ctx.outputs:context","pcl.inputs:context"),("ctx.outputs:context","rgb.inputs:context"),("qos.outputs:qosProfile","pcl.inputs:qosProfile"),("qos.outputs:qosProfile","rgb.inputs:qosProfile"),("qos.outputs:qosProfile","info.inputs:qosProfile"),("rp.outputs:renderProductPath","pcl.inputs:renderProductPath"),("rp.outputs:renderProductPath","rgb.inputs:renderProductPath"),("rp.outputs:renderProductPath","info.inputs:renderProductPath")],[("rp.inputs:width",w),("rp.inputs:height",h),("qos.inputs:depth",5),("qos.inputs:durability","transientLocal"),("qos.inputs:reliability","bestEffort"),("pcl.inputs:frameId","ZividCamera"),("pcl.inputs:topicName","ZividCamera/pointcloud"),("pcl.inputs:type","depth_pcl"),("rgb.inputs:frameId","ZividCamera"),("rgb.inputs:topicName","ZividCamera/rgb"),("rgb.inputs:type","rgb"),("info.inputs:frameId","ZividCamera"),("info.inputs:topicName","ZividCamera/camera_info")])
    rel(s,f"{CAM}/rp","inputs:cameraPrim",[CAMERA])
    edit(TF,[("tick","omni.graph.action.OnPlaybackTick"),("pub","isaacsim.ros2.bridge.ROS2PublishTransformTree"),("time","isaacsim.core.nodes.IsaacReadSimulationTime"),("ctx","isaacsim.ros2.bridge.ROS2Context"),("qos","isaacsim.ros2.bridge.ROS2QoSProfile")],[("tick.outputs:tick","pub.inputs:execIn"),("time.outputs:simulationTime","pub.inputs:timeStamp"),("ctx.outputs:context","pub.inputs:context"),("qos.outputs:qosProfile","pub.inputs:qosProfile")])
    rel(s,f"{TF}/pub","inputs:parentPrim",[PARENT]); rel(s,f"{TF}/pub","inputs:targetPrims",[CAMERA])

class Builder:
    def __init__(self):
        self.pickit=ui.SimpleBoolModel(True); self.moveit=ui.SimpleBoolModel(True)
        self.overwrite=ui.SimpleBoolModel(False); self.save=ui.SimpleBoolModel(True)
        self.width=ui.SimpleIntModel(972); self.height=ui.SimpleIntModel(600)
        self.window=ui.Window("ROS2 OmniGraph Builder",width=570,height=410)
        with self.window.frame:
            with ui.VStack(spacing=8):
                ui.Label("Isaac Sim 5.1 ROS2 OmniGraph Builder",style={"font_size":20})
                ui.Label("Open and save gearbox_disassembly_ur5e.usda before creation.",word_wrap=True)
                with ui.HStack(height=24): ui.CheckBox(model=self.pickit,width=24); ui.Label("Pickit: Zivid RGB, point cloud, camera info, and TF")
                with ui.HStack(height=24): ui.CheckBox(model=self.moveit,width=24); ui.Label("MoveIt2: controller, joint state, and clock")
                ui.Label("Zivid render-product resolution")
                with ui.HStack(height=28):
                    ui.Label("Width",width=65); ui.IntField(model=self.width,width=150)
                    ui.Label("Height",width=65); ui.IntField(model=self.height,width=150)
                with ui.HStack(height=24): ui.CheckBox(model=self.overwrite,width=24); ui.Label("I confirm replacement of selected existing graphs")
                with ui.HStack(height=24): ui.CheckBox(model=self.save,width=24); ui.Label("Save scene after graph creation")
                ui.Button("Create Graphs",height=36,clicked_fn=self.create)
                self.status=ui.Label("Select graph types, then click Create Graphs.",word_wrap=True,height=64)

    def message(self,text):
        self.status.text=text; print("ROS2 OmniGraph Builder:",text)

    def create(self):
        try:
            pickit=self.pickit.get_value_as_bool(); moveit=self.moveit.get_value_as_bool()
            w=self.width.get_value_as_int(); h=self.height.get_value_as_int()
            if not pickit and not moveit: raise RuntimeError("Select Pickit and/or MoveIt2.")
            if w<=0 or h<=0: raise RuntimeError("Camera width and height must be positive.")
            s=omni.usd.get_context().get_stage()
            if s is None or s.GetRootLayer().anonymous: raise RuntimeError("Open and save a USD stage before creating graphs.")
            needed=([] if not moveit else [ROBOT,JOINT])+([] if not pickit else [CAMERA,PARENT])
            missing=[x for x in needed if not s.GetPrimAtPath(x).IsValid()]
            if missing: raise RuntimeError("Missing required prim(s): "+", ".join(missing))
            wanted=([] if not moveit else [CTRL])+([] if not pickit else [CAM,TF])
            existing=[x for x in wanted if s.GetPrimAtPath(x).IsValid()]
            if existing and not self.overwrite.get_value_as_bool():
                raise RuntimeError("Existing graph(s): "+", ".join(existing)+". Check the overwrite confirmation box and click Create Graphs again to replace only these graphs.")
            manager=omni.kit.app.get_app().get_extension_manager()
            for ext in ("isaacsim.ros2.bridge",):
                if not manager.is_extension_enabled(ext): manager.set_extension_enabled_immediate(ext,True)
            with Usd.EditContext(s,Usd.EditTarget(s.GetRootLayer())):
                if not s.GetPrimAtPath(ROOT).IsValid(): s.DefinePrim(ROOT,"Scope")
                for x in existing: s.RemovePrim(x)
                if moveit: controller(s)
                if pickit: camera_and_tf(s,w,h)
                if self.save.get_value_as_bool(): s.GetRootLayer().Save()
            saved=" Scene saved." if self.save.get_value_as_bool() else " Save manually with Ctrl+S."
            self.message("Created: "+", ".join(wanted)+"."+saved)
        except Exception as e:
            traceback.print_exc(); self.message("Creation failed: "+str(e))

try: 
  ROS2_OMNIGRAPH_BUILDER.window.visible=False
except Exception as ex: 
  print(f"\nWarning: {ex}\n")
ROS2_OMNIGRAPH_BUILDER=Builder()

```

1. Open and save `scenes/gearbox_disassembly_ur5e.usda`.
2. Open **Window → Script Editor**.
3. Copy the **entire** contents into a new Python tab, and press **Run**.
4. The **ROS2 OmniGraph Builder** window appears.
5. Select one or both graph groups:
    * **a. Pickit:** Enable to create ROS2 point cloud publisher
    * **b. Moveit:** Enable to create `/joint_states` publisher and `/joint_command` subscriber
6. Set the desired **camera resolution** for Pickit
7. Press **Create Graphs**.

The GUI exposes these controls:

| Control | Result |
|---|---|
| Pickit | Creates Zivid RGB, point-cloud, camera-info, and TF graphs |
| MoveIt2 | Creates controller, joint-state, and clock graph |
| Width / Height | Zivid render-product resolution; default 972 × 600 |
| I confirm replacement of selected existing graphs | Permits replacement of only selected graph paths |
| Save scene after graph creation | Saves `gearbox_disassembly_ur5e.usda` after successful creation |


## OmniGraph details

Required stage contract:

```text
Robot wrapper:           /World/ur5e_
Articulation root joint: /World/ur5e_/root_joint
Zivid camera:            /World/Zivid/projector
TF parent:               /World/ur5e_/world
```

In Isaac Sim 5.1 the ROS 2 publishers read the robot and the camera themselves (Isaac Sim 6.x splits this into separate `IsaacReadJointState` and `IsaacComputeTransformTree` nodes, which 5.1 does not have):

```text
Joint states: ROS2PublishJointState.inputs:targetPrim     → /World/ur5e_/root_joint
Timestamps:   IsaacReadSimulationTime.outputs:simulationTime
              → ROS2PublishJointState.inputs:timeStamp, ROS2PublishClock.inputs:timeStamp
TF:           ROS2PublishTransformTree.inputs:parentPrim  → /World/ur5e_/world
              ROS2PublishTransformTree.inputs:targetPrims → /World/Zivid/projector
```

Zivid output defaults:

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

#### Robot, gripper and sensor models

The UR5e (Universal Robots), CRX-5iA (FANUC), Robotiq 2F-140 and Zivid models are converted from their official upstream descriptions and stay under their upstream licenses (BSD-3-Clause for UR and Zivid, Apache-2.0 for FANUC, BSD-2-Clause for Robotiq). Each package keeps the license text and its provenance in a `LICENSES/` folder; see `THIRD_PARTY_NOTICES.md`.

#### GearBox workpiece

assets/workpiece/GearBox/ contains modified and OpenUSD-converted assets derived from a CAD model obtained from the GrabCAD Library. The GearBox files are supplied **only to allow visualization of this repository's educational demonstration**. They are not licensed under the repository's MIT License.

The repository maintainers grant no licence, sublicence, reuse permission, modification permission, redistribution permission, or commercial-use permission for the GearBox CAD/USD assets. For any use other than visualization, the users must obtain and verify any necessary rights directly from the applicable upstream rights holder(s) and under the applicable upstream terms. See `assets/workpiece/GearBox/NOTICE.md`.
