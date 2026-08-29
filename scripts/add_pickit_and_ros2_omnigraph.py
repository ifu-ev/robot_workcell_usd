#!/usr/bin/env python3
# Isaac Sim 5.1 standalone graph builder.
# Example:
# ~/isaacsim/python.sh scripts/add_pickit_and_ros2_omnigraph_standalone.py \
#   --root /mnt/linux-data/projects/robot_workcell_usd --headless

import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description="Add ROS2 Pickit and MoveIt2 OmniGraphs to the portable gearbox scene.")
parser.add_argument("--root", required=True, help="Repository root containing scenes/gearbox_disassembly.usda")
parser.add_argument("--scene", default=None, help="Override scene path; default is <root>/scenes/gearbox_disassembly.usda")
parser.add_argument("--width", type=int, default=972, help="Zivid render-product width")
parser.add_argument("--height", type=int, default=600, help="Zivid render-product height")
parser.add_argument("--pickit", action=argparse.BooleanOptionalAction, default=True, help="Create Zivid RGB/pointcloud/camera-info and TF graphs")
parser.add_argument("--moveit", action=argparse.BooleanOptionalAction, default=True, help="Create joint-state, topic-controller, and clock graph")
parser.add_argument("--overwrite", action="store_true", help="Replace requested existing ROS2 graphs")
parser.add_argument("--headless", action="store_true", help="Launch Isaac Sim without a viewport")
args = parser.parse_args()

if not args.pickit and not args.moveit:
    parser.error("Select at least one of --pickit or --moveit.")
if args.width <= 0 or args.height <= 0:
    parser.error("--width and --height must be positive.")

PROJECT_ROOT = Path(args.root).expanduser().resolve()
SCENE_PATH = Path(args.scene).expanduser().resolve() if args.scene else PROJECT_ROOT / "scenes" / "gearbox_disassembly.usda"
if not SCENE_PATH.is_file():
    parser.error(f"Scene does not exist: {SCENE_PATH}")

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pxr import Sdf, Usd
import omni.graph.core as og
import omni.usd

ROBOT_PRIM = "/World/ur5e_"
JOINT_STATE_PRIM = "/World/ur5e_/root_joint"
CAMERA_PRIM = "/World/Zivid/projector"
TF_PARENT_PRIM = "/World/ur5e_/world"
ROS_ROOT = "/World/ROS2"
CTRL_GRAPH = f"{ROS_ROOT}/Topic_Based_Controller"
CAM_GRAPH = f"{ROS_ROOT}/Camera_Publisher"
TF_GRAPH = f"{ROS_ROOT}/TF_Publisher"


def fail(message):
    raise RuntimeError(message)


def open_scene(path):
    context = omni.usd.get_context()
    if not context.open_stage(str(path)):
        fail(f"Could not open stage: {path}")
    for _ in range(600):
        simulation_app.update()
        stage = context.get_stage()
        if stage is not None and stage.GetRootLayer().realPath == str(path):
            stage.Load()
            simulation_app.update()
            return stage
    fail(f"Timed out opening stage: {path}")


def edit_graph(path, nodes, connections, values=()):
    keys = og.Controller.Keys
    commands = {keys.CREATE_NODES: nodes, keys.CONNECT: connections}
    if values:
        commands[keys.SET_VALUES] = list(values)
    return og.Controller.edit({"graph_path": path, "evaluator_name": "execution"}, commands)


def set_rel(stage, prim_path, rel_name, targets):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        fail(f"Missing graph prim: {prim_path}")
    rel = prim.GetRelationship(rel_name)
    if not rel:
        fail(f"Missing relationship: {prim_path}.{rel_name}")
    rel.SetTargets([Sdf.Path(target) for target in targets])


def create_controller(stage):
    edit_graph(CTRL_GRAPH,
        [("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
         ("ros2_publish_joint_state", "isaacsim.ros2.bridge.ROS2PublishJointState"),
         ("ros2_subscribe_joint_state", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
         ("articulation_controller", "isaacsim.core.nodes.IsaacArticulationController"),
         ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
         ("ros2_qos_profile", "isaacsim.ros2.bridge.ROS2QoSProfile"),
         ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
         ("ros2_publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock")],
        [("on_playback_tick.outputs:tick", "ros2_publish_joint_state.inputs:execIn"),
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
         ("ros2_subscribe_joint_state.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand")],
        [("isaac_read_simulation_time.inputs:resetOnStop", True),
         ("articulation_controller.inputs:robotPath", JOINT_STATE_PRIM)])
    set_rel(stage, f"{CTRL_GRAPH}/ros2_publish_joint_state", "inputs:targetPrim", [JOINT_STATE_PRIM])
    set_rel(stage, f"{CTRL_GRAPH}/articulation_controller", "inputs:targetPrim", [ROBOT_PRIM])


def create_pickit(stage):
    edit_graph(CAM_GRAPH,
        [("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
         ("isaac_create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
         ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
         ("ros2_qos_profile", "isaacsim.ros2.bridge.ROS2QoSProfile"),
         ("ros2_camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
         ("ros2_camera_helper_02", "isaacsim.ros2.bridge.ROS2CameraHelper"),
         ("ros2_camera_info_helper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper")],
        [("on_playback_tick.outputs:tick", "isaac_create_render_product.inputs:execIn"),
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
         ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_info_helper.inputs:renderProductPath")],
        [("isaac_create_render_product.inputs:width", args.width),
         ("isaac_create_render_product.inputs:height", args.height),
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
         ("ros2_camera_info_helper.inputs:topicName", "ZividCamera/camera_info")])
    set_rel(stage, f"{CAM_GRAPH}/isaac_create_render_product", "inputs:cameraPrim", [CAMERA_PRIM])

    edit_graph(TF_GRAPH,
        [("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
         ("ros2_publish_transform_tree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
         ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
         ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
         ("ros2_qos_profile", "isaacsim.ros2.bridge.ROS2QoSProfile")],
        [("on_playback_tick.outputs:tick", "ros2_publish_transform_tree.inputs:execIn"),
         ("ros2_context.outputs:context", "ros2_publish_transform_tree.inputs:context"),
         ("ros2_qos_profile.outputs:qosProfile", "ros2_publish_transform_tree.inputs:qosProfile"),
         ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_transform_tree.inputs:timeStamp")])
    set_rel(stage, f"{TF_GRAPH}/ros2_publish_transform_tree", "inputs:parentPrim", [TF_PARENT_PRIM])
    set_rel(stage, f"{TF_GRAPH}/ros2_publish_transform_tree", "inputs:targetPrims", [CAMERA_PRIM])


try:
    stage = open_scene(SCENE_PATH)
    required = ([] if not args.moveit else [ROBOT_PRIM, JOINT_STATE_PRIM]) + ([] if not args.pickit else [CAMERA_PRIM, TF_PARENT_PRIM])
    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        fail("Missing required composed prim(s): " + ", ".join(missing))
    requested = ([] if not args.moveit else [CTRL_GRAPH]) + ([] if not args.pickit else [CAM_GRAPH, TF_GRAPH])
    existing = [path for path in requested if stage.GetPrimAtPath(path).IsValid()]
    if existing and not args.overwrite:
        fail("Existing graph(s) found; re-run with --overwrite to replace only: " + ", ".join(existing))

    root = stage.GetRootLayer()
    with Usd.EditContext(stage, Usd.EditTarget(root)):
        if not stage.GetPrimAtPath(ROS_ROOT).IsValid():
            stage.DefinePrim(ROS_ROOT, "Scope")
        for path in existing:
            stage.RemovePrim(path)
        if args.moveit:
            create_controller(stage)
        if args.pickit:
            create_pickit(stage)
        root.Save()

    print("=" * 100)
    print(f"SAVED ROS2 GRAPH LAYER: {SCENE_PATH}")
    print("Created:", ", ".join(requested))
    print(f"Camera resolution: {args.width}x{args.height}")
finally:
    simulation_app.close()
