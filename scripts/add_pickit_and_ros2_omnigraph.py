#!/usr/bin/env python3
# Isaac Sim 6.1 standalone ROS 2 OmniGraph builder.
# Example: $ISAACSIM_ROOT/python.sh scripts/add_pickit_and_ros2_omnigraph.py --root "$PWD" --headless

import argparse
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument("--root",required=True); p.add_argument("--scene")
p.add_argument("--width",type=int,default=972); p.add_argument("--height",type=int,default=600)
p.add_argument("--pickit",action=argparse.BooleanOptionalAction,default=True)
p.add_argument("--moveit",action=argparse.BooleanOptionalAction,default=True)
p.add_argument("--overwrite",action="store_true"); p.add_argument("--headless",action="store_true")
a=p.parse_args()
if not a.pickit and not a.moveit: p.error("Select --pickit and/or --moveit")
if a.width<=0 or a.height<=0: p.error("width and height must be positive")
root=Path(a.root).expanduser().resolve()
scene=Path(a.scene).expanduser().resolve() if a.scene else root/"scenes/gearbox_disassembly.usda"
if not scene.is_file(): p.error(f"Scene does not exist: {scene}")

from isaacsim import SimulationApp
app=SimulationApp({"headless":a.headless})
try:
 from isaacsim.core.utils.extensions import enable_extension
 enable_extension("isaacsim.ros2.bridge"); enable_extension("isaacsim.sensors.physics.nodes")
 app.update()
 from pxr import Sdf, Usd
 import omni.graph.core as og
 import omni.usd
 R="/World/ur5e_"; J=f"{R}/root_joint"; C="/World/Zivid/projector"; P=f"{R}/world"
 ROS="/World/ROS2"; CTRL=f"{ROS}/Topic_Based_Controller"; CAM=f"{ROS}/Camera_Publisher"; TF=f"{ROS}/TF_Publisher"
 def fail(x): raise RuntimeError(x)
 def g(path,nodes,links,vals=()):
  k=og.Controller.Keys; d={k.CREATE_NODES:nodes,k.CONNECT:links}
  if vals:d[k.SET_VALUES]=vals
  og.Controller.edit({"graph_path":path,"evaluator_name":"execution"},d)
 def rel(s,path,name,items):
  r=s.GetPrimAtPath(path).GetRelationship(name)
  if not r:fail(f"Missing relationship: {path}.{name}")
  r.SetTargets([Sdf.Path(x) for x in items])
 def controller(s):
  g(CTRL,[("tick","omni.graph.action.OnPlaybackTick"),("read","isaacsim.sensors.physics.IsaacReadJointState"),("pub","isaacsim.ros2.bridge.ROS2PublishJointState"),("sub","isaacsim.ros2.bridge.ROS2SubscribeJointState"),("ctrl","isaacsim.core.nodes.IsaacArticulationController"),("ctx","isaacsim.ros2.bridge.ROS2Context"),("qos","isaacsim.ros2.bridge.ROS2QoSProfile"),("time","isaacsim.core.nodes.IsaacReadSimulationTime"),("clock","isaacsim.ros2.bridge.ROS2PublishClock")],[("tick.outputs:tick","read.inputs:execIn"),("read.outputs:execOut","pub.inputs:execIn"),("tick.outputs:tick","sub.inputs:execIn"),("tick.outputs:tick","ctrl.inputs:execIn"),("tick.outputs:tick","clock.inputs:execIn"),("read.outputs:jointNames","pub.inputs:jointNames"),("read.outputs:jointPositions","pub.inputs:jointPositions"),("read.outputs:jointVelocities","pub.inputs:jointVelocities"),("read.outputs:jointEfforts","pub.inputs:jointEfforts"),("read.outputs:jointDofTypes","pub.inputs:jointDofTypes"),("read.outputs:stageMetersPerUnit","pub.inputs:stageMetersPerUnit"),("read.outputs:sensorTime","pub.inputs:sensorTime"),("ctx.outputs:context","pub.inputs:context"),("ctx.outputs:context","sub.inputs:context"),("ctx.outputs:context","clock.inputs:context"),("qos.outputs:qosProfile","pub.inputs:qosProfile"),("qos.outputs:qosProfile","sub.inputs:qosProfile"),("qos.outputs:qosProfile","clock.inputs:qosProfile"),("time.outputs:simulationTime","clock.inputs:timeStamp"),("sub.outputs:jointNames","ctrl.inputs:jointNames"),("sub.outputs:positionCommand","ctrl.inputs:positionCommand"),("sub.outputs:velocityCommand","ctrl.inputs:velocityCommand"),("sub.outputs:effortCommand","ctrl.inputs:effortCommand")],[("time.inputs:resetOnStop",True),("ctrl.inputs:robotPath",J)])
  rel(s,f"{CTRL}/read","inputs:prim",[J]); rel(s,f"{CTRL}/ctrl","inputs:targetPrim",[R])
 def camera_tf(s):
  g(CAM,[("tick","omni.graph.action.OnPlaybackTick"),("rp","isaacsim.core.nodes.IsaacCreateRenderProduct"),("ctx","isaacsim.ros2.bridge.ROS2Context"),("qos","isaacsim.ros2.bridge.ROS2QoSProfile"),("pcl","isaacsim.ros2.bridge.ROS2CameraHelper"),("rgb","isaacsim.ros2.bridge.ROS2CameraHelper"),("info","isaacsim.ros2.bridge.ROS2CameraInfoHelper")],[("tick.outputs:tick","rp.inputs:execIn"),("rp.outputs:execOut","pcl.inputs:execIn"),("rp.outputs:execOut","rgb.inputs:execIn"),("rp.outputs:execOut","info.inputs:execIn"),("ctx.outputs:context","pcl.inputs:context"),("ctx.outputs:context","rgb.inputs:context"),("qos.outputs:qosProfile","pcl.inputs:qosProfile"),("qos.outputs:qosProfile","rgb.inputs:qosProfile"),("qos.outputs:qosProfile","info.inputs:qosProfile"),("rp.outputs:renderProductPath","pcl.inputs:renderProductPath"),("rp.outputs:renderProductPath","rgb.inputs:renderProductPath"),("rp.outputs:renderProductPath","info.inputs:renderProductPath")],[("rp.inputs:width",a.width),("rp.inputs:height",a.height),("qos.inputs:depth",5),("qos.inputs:durability","transientLocal"),("qos.inputs:reliability","bestEffort"),("pcl.inputs:frameId","ZividCamera"),("pcl.inputs:topicName","ZividCamera/pointcloud"),("pcl.inputs:type","depth_pcl"),("rgb.inputs:frameId","ZividCamera"),("rgb.inputs:topicName","ZividCamera/rgb"),("rgb.inputs:type","rgb"),("info.inputs:frameId","ZividCamera"),("info.inputs:topicName","ZividCamera/camera_info")])
  rel(s,f"{CAM}/rp","inputs:cameraPrim",[C])
  g(TF,[("tick","omni.graph.action.OnPlaybackTick"),("compute","isaacsim.core.nodes.IsaacComputeTransformTree"),("pub","isaacsim.ros2.bridge.ROS2PublishTransformTree"),("time","isaacsim.core.nodes.IsaacReadSimulationTime"),("ctx","isaacsim.ros2.bridge.ROS2Context"),("qos","isaacsim.ros2.bridge.ROS2QoSProfile")],[("tick.outputs:tick","compute.inputs:execIn"),("compute.outputs:execOut","pub.inputs:execIn"),("compute.outputs:parentFrames","pub.inputs:parentFrames"),("compute.outputs:childFrames","pub.inputs:childFrames"),("compute.outputs:translations","pub.inputs:translations"),("compute.outputs:orientations","pub.inputs:orientations"),("ctx.outputs:context","pub.inputs:context"),("qos.outputs:qosProfile","pub.inputs:qosProfile"),("time.outputs:simulationTime","pub.inputs:timeStamp")])
  rel(s,f"{TF}/compute","inputs:parentPrim",[P]); rel(s,f"{TF}/compute","inputs:targetPrims",[C])
 ctx=omni.usd.get_context(); ctx.open_stage(str(scene))
 for _ in range(600):
  app.update(); s=ctx.get_stage()
  if s and Path(s.GetRootLayer().realPath).resolve()==scene: break
 else: fail(f"Timed out opening {scene}")
 s.Load(); app.update()
 needed=([R,J] if a.moveit else [])+([C,P] if a.pickit else [])
 missing=[x for x in needed if not s.GetPrimAtPath(x).IsValid()]
 if missing:fail("Missing required prim(s): "+", ".join(missing))
 requested=([CTRL] if a.moveit else [])+([CAM,TF] if a.pickit else [])
 existing=[x for x in requested if s.GetPrimAtPath(x).IsValid()]
 if existing and not a.overwrite:fail("Existing graph(s): "+", ".join(existing)+"; use --overwrite")
 layer=s.GetRootLayer()
 with Usd.EditContext(s,Usd.EditTarget(layer)):
  if not s.GetPrimAtPath(ROS).IsValid():s.DefinePrim(ROS,"Scope")
  for x in existing:s.RemovePrim(x)
  if a.moveit:controller(s)
  if a.pickit:camera_tf(s)
  layer.Save()
 print("="*90); print("SAVED ISAAC SIM 6.1 ROS2 GRAPHS:",scene); print("Created:",", ".join(requested))
finally:
 app.close()
