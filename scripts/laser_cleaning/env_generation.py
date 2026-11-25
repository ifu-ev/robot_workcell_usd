"""
Isaac Sim Script Editor (see README.md)
Laser-cleaning raycast test scene - ENVIRONMENT GENERATION

Builds the target side of the scene from scratch:

  1. References ASSET_PATH into TARGET_ROOT_PATH under a fresh Xform whose
     transform stack is exactly (translate, rotateX, scale) - a known,
     reproducible stack, unlike whatever a manual import leaves behind.
  2. Corrects a unit mismatch (source authored in mm -> stage in m) from
     the source's own metersPerUnit metadata.
  3. Optionally stands the part up: OBJ/CAD sources are often Y-up, and a
     reference carries no orientation fix of its own, so a flat part can
     arrive on its edge in a Z-up stage.
  4. Optionally places the part directly under the emitter at a chosen
     working distance, so the beam lands in-gate with no manual dragging.
  5. Creates the emitter as a child of the laser head, if missing.

IMPORTANT once the raycast script is running: move the ROOT prim
(TARGET_ROOT_PATH) only, never the mesh prim underneath it. The BVH is
built once in root-local space; shifting a child inside that subtree
leaves the BVH stale and produces MISS through solid geometry.

Re-running is safe: nothing is duplicated, only repositioned/rescaled.
"""

import omni.usd
import numpy as np
from pxr import Usd, UsdGeom, Sdf, Gf

# -----------------------------------------------------------------------------
# USER CONFIG
# -----------------------------------------------------------------------------
# Relative paths resolve against the open scene file (scenes/*.usda), absolute paths work too.
ASSET_PATH       = r"../assets/workpiece/FlyWheel/FlyWheel_Rusted.usd"
TARGET_ROOT_PATH = r"/World/targets/part"
EMITTER_PATH     = r"/World/Fanuc_CRX_5iA/root_joint/ee_link/Fanuc_Laser_Head/emitter"

LOAD_ASSET  = True      # True -> reference ASSET_PATH in; False -> validate only
HIDE_TARGET = False     # True -> MISS test (T5)

# --- Orientation -------------------------------------------------------------
# AUTO_STAND_UP: if the part's thinnest axis comes out as world X or Y, add a
# 90 deg rotation so it lies flat in a Z-up stage. Set False and use
# TARGET_ROTATE_X_DEG manually if the part isn't plate-like (a flywheel is).
AUTO_STAND_UP       = True
TARGET_ROTATE_X_DEG = 0.0      # starting value; AUTO_STAND_UP may override

# --- Placement ---------------------------------------------------------------
# AUTO_PLACE_UNDER_EMITTER: translate the part so its bbox top-centre sits
# directly below the emitter at PLACEMENT_STANDOFF_M. Set False to use
# TRANSLATE_TARGET_BY verbatim instead.
AUTO_PLACE_UNDER_EMITTER = True
PLACEMENT_STANDOFF_M     = 0.15               # working distance, must be in-gate
TRANSLATE_TARGET_BY      = Gf.Vec3d(0, 0, 0)  # used when AUTO_PLACE is False

# --- Emitter (child of the laser head, local frame) --------------------------
CREATE_EMITTER    = True
EMITTER_LOCAL_XYZ = Gf.Vec3d(0.0, 0.0, 0.180)
EMITTER_ROT_X_DEG = 0.0        # 0 -> beam fires along the head's local -Z

# --- Units -------------------------------------------------------------------
# None -> AUTO (read the asset's metersPerUnit). Set a float to force it,
# e.g. 0.001 for mm-authored CAD with no authored metadata.
UNIT_SCALE_OVERRIDE = None
# -----------------------------------------------------------------------------

ctx   = omni.usd.get_context()
stage = ctx.get_stage()
if stage is None:
    raise RuntimeError("No stage open. Open or create a stage first.")

# Uncomment when building a brand-new scene from scratch:
# UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
# UsdGeom.SetStageMetersPerUnit(stage, 1.0)


# --- Helpers -----------------------------------------------------------------
def world_bbox(prim):
    """World-space axis-aligned bbox of a prim, recomputed fresh each call."""
    return UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    ).ComputeWorldBound(prim).ComputeAlignedBox()


def bbox_report(prim, label):
    box = world_bbox(prim)
    if box.IsEmpty():
        print(f"[env] {label}: bbox EMPTY - no geometry?")
        return None
    bmin, bmax = box.GetMin(), box.GetMax()
    size = bmax - bmin
    cx, cy, top_z = 0.5 * (bmin[0] + bmax[0]), 0.5 * (bmin[1] + bmax[1]), bmax[2]
    print(f"[env] {label}: size=({size[0]:.4f}, {size[1]:.4f}, {size[2]:.4f}) m  "
          f"top-centre=({cx:.3f}, {cy:.3f}, {top_z:.3f})")
    return {"min": bmin, "max": bmax, "size": size, "cx": cx, "cy": cy, "top_z": top_z}


def get_asset_meters_per_unit(asset_path: str) -> float:
    """Open the source layer WITHOUT loading it into the scene, just to read
    its metersPerUnit. Relative paths are resolved against the open scene's
    root layer, the same way the reference below resolves them."""
    asset_path = stage.GetRootLayer().ComputeAbsolutePath(asset_path)
    try:
        src_stage = Usd.Stage.Open(asset_path)
    except Exception as exc:
        print(f"[env] WARNING: could not open {asset_path} to inspect units:\n      {exc}")
        return 1.0
    if src_stage is None:
        print(f"[env] WARNING: could not open {asset_path} to inspect units.")
        return 1.0
    mpu = UsdGeom.GetStageMetersPerUnit(src_stage)
    return mpu if mpu and mpu > 0 else 1.0


def get_ops(prim):
    """Return the (translate, rotateX, scale) ops of a prim, by type."""
    xf = UsdGeom.Xformable(prim)
    t = r = s = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            t = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeRotateX:
            r = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
            s = op
    return t, r, s


# --- Units -------------------------------------------------------------------
if UNIT_SCALE_OVERRIDE is not None:
    unit_scale = float(UNIT_SCALE_OVERRIDE)
    print(f"[env] Using UNIT_SCALE_OVERRIDE = {unit_scale}")
elif not LOAD_ASSET:
    unit_scale = 1.0
    print("[env] LOAD_ASSET=False -> part already on stage, skipping unit probe")
else:
    src_mpu = get_asset_meters_per_unit(ASSET_PATH)
    unit_scale = src_mpu / 1.0     # target stage is 1.0 m/unit
    print(f"[env] Detected source metersPerUnit = {src_mpu}  -> corrective scale = {unit_scale}")


# --- Target ------------------------------------------------------------------
if not stage.GetPrimAtPath("/World/targets").IsValid():
    UsdGeom.Xform.Define(stage, "/World/targets")
    print("[env] Created /World/targets")

target_prim = stage.GetPrimAtPath(TARGET_ROOT_PATH)
first_load  = not target_prim.IsValid()

if not LOAD_ASSET:
    if first_load:
        raise RuntimeError(
            f"[env] No prim at {TARGET_ROOT_PATH} and LOAD_ASSET=False.\n"
            f"      Set LOAD_ASSET = True to reference ASSET_PATH in from disk."
        )
    print(f"[env] Target found on stage: {TARGET_ROOT_PATH}")

elif first_load:
    # Ops are added in T, R, S order - USD requires this ordering, and the
    # raycast script relies on the whole subtree moving via this one prim.
    target_xf = UsdGeom.Xform.Define(stage, TARGET_ROOT_PATH)
    target_xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    target_xf.AddRotateXOp().Set(float(TARGET_ROTATE_X_DEG))
    target_xf.AddScaleOp().Set(Gf.Vec3f(unit_scale, unit_scale, unit_scale))
    target_prim = target_xf.GetPrim()
    target_prim.GetReferences().AddReference(ASSET_PATH)
    print(f"[env] Referenced asset:\n      {ASSET_PATH}\n      -> {TARGET_ROOT_PATH}")
    print(f"[env] Applied corrective scale = {unit_scale}")

else:
    print(f"[env] Reusing existing {TARGET_ROOT_PATH}")
    if not target_prim.HasAuthoredReferences():
        target_prim.GetReferences().AddReference(ASSET_PATH)
        print("[env] Re-attached missing reference.")
    t_op, r_op, s_op = get_ops(target_prim)
    xf = UsdGeom.Xformable(target_prim)
    if t_op is None:
        t_op = xf.AddTranslateOp()
    if r_op is None:
        r_op = xf.AddRotateXOp()
    if s_op is None:
        s_op = xf.AddScaleOp()
    r_op.Set(float(TARGET_ROTATE_X_DEG))
    s_op.Set(Gf.Vec3f(unit_scale, unit_scale, unit_scale))

translate_op, rotate_op, scale_op = get_ops(target_prim)

info = bbox_report(target_prim, "After reference")
if info is not None and (info["size"][0] > 5.0 or info["size"][2] > 5.0):
    print("[env] WARNING: bbox looks too large (>5 m). Try UNIT_SCALE_OVERRIDE = 0.001")
    print("      (mm-authored CAD with no metersPerUnit metadata).")


# --- Orientation: stand the part up if it arrived on its edge ----------------
if LOAD_ASSET and AUTO_STAND_UP and info is not None:
    size = np.array([info["size"][0], info["size"][1], info["size"][2]])
    thin = int(np.argmin(size))
    if thin == 2:
        print("[env] Orientation OK - thinnest axis is world Z (part lies flat).")
    elif thin == 1:
        print("[env] Thinnest axis is world Y -> rotating 90 deg about X to stand the part flat.")
        rotate_op.Set(90.0)
        info = bbox_report(target_prim, "After stand-up")
    else:
        print("[env] Thinnest axis is world X. A rotate about X won't fix that -")
        print("      add a rotateY op manually, or set AUTO_STAND_UP=False and")
        print("      orient the source asset instead.")


# --- Emitter: child of the laser head, placed in the HEAD's local frame ------
emitter_prim = stage.GetPrimAtPath(EMITTER_PATH)
if CREATE_EMITTER:
    parent_path = Sdf.Path(EMITTER_PATH).GetParentPath()
    if not stage.GetPrimAtPath(parent_path).IsValid():
        raise RuntimeError(
            f"[env] Emitter parent {parent_path} does not exist.\n"
            f"      Check EMITTER_PATH against the robot's actual prim names."
        )
    if not emitter_prim.IsValid():
        exf = UsdGeom.Xform.Define(stage, EMITTER_PATH)
        exf.AddTranslateOp().Set(EMITTER_LOCAL_XYZ)
        exf.AddRotateXOp().Set(EMITTER_ROT_X_DEG)
        emitter_prim = exf.GetPrim()
        print(f"[env] Created {EMITTER_PATH}  local={tuple(EMITTER_LOCAL_XYZ)}  rotX={EMITTER_ROT_X_DEG}")
    else:
        e_t, e_r, _ = get_ops(emitter_prim)
        exf = UsdGeom.Xformable(emitter_prim)
        if e_t is None:
            e_t = exf.AddTranslateOp()
        if e_r is None:
            e_r = exf.AddRotateXOp()
        e_t.Set(EMITTER_LOCAL_XYZ)
        e_r.Set(EMITTER_ROT_X_DEG)
        print(f"[env] Repositioned {EMITTER_PATH}  local={tuple(EMITTER_LOCAL_XYZ)}  rotX={EMITTER_ROT_X_DEG}")

if not emitter_prim.IsValid():
    raise RuntimeError(f"[env] No emitter at {EMITTER_PATH}. Set CREATE_EMITTER = True.")

M_e     = UsdGeom.Xformable(emitter_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
o_world = Gf.Vec3d(M_e.ExtractTranslation())
d_world = Gf.Vec3d(M_e.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()
print(f"[env] Emitter world origin : ({o_world[0]:.4f}, {o_world[1]:.4f}, {o_world[2]:.4f})")
print(f"[env] Beam direction (+Z)  : ({d_world[0]:.3f}, {d_world[1]:.3f}, {d_world[2]:.3f})")


# --- Placement ---------------------------------------------------------------
if AUTO_PLACE_UNDER_EMITTER and info is not None:
    if d_world[2] > -0.9:
        print("[env] WARNING: beam is not pointing (mostly) straight down. Auto-placement")
        print("      assumes a vertical beam - the part will be centred below the emitter")
        print("      regardless, which may not be where the beam actually lands.")
    want  = Gf.Vec3d(o_world[0], o_world[1], o_world[2] - PLACEMENT_STANDOFF_M)
    delta = Gf.Vec3d(want[0] - info["cx"], want[1] - info["cy"], want[2] - info["top_z"])
    cur   = translate_op.Get() or Gf.Vec3d(0, 0, 0)
    translate_op.Set(cur + delta)
    new = translate_op.Get()
    print(f"[env] Auto-placed under emitter: translate {tuple(cur)} -> {tuple(new)}")
    print(f"      (top face {PLACEMENT_STANDOFF_M} m below the emitter)")
    print(f"[env] Reproducible placement -> TRANSLATE_TARGET_BY = "
          f"Gf.Vec3d({new[0]:.6f}, {new[1]:.6f}, {new[2]:.6f})")
    info = bbox_report(target_prim, "After placement")
elif not AUTO_PLACE_UNDER_EMITTER:
    translate_op.Set(TRANSLATE_TARGET_BY)
    print(f"[env] Target translate set to {TRANSLATE_TARGET_BY}")
    info = bbox_report(target_prim, "After placement")


# --- Visibility --------------------------------------------------------------
img = UsdGeom.Imageable(target_prim)
if HIDE_TARGET:
    img.MakeInvisible()
    print("[env] Target HIDDEN (T5 configuration)")
else:
    img.MakeVisible()


# --- Final aiming check ------------------------------------------------------
print("\n[env] Scene ready.")
print(f"      Target  : {TARGET_ROOT_PATH}  (hidden={HIDE_TARGET})  scale={unit_scale}")
print(f"      Emitter : {EMITTER_PATH}")

if info is not None:
    lateral = ((o_world[0] - info["cx"]) ** 2 + (o_world[1] - info["cy"]) ** 2) ** 0.5
    height  = o_world[2] - info["top_z"]
    print(f"      Lateral offset   : {lateral:.4f} m  (want < 0.025 for full footprint)")
    print(f"      Height above top : {height:.4f} m  (want inside the raycast gate)")
    if lateral > 0.025:
        print("[env] -> Laterally off the part; expect MISS.")
    else:
        print("[env] -> Positioned for a hit. Note a flywheel has spoke gaps and a")
        print("         possible centre bore - a dead-centre ray may legitimately pass")
        print("         through. Offset onto solid rim if the centre reports MISS.")

print("\n      Reminder: once the raycast script is running, move ONLY")
print(f"      {TARGET_ROOT_PATH} - never the mesh prim inside it, or the")
print("      BVH goes stale and reports MISS through solid geometry.")
print("      Now run the raycast script to start the live loop.\n")
