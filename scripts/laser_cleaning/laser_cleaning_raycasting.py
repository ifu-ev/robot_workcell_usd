"""
Isaac Sim Script Editor (see README.md)
Laser-cleaning raycast + PAINT (local-space BVH + multi-ray footprint + per-face color)

Assumes env_generation.py has already been run (target referenced and
placed, emitter created under the laser head).

Design notes:
  - Per-face displayColor painting, not UV texture painting: CAD assets
    rarely carry a usable unwrap, and a fine mesh gives good resolution
    for free. Colors live on the mesh's own local-space primvar, so they
    travel with the part when it moves (conveyor motion) for free.
  - A single center ray under-samples the real laser's ~50 mm footprint,
    so an N x N grid of parallel sub-rays is fired in ONE batched Warp
    launch. Occluded faces behind folds are correctly NOT painted, unlike
    a cheap radius query.
  - Every sub-ray is gated on its OWN hit distance, so the footprint paints
    wherever any part of it is over the part, even when the centre ray is
    over a hole (e.g. a bore). The centre ray only drives the beam line.
  - DOSE_MODEL (default): time-accurate cleaning (README.md, "Dose model").
    While the laser is enabled (/laser/enable, e.g. from the robot program),
    every face that is the first surface along the beam and whose centroid
    lies inside the square scan pattern gains dose = irradiance * sim-time step * efficiency, the
    efficiency falling with defocus and incidence. A face is clean once its
    dose covers the layer energy; dose far beyond it is flagged as
    overexposure. DOSE_MODEL = False keeps the instant paint-on-hit preview.
  - Grid is FIXED-ORIENTATION relative to the emitter (parallel rays,
    constant footprint regardless of incidence angle).
  - The BVH is built ONCE in the target's LOCAL space; the ray is
    transformed into that space fresh every tick from the part's live
    world transform. That is what makes a moving part work with no
    rebuilds.

IMPORTANT: move the ROOT prim (TARGET_ROOT_PATH) only, never the mesh
prim underneath it, while this script is running. The BVH is built in
root-local space, so shifting a child inside that subtree leaves the BVH
stale and produces MISS through solid geometry. A staleness guard below
detects this and rebuilds, but avoiding it is cheaper.

Controls: Pause / Resume / STOP / Clear Paint / Aim Report in the
on-screen window.
"""

import omni.usd
import omni.kit.app
import omni.timeline
import omni.ui as ui
import warp as wp
import numpy as np
import builtins
from pxr import Usd, UsdGeom, UsdShade, Gf, Vt, Sdf

# -----------------------------------------------------------------------------
# USER CONFIG
# -----------------------------------------------------------------------------
TARGET_ROOT_PATH = "/World/targets/part"
EMITTER_PATH     = "/World/Fanuc_CRX_5iA/root_joint/ee_link/Fanuc_Laser_Head/emitter"

WORKING_DIST_MIN = 0.10          # metres
WORKING_DIST_MAX = 0.50
RAY_MAX_T        = 5.0           # metres (converted to local units per tick)
TICK_HZ          = 25

# --- Paint (per-face displayColor) config -------------------------------------
PAINT_ENABLED    = True
FOOTPRINT_SIZE_M = 0.05          # 50 mm x 50 mm laser footprint
GRID_N           = 21            # 21x21 = 441 parallel sub-rays per tick (2.5 mm apart)
CLEAN_COLOR      = (0.55, 0.50, 0.45)   # color applied to cleaned faces
DIRTY_COLOR      = (0.66, 0.15, 0.03)   # initial/unpainted face color

# --- Laser dose model (README.md, "Dose model") -------------------------------
# 1 kW / 50 mJ ns fiber laser, 2D square scan pattern of FOOTPRINT_SIZE_M,
# 254 mm field lens (spot and Rayleigh-like range scaled from the laser's
# 210 mm lens values by 254/210). Heavy rust, conservative rate.
DOSE_MODEL           = True
LASER_POWER_W        = 1000.0
FOCAL_DIST_M         = 0.254     # emitter (lens) to focus
F0_J_CM2             = 3.0       # peak per-pulse fluence at focus
ZR_M                 = 0.0365    # defocus at which the fluence halves
F_TH_J_CM2           = 1.5       # ablation threshold (rust)
MAX_INCIDENCE_DEG    = 70.0      # steeper than this -> no irradiation
LAYER_ENERGY_J_CM2   = 120.0     # heavy rust: 0.12 s/cm2 at 1 kW
OVEREXPOSE_J_CM2     = 150.0     # metal dose beyond the layer flagged as overexposure
OVEREXPOSED_COLOR    = (0.15, 0.35, 0.85)
LASER_ENABLE_TOPIC   = "/laser/enable"   # std_msgs/Bool; "" -> laser always on
COLOR_UPDATE_S       = 0.25      # wall seconds between colour writes (dose integrates every tick)
VISIBILITY_STEP_M    = 0.001     # ray spacing for the first-surface (visibility) map

# --- Material handling --------------------------------------------------------
# displayColor is USD's FALLBACK color: the renderer honours it only when a
# mesh has no bound material, or when the bound material explicitly reads it
# through a primvar reader.
#
# UNBIND_TARGET_MATERIAL = True  -> unbind the mesh's material outright. This
#   is the reliable route for MDL/OmniPBR materials (typical of OBJ/HOOPS
#   imports), which do NOT honour a UsdPrimvarReader connection. The part
#   loses its CAD appearance and renders entirely in DIRTY/CLEAN color.
#   Non-destructive: the Looks scope stays on the stage, and re-referencing
#   the asset restores the binding.
#
# UNBIND_TARGET_MATERIAL = False -> try to wire displayColor into whatever
#   material is bound. Works for UsdPreviewSurface; may silently do nothing
#   on MDL-only materials.
UNBIND_TARGET_MATERIAL = True

# --- Staleness guard ----------------------------------------------------------
# Detect a sub-mesh being moved relative to the target root after the BVH was
# built (e.g. dragging the mesh prim instead of the root) and rebuild rather
# than silently raycasting against stale geometry.
CHECK_SUBMESH_DRIFT = True
DRIFT_TOLERANCE     = 1e-6
# -----------------------------------------------------------------------------

# --- Teardown previous subscription / window if re-running -------------------
if getattr(builtins, "_laser_test_sub", None) is not None:
    builtins._laser_test_sub.unsubscribe()
    builtins._laser_test_sub = None
    print("[teardown] Previous update subscription removed.")

if getattr(builtins, "_laser_test_window", None) is not None:
    builtins._laser_test_window.destroy()
    builtins._laser_test_window = None
    print("[teardown] Previous control window destroyed.")

if getattr(builtins, "_laser_ros_node", None) is not None:
    builtins._laser_ros_node.destroy_node()
    builtins._laser_ros_node = None
    print("[teardown] Previous laser-enable subscriber removed.")

ctx   = omni.usd.get_context()
stage = ctx.get_stage()
if stage is None:
    raise RuntimeError("No stage open. Run env_generation.py first.")

root_prim = stage.GetPrimAtPath(TARGET_ROOT_PATH)
if not root_prim.IsValid():
    raise RuntimeError(f"Target root not found at {TARGET_ROOT_PATH}. Run env_generation.py first.")
if not stage.GetPrimAtPath(EMITTER_PATH).IsValid():
    raise RuntimeError(f"Emitter not found at {EMITTER_PATH}. Run env_generation.py first.")

# --- Debug draw interface (best-effort, multiple namespace fallbacks) --------
_draw = None
try:
    from omni.isaac.debug_draw import _debug_draw
    _draw = _debug_draw.acquire_debug_draw_interface()
    print("[debug_draw] Using omni.isaac.debug_draw")
except ImportError:
    try:
        from isaacsim.util.debug_draw import _debug_draw
        _draw = _debug_draw.acquire_debug_draw_interface()
        print("[debug_draw] Using isaacsim.util.debug_draw")
    except ImportError:
        print("[debug_draw] WARNING: no debug_draw extension found.")
        print("             Enable 'omni.isaac.debug_draw' via Window > Extensions to see the beam line.")


# --- Startup aiming report ----------------------------------------------------
def print_aiming_report():
    """Report where the emitter is relative to the target in the two terms
    that decide HIT vs MISS: lateral offset from the part's top-centre, and
    height above its top face. A dot-product 'alignment' is misleading when
    the beam is vertical but laterally displaced."""
    e_prim = stage.GetPrimAtPath(EMITTER_PATH)
    M_e = UsdGeom.Xformable(e_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    o   = Gf.Vec3d(M_e.ExtractTranslation())
    d   = Gf.Vec3d(M_e.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()

    bb = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    ).ComputeWorldBound(root_prim).ComputeAlignedBox()

    print(f"[aim] Emitter origin : ({o[0]:.4f}, {o[1]:.4f}, {o[2]:.4f})")
    print(f"[aim] Beam direction : ({d[0]:.3f}, {d[1]:.3f}, {d[2]:.3f})")
    if bb.IsEmpty():
        print("[aim] Target bbox empty - cannot report offsets.")
        return
    bmin, bmax = bb.GetMin(), bb.GetMax()
    cx, cy, top_z = 0.5 * (bmin[0] + bmax[0]), 0.5 * (bmin[1] + bmax[1]), bmax[2]
    lateral = ((o[0] - cx) ** 2 + (o[1] - cy) ** 2) ** 0.5
    height  = o[2] - top_z
    print(f"[aim] Target top-centre : ({cx:.3f}, {cy:.3f}, {top_z:.3f})")
    print(f"[aim] Lateral offset    : {lateral:.4f} m   "
          f"(want < {FOOTPRINT_SIZE_M * 0.5:.3f} for full footprint coverage)")
    print(f"[aim] Height above top  : {height:.4f} m   "
          f"(want {WORKING_DIST_MIN:.2f} - {WORKING_DIST_MAX:.2f} for in-gate)")
    if lateral > FOOTPRINT_SIZE_M * 0.5:
        print("[aim] -> Laterally off the part. Expect MISS until the robot is jogged")
        print("         over it (or the part is moved under the head).")
    elif not (WORKING_DIST_MIN <= height <= WORKING_DIST_MAX):
        print("[aim] -> Over the part but outside the working-distance gate. Expect")
        print("         HIT-OUT-OF-GATE (yellow beam), no painting.")
    else:
        print("[aim] -> Positioned for HIT-IN-GATE. Painting should start immediately.")
        print("         (A part with holes - spoke gaps, a centre bore - can still")
        print("          legitimately report MISS if the beam drops through one.)")

print_aiming_report()

# --- Warp init ----------------------------------------------------------------
wp.init()


@wp.kernel
def kernel_query_ray_single(
    mesh_id : wp.uint64,
    origin  : wp.vec3,
    direction: wp.vec3,
    max_t   : float,
    out_result: wp.array(dtype=wp.int32),
    out_t     : wp.array(dtype=float),
    out_px    : wp.array(dtype=float),
    out_py    : wp.array(dtype=float),
    out_pz    : wp.array(dtype=float),
    out_nx    : wp.array(dtype=float),
    out_ny    : wp.array(dtype=float),
    out_nz    : wp.array(dtype=float),
):
    q = wp.mesh_query_ray(mesh_id, origin, direction, max_t)
    if q.result:
        out_result[0] = wp.int32(1)
        # NOTE: wp.mesh_eval_position is DEVICE-ONLY - it does not exist on
        # the host-side warp module. It must be called inside the kernel.
        pos = wp.mesh_eval_position(mesh_id, q.face, q.u, q.v)
        out_px[0] = pos[0]
        out_py[0] = pos[1]
        out_pz[0] = pos[2]
    else:
        out_result[0] = wp.int32(0)
        out_px[0] = float(0.0)
        out_py[0] = float(0.0)
        out_pz[0] = float(0.0)
    out_t[0]  = q.t
    out_nx[0] = q.normal[0]
    out_ny[0] = q.normal[1]
    out_nz[0] = q.normal[2]


@wp.kernel
def kernel_query_ray_grid(
    mesh_id : wp.uint64,
    origins : wp.array(dtype=wp.vec3),
    direction: wp.vec3,
    max_t   : float,
    out_result: wp.array(dtype=wp.int32),
    out_face  : wp.array(dtype=wp.int32),
    out_t     : wp.array(dtype=float),
):
    tid = wp.tid()
    q = wp.mesh_query_ray(mesh_id, origins[tid], direction, max_t)
    # NOTE: ternary expressions (x if c else y) are NOT supported inside
    # @wp.kernel - they raise WarpCodegenError. Use full if/else.
    if q.result:
        out_result[tid] = wp.int32(1)
        out_face[tid]   = q.face
        out_t[tid]      = q.t
    else:
        out_result[tid] = wp.int32(0)
        out_face[tid]   = wp.int32(-1)
        out_t[tid]      = float(-1.0)


_buf_result = wp.zeros(1, dtype=wp.int32,  device="cuda:0")
_buf_t      = wp.zeros(1, dtype=float,     device="cuda:0")
_buf_px     = wp.zeros(1, dtype=float,     device="cuda:0")
_buf_py     = wp.zeros(1, dtype=float,     device="cuda:0")
_buf_pz     = wp.zeros(1, dtype=float,     device="cuda:0")
_buf_nx     = wp.zeros(1, dtype=float,     device="cuda:0")
_buf_ny     = wp.zeros(1, dtype=float,     device="cuda:0")
_buf_nz     = wp.zeros(1, dtype=float,     device="cuda:0")

_N_GRID_RAYS     = GRID_N * GRID_N
_buf_grid_result = wp.zeros(_N_GRID_RAYS, dtype=wp.int32, device="cuda:0")
_buf_grid_face   = wp.zeros(_N_GRID_RAYS, dtype=wp.int32, device="cuda:0")
_buf_grid_t      = wp.zeros(_N_GRID_RAYS, dtype=float,    device="cuda:0")


def fan_triangulate(counts, indices):
    tris = []
    idx  = 0
    for n in counts:
        v0 = indices[idx]
        for k in range(1, n - 1):
            tris += [v0, indices[idx + k], indices[idx + k + 1]]
        idx += n
    return tris


def find_mesh_prims_instance_aware(root_prim):
    for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh):
            yield prim


# -- Per-mesh paint state -----------------------------------------------------
# For each sub-mesh: cache of its current face colors (numpy, [n_faces,3]),
# the live UsdGeom.Mesh schema + displayColor attr, a dirty flag so we only
# author back to USD when something actually changed this tick, and the
# mesh-relative-to-root matrix captured at build time for the drift guard.
class MeshPaintState:
    __slots__ = ("mesh_prim", "mesh_schema", "color_attr", "colors",
                 "dirty", "n_faces", "M_rel_at_build",
                 "dose", "visible", "centroid", "normal", "area")

    def __init__(self, mesh_prim, mesh_schema, color_attr, n_faces, M_rel_at_build):
        self.mesh_prim      = mesh_prim
        self.mesh_schema    = mesh_schema
        self.color_attr     = color_attr
        self.n_faces        = n_faces
        self.colors         = np.tile(np.array(DIRTY_COLOR, dtype=np.float32), (n_faces, 1))
        self.dirty          = True
        self.M_rel_at_build = M_rel_at_build
        # Dose model, per face, root-local geometry (set in build_warp_mesh_local)
        self.dose     = np.zeros(n_faces)                  # J/cm2
        self.visible  = np.zeros(n_faces, dtype=bool)      # first surface along the beam
        self.centroid = np.zeros((n_faces, 3))
        self.normal   = np.zeros((n_faces, 3))
        self.area     = np.zeros(n_faces)                  # root-local units^2


def handle_mesh_material(mesh_prim):
    """
    Make displayColor actually render on this mesh.

    Case A - UNBIND_TARGET_MATERIAL: unbind whatever material is bound. USD
      then uses displayColor as its native fallback. Reliable for MDL.

    Case B - wire a UsdPrimvarReader_float3 into the bound material:
      B1. No material bound          -> nothing to do.
      B2. Shaders read from a shared Material.diffuseColor input -> one
          connection there fixes PreviewSurface + MDL paths at once.
      B3. Shaders hold their own static diffuse input (typical of OBJ/HOOPS
          imports) -> connect that shader input directly.

    NOTE: UsdShade getters return INVALID objects, not None, when an input
    is absent. Test them for truthiness - `is not None` passes on an invalid
    object and the next call throws UsdExpiredPrimAccessError.

    Idempotent - safe to call on every script run.
    """
    binding_api = UsdShade.MaterialBindingAPI(mesh_prim)

    if UNBIND_TARGET_MATERIAL:
        mat, _ = binding_api.ComputeBoundMaterial()
        if mat:
            binding_api.UnbindAllBindings()
            print(f"[paint] Unbound material on {mesh_prim.GetPath()} - "
                  f"displayColor now renders natively.")
        else:
            print(f"[paint] No material bound on {mesh_prim.GetPath()} - "
                  f"displayColor renders natively.")
        return

    mat, _ = binding_api.ComputeBoundMaterial()
    if not mat:
        print(f"[paint] No material bound on {mesh_prim.GetPath()} - "
              f"displayColor will render natively, no wiring needed.")
        return

    mat_prim    = mat.GetPrim()
    reader_path = mat_prim.GetPath().AppendChild("DisplayColorReader")
    stage_local = mesh_prim.GetStage()
    reader_prim = stage_local.GetPrimAtPath(reader_path)

    if not reader_prim.IsValid():
        reader = UsdShade.Shader.Define(stage_local, reader_path)
        reader.CreateIdAttr("UsdPrimvarReader_float3")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
        reader.CreateInput("fallback", Sdf.ValueTypeNames.Float3).Set((0.5, 0.5, 0.5))
        reader_out = reader.CreateOutput("result", Sdf.ValueTypeNames.Float3)
        print(f"[paint] Created displayColor primvar reader at {reader_path}")
    else:
        reader = UsdShade.Shader(reader_prim)
        reader_out = reader.GetOutput("result")

    wired_any = False

    # --- B2: shared Material-level diffuseColor input ---
    diffuse_input = mat.GetInput("diffuseColor")
    if diffuse_input:
        already = diffuse_input.HasConnectedSource() and \
            diffuse_input.GetConnectedSource()[0].GetPrim().GetPath() == reader_path
        if already:
            print(f"[paint] {mat_prim.GetPath()}.diffuseColor already wired to displayColor reader")
            return
        diffuse_input.ConnectToSource(reader_out)
        wired_any = True
        print(f"[paint] Connected {mat_prim.GetPath()}.diffuseColor <- {reader_path}.result")

    # --- B3: shader-level static diffuse input ---
    mdl_only = True
    for output in mat.GetSurfaceOutputs():
        src = output.GetConnectedSource()
        if not src:
            continue
        shader = UsdShade.Shader(src[0].GetPrim())
        if shader.GetIdAttr().Get() == "UsdPreviewSurface":
            mdl_only = False
        for name in ("diffuseColor", "diffuse_color_constant", "base_color", "baseColor"):
            inp = shader.GetInput(name)
            if inp and not inp.HasConnectedSource():
                inp.ConnectToSource(reader_out)
                wired_any = True
                print(f"[paint] Connected {shader.GetPath()}.{name} <- {reader_path}.result")

    if not wired_any:
        print(f"[paint] WARNING: no diffuse input found to wire on {mat_prim.GetPath()}. "
              f"displayColor may not be visible.")
    elif mdl_only:
        print(f"[paint] NOTE: {mat_prim.GetPath()} appears to be MDL-only. MDL shaders")
        print(f"        generally ignore UsdPrimvarReader connections. If no color change")
        print(f"        is visible, set UNBIND_TARGET_MATERIAL = True and re-run.")


def build_warp_mesh_local(root_prim):
    """
    Builds the merged LOCAL-space BVH, and a global-triangle-id ->
    (mesh_index, local face_index) map so a hit on the merged BVH can be
    routed back to the right sub-mesh's own displayColor primvar. Also
    initializes/reuses a 'uniform' displayColor primvar (one color per
    polygon face) on every sub-mesh found.
    """
    mesh_prims = list(find_mesh_prims_instance_aware(root_prim))
    if not mesh_prims:
        print(f"[bvh] ERROR: no UsdGeom.Mesh prims found under {root_prim.GetPath()}")
        return None, [], [], []

    M_root   = UsdGeom.Xformable(root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    Root_inv = M_root.GetInverse()

    all_pts  = []
    all_tris = []
    tri_owner_mesh = []   # global tri index -> index into mesh_prims/paint_states
    tri_owner_face = []   # global tri index -> local face index within that mesh
    paint_states = []
    vert_offset  = 0
    total_meshes = 0

    for mesh_idx, mesh_prim in enumerate(mesh_prims):
        mesh_schema = UsdGeom.Mesh(mesh_prim)
        pts_attr = mesh_schema.GetPointsAttr()
        if not pts_attr or pts_attr.Get() is None:
            paint_states.append(None)
            continue
        pts_local_mesh = np.array(pts_attr.Get(), dtype=np.float64)
        counts  = mesh_schema.GetFaceVertexCountsAttr().Get()
        raw_idx = mesh_schema.GetFaceVertexIndicesAttr().Get()
        if not counts or not raw_idx:
            paint_states.append(None)
            continue

        n_faces = len(counts)

        # --- displayColor primvar (uniform: one color per polygon face) ---
        pv_api = UsdGeom.PrimvarsAPI(mesh_prim)
        color_pv = pv_api.GetPrimvar("displayColor")
        needs_init = True
        if color_pv and color_pv.HasAuthoredValue():
            existing = color_pv.Get()
            if existing is not None and len(existing) == n_faces \
                    and color_pv.GetInterpolation() == UsdGeom.Tokens.uniform:
                needs_init = False
        if not color_pv or needs_init:
            color_pv = mesh_schema.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform)
            color_pv.Set(Vt.Vec3fArray([Gf.Vec3f(*DIRTY_COLOR)] * n_faces))
            print(f"[paint] Initialized displayColor (uniform, {n_faces} faces) on {mesh_prim.GetPath()}")
        else:
            print(f"[paint] Reusing existing displayColor ({n_faces} faces) on {mesh_prim.GetPath()}")

        handle_mesh_material(mesh_prim)

        # --- BVH geometry ---
        tris = fan_triangulate(counts, raw_idx)
        if not tris:
            paint_states.append(None)
            continue

        # Per-output-triangle -> source-polygon-face map. Fan triangulation
        # of a triangle yields 1 output triangle with the SAME face index;
        # an n-gon yields n-2 triangles all mapped to that same face index.
        tri_src_face = []
        for face_i, n in enumerate(counts):
            for _ in range(n - 2):
                tri_src_face.append(face_i)

        M_mesh = UsdGeom.Xformable(mesh_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        M_rel  = M_mesh * Root_inv
        M_np   = np.array(M_rel, dtype=np.float64).reshape(4, 4)

        pstate = MeshPaintState(mesh_prim, mesh_schema, color_pv, n_faces, M_np.copy())
        if not needs_init:
            pstate.colors = np.array(color_pv.Get(), dtype=np.float32)
        paint_states.append(pstate)

        pts_h = np.hstack([pts_local_mesh, np.ones((len(pts_local_mesh), 1))])
        pts_root_local = (pts_h @ M_np)[:, :3]

        # Per-face centroid, unit normal and area for the dose model.
        cnt = np.asarray(counts)
        fidx = np.asarray(raw_idx)
        starts = np.concatenate([[0], np.cumsum(cnt)[:-1]])
        pstate.centroid = np.add.reduceat(pts_root_local[fidx], starts, axis=0) / cnt[:, None]
        tri_arr = np.asarray(tris).reshape(-1, 3)
        p0, p1, p2 = (pts_root_local[tri_arr[:, i]] for i in range(3))
        area_vec = np.zeros((n_faces, 3))
        np.add.at(area_vec, np.asarray(tri_src_face), np.cross(p1 - p0, p2 - p0))
        norm = np.linalg.norm(area_vec, axis=1)
        pstate.normal = area_vec / np.maximum(norm, 1e-15)[:, None]
        pstate.area = 0.5 * norm

        all_pts.append(pts_root_local.astype(np.float32))
        all_tris.extend([i + vert_offset for i in tris])
        tri_owner_mesh.extend([mesh_idx] * len(tri_src_face))
        tri_owner_face.extend(tri_src_face)
        vert_offset += len(pts_local_mesh)
        total_meshes += 1

    if not all_pts:
        print("[bvh] ERROR: mesh prims found but none had valid points/topology.")
        return None, [], [], []

    merged_pts  = np.concatenate(all_pts, axis=0)
    merged_tris = np.array(all_tris, dtype=np.int32)

    wp_pts  = wp.array(merged_pts,  dtype=wp.vec3,  device="cuda:0")
    wp_tris = wp.array(merged_tris, dtype=wp.int32, device="cuda:0")
    mesh    = wp.Mesh(points=wp_pts, indices=wp_tris)
    print(f"[bvh] Built LOCAL-space BVH: {total_meshes} sub-mesh(es), "
          f"{len(merged_pts)} verts, {len(merged_tris)//3} tris <- {root_prim.GetPath()}")
    print(f"[bvh] Local bbox of merged points: min={merged_pts.min(axis=0)}  max={merged_pts.max(axis=0)}")
    return mesh, tri_owner_mesh, tri_owner_face, paint_states


def rebuild_bvh():
    """Rebuild the BVH and refresh every derived lookup array in one place,
    so no caller can leave the routing arrays stale."""
    global _warp_mesh, _tri_owner_mesh_np, _tri_owner_face_np, _paint_states, _vis_key
    mesh, owner_mesh, owner_face, pstates = build_warp_mesh_local(root_prim)
    _warp_mesh = mesh
    _tri_owner_mesh_np = np.array(owner_mesh, dtype=np.int32) if owner_mesh else np.empty(0, dtype=np.int32)
    _tri_owner_face_np = np.array(owner_face, dtype=np.int32) if owner_face else np.empty(0, dtype=np.int32)
    _paint_states = pstates
    _vis_key = None                     # visibility must be rebuilt for the new BVH
    return mesh is not None


_warp_mesh = None
_tri_owner_mesh_np = np.empty(0, dtype=np.int32)
_tri_owner_face_np = np.empty(0, dtype=np.int32)
_paint_states = []
_vis_key = None                         # beam direction the visibility map was built for
rebuild_bvh()


def submesh_drift_detected(Winv):
    """True if any sub-mesh has moved relative to the target root since the
    BVH was built - i.e. someone dragged the mesh prim instead of the root,
    which would silently invalidate the BVH."""
    for pstate in _paint_states:
        if pstate is None:
            continue
        if not pstate.mesh_prim.IsValid():
            return True
        M_mesh_now = UsdGeom.Xformable(pstate.mesh_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        M_rel_now = np.array(M_mesh_now * Winv, dtype=np.float64).reshape(4, 4)
        if not np.allclose(M_rel_now, pstate.M_rel_at_build, atol=DRIFT_TOLERANCE):
            return True
    return False


def paint_faces(global_tri_ids: np.ndarray):
    """Route a batch of global (merged-BVH) triangle ids to their owning
    sub-mesh's local face index, and set those faces to CLEAN_COLOR."""
    if global_tri_ids.size == 0:
        return 0
    global_tri_ids = np.unique(global_tri_ids)
    global_tri_ids = global_tri_ids[global_tri_ids >= 0]
    if global_tri_ids.size == 0:
        return 0
    mesh_idxs = _tri_owner_mesh_np[global_tri_ids]
    face_idxs = _tri_owner_face_np[global_tri_ids]

    n_newly_painted = 0
    for mesh_idx in np.unique(mesh_idxs):
        pstate = _paint_states[mesh_idx]
        if pstate is None:
            continue
        faces_this_mesh = face_idxs[mesh_idxs == mesh_idx]
        clean = np.array(CLEAN_COLOR, dtype=np.float32)
        # Dedup: the mesh's own face identity is a stable natural key, so no
        # spatial hashing or visited-point set is needed.
        already = np.all(pstate.colors[faces_this_mesh] == clean, axis=1)
        newly = faces_this_mesh[~already]
        if newly.size == 0:
            continue
        pstate.colors[newly] = clean
        pstate.dirty = True
        n_newly_painted += int(newly.size)

    for pstate in _paint_states:
        if pstate is not None and pstate.dirty:
            # NOTE: Gf.Vec3f's Boost.Python binding requires native Python
            # float, not numpy.float32 - iterating a numpy array yields numpy
            # scalars, which fail C++ overload resolution with "did not match
            # C++ signature". Cast each component through float().
            pstate.color_attr.Set(Vt.Vec3fArray([
                Gf.Vec3f(float(c[0]), float(c[1]), float(c[2])) for c in pstate.colors
            ]))
            pstate.dirty = False

    return n_newly_painted


def clear_paint():
    for pstate in _paint_states:
        if pstate is None:
            continue
        pstate.colors[:] = np.array(DIRTY_COLOR, dtype=np.float32)
        pstate.dose[:] = 0.0
        pstate.color_attr.Set(Vt.Vec3fArray([Gf.Vec3f(*DIRTY_COLOR)] * pstate.n_faces))
    print("[paint] Cleared all faces back to dirty color (dose reset).")


# -- Dose model ---------------------------------------------------------------
_IRRADIANCE_W_CM2 = LASER_POWER_W / ((FOOTPRINT_SIZE_M * 100.0) ** 2)   # pattern-average
_COS_MAX_INCIDENCE = float(np.cos(np.radians(MAX_INCIDENCE_DEG)))


def _update_visibility(d_local: np.ndarray):
    """Mark the faces that are the first surface along the beam direction,
    using a dense parallel ray grid across the part. Cached per direction;
    the path programs hold the tool orientation, so it is built once a run."""
    global _vis_key
    key = tuple(np.round(d_local, 2))
    if key == _vis_key or _warp_mesh is None:
        return
    pts = _warp_mesh.points.numpy()
    centre = 0.5 * (pts.min(axis=0) + pts.max(axis=0))
    radius = 0.5 * float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))
    d = d_local / np.linalg.norm(d_local)
    a = np.cross(d, [1.0, 0.0, 0.0] if abs(d[0]) < 0.9 else [0.0, 1.0, 0.0])
    a /= np.linalg.norm(a)
    b = np.cross(d, a)
    off = np.arange(-radius, radius + 1e-9, VISIBILITY_STEP_M)
    ga, gb = np.meshgrid(off, off, indexing="ij")
    origins = (centre - d * 2.0 * radius
               + ga.reshape(-1, 1) * a + gb.reshape(-1, 1) * b).astype(np.float32)
    n = len(origins)
    res = wp.zeros(n, dtype=wp.int32, device="cuda:0")
    face = wp.zeros(n, dtype=wp.int32, device="cuda:0")
    t = wp.zeros(n, dtype=float, device="cuda:0")
    wp.launch(kernel_query_ray_grid, dim=n,
              inputs=[_warp_mesh.id, wp.array(origins, dtype=wp.vec3, device="cuda:0"),
                      d_local_gf_to_wp(d), float(4.0 * radius), res, face, t],
              device="cuda:0")
    wp.synchronize()
    tri = face.numpy()[res.numpy() == 1]
    tri = np.unique(tri[tri >= 0])
    owner_mesh, owner_face = _tri_owner_mesh_np[tri], _tri_owner_face_np[tri]
    for mi, pstate in enumerate(_paint_states):
        if pstate is None:
            continue
        pstate.visible[:] = False
        pstate.visible[owner_face[owner_mesh == mi]] = True
    _vis_key = key
    n_vis = sum(int(p.visible.sum()) for p in _paint_states if p is not None)
    print(f"[dose] visibility along beam {key}: {n_vis} faces are first surface ({n} rays)")


def _apply_dose(M_e, W, dt_sim: float) -> int:
    """Add irradiance * dt * efficiency to every visible face whose centroid is
    inside the square scan pattern. Returns the number of faces dosed."""
    W_np = np.array(W, dtype=np.float64).reshape(4, 4)
    rot, trans = W_np[:3, :3], W_np[3, :3]                 # USD row vectors: p' = p @ M
    o = np.array(M_e.ExtractTranslation())
    d = np.array(Gf.Vec3d(M_e.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized())
    ex = np.array(Gf.Vec3d(M_e.TransformDir(Gf.Vec3d(1, 0, 0))).GetNormalized())
    ey = np.array(Gf.Vec3d(M_e.TransformDir(Gf.Vec3d(0, 1, 0))).GetNormalized())
    half = FOOTPRINT_SIZE_M * 0.5
    dosed = 0
    for pstate in _paint_states:
        if pstate is None:
            continue
        rel = pstate.centroid @ rot + trans - o
        along = rel @ d
        inside = (pstate.visible & (along > 0.0)
                  & (np.abs(rel @ ex) <= half) & (np.abs(rel @ ey) <= half))
        if not inside.any():
            continue
        nw = pstate.normal[inside] @ rot
        cos_inc = np.abs(nw @ d) / np.maximum(np.linalg.norm(nw, axis=1), 1e-15)
        dz = along[inside] - FOCAL_DIST_M
        fluence = F0_J_CM2 * cos_inc / (1.0 + (dz / ZR_M) ** 2)
        eta = np.clip((fluence - F_TH_J_CM2) / (F0_J_CM2 - F_TH_J_CM2), 0.0, 1.0)
        eta[cos_inc < _COS_MAX_INCIDENCE] = 0.0
        pstate.dose[inside] += _IRRADIANCE_W_CM2 * dt_sim * eta
        pstate.dirty = True
        dosed += int(inside.sum())
    return dosed


def _write_dose_colors():
    """Rust -> metal by the fraction of the layer energy received; overexposed
    faces in OVEREXPOSED_COLOR."""
    dirty = np.array(DIRTY_COLOR, dtype=np.float32)
    clean = np.array(CLEAN_COLOR, dtype=np.float32)
    for pstate in _paint_states:
        if pstate is None or not pstate.dirty:
            continue
        frac = np.clip(pstate.dose / LAYER_ENERGY_J_CM2, 0.0, 1.0)[:, None].astype(np.float32)
        colors = dirty * (1.0 - frac) + clean * frac
        colors[(pstate.dose - LAYER_ENERGY_J_CM2) > OVEREXPOSE_J_CM2] = OVEREXPOSED_COLOR
        pstate.colors = colors.astype(np.float32)
        pstate.color_attr.Set(Vt.Vec3fArray.FromNumpy(pstate.colors))
        pstate.dirty = False


def _shares(area: np.ndarray, dose: np.ndarray) -> dict:
    total = float(area.sum()) or 1.0
    return {
        "area_cm2": float(area.sum()),
        "clean_pct": 100.0 * float(area[dose >= LAYER_ENERGY_J_CM2].sum()) / total,
        "partial_pct": 100.0 * float(area[(dose > 0) & (dose < LAYER_ENERGY_J_CM2)].sum()) / total,
        "untouched_pct": 100.0 * float(area[dose <= 0].sum()) / total,
        "overexposed_pct": 100.0 * float(area[(dose - LAYER_ENERGY_J_CM2) > OVEREXPOSE_J_CM2].sum()) / total,
        "mean_dose_j_cm2": float((area * dose).sum()) / total,
    }


def dose_summary(centre_xy=None, band_m: float = 0.025, verbose: bool = True) -> dict:
    """
    Area-weighted state of the cleanable faces: first surface along the beam
    (the last direction lased) and no steeper than MAX_INCIDENCE_DEG to it -
    steeper faces (e.g. a side wall seen at a slight tilt) can never be
    cleaned and are reported separately as grazing. With centre_xy (world x, y)
    the result also has radius bands of band_m.
    """
    if _vis_key is None:
        return {}
    W = UsdGeom.Xformable(root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    W_np = np.array(W, dtype=np.float64).reshape(4, 4)
    scale2 = float(np.linalg.norm(W_np[0, :3])) ** 2
    d_local = np.array(_vis_key, dtype=np.float64)
    d_local /= np.linalg.norm(d_local)
    area, dose, radius, grazing = [], [], [], 0.0
    for pstate in _paint_states:
        if pstate is None:
            continue
        steep = np.abs(pstate.normal @ d_local) < _COS_MAX_INCIDENCE
        keep = pstate.visible & ~steep
        grazing += float(pstate.area[pstate.visible & steep].sum()) * scale2 * 1e4
        area.append(pstate.area[keep] * scale2 * 1e4)                     # cm2
        dose.append(pstate.dose[keep])
        if centre_xy is not None:
            c = pstate.centroid[keep] @ W_np[:3, :3] + W_np[3, :3]
            radius.append(np.hypot(c[:, 0] - centre_xy[0], c[:, 1] - centre_xy[1]))
    area, dose = np.concatenate(area), np.concatenate(dose)
    rep = {"beam_on_s": laser_test.beam_on_s, "layer_j_cm2": LAYER_ENERGY_J_CM2,
           "grazing_cm2": grazing, **_shares(area, dose)}
    if centre_xy is not None:
        radius = np.concatenate(radius)
        rep["bands"] = {}
        for lo in np.arange(0.0, radius.max() + 1e-9, band_m):
            sel = (radius >= lo) & (radius < lo + band_m)
            if sel.any():
                rep["bands"][f"{lo * 1000:.0f}-{(lo + band_m) * 1000:.0f}"] = _shares(area[sel], dose[sel])
    if verbose:
        print(f"[dose] beam-on {rep['beam_on_s']:.1f} s sim | cleanable {rep['area_cm2']:.0f} cm2 "
              f"(+{grazing:.0f} cm2 grazing): clean {rep['clean_pct']:.1f}%, partial {rep['partial_pct']:.1f}%, "
              f"untouched {rep['untouched_pct']:.1f}%, overexposed {rep['overexposed_pct']:.1f}% | "
              f"mean dose {rep['mean_dose_j_cm2']:.0f} J/cm2 (layer {LAYER_ENERGY_J_CM2:.0f})")
    return rep


def dose_report(verbose: bool = True) -> dict:
    return dose_summary(None, verbose=verbose)


# -- Controller (state + UI) --------------------------------------------------
class LaserTestController:
    def __init__(self):
        self.running = True
        self.paused  = False
        self._sub    = None
        self.status_label = None
        self.last_reading_label = None
        self.paint_count_label  = None
        self.laser_label        = None
        self.total_painted = 0
        self.beam_on_s     = 0.0         # sim seconds with the laser enabled
        self._last_sim_t   = None
        self._color_elapsed = 0.0

    def _refresh_label(self):
        if self.status_label is not None:
            state = "STOPPED" if not self.running else ("PAUSED" if self.paused else "RUNNING")
            self.status_label.text = f"State: {state}"

    def pause(self):
        self.paused = True
        self._refresh_label()
        print("[laser_test] PAUSED.")

    def resume(self):
        self.paused = False
        self._refresh_label()
        print("[laser_test] RESUMED.")

    def stop(self):
        self.running = False
        if self._sub is not None:
            self._sub.unsubscribe()
            self._sub = None
        builtins._laser_test_sub = None
        if _draw is not None:
            _draw.clear_lines()
            _draw.clear_points()
        self._refresh_label()
        print("[laser_test] STOPPED. Subscription removed, beam cleared. Re-run script to restart.")

    def clear_paint(self):
        clear_paint()
        self.total_painted = 0
        self.beam_on_s = 0.0
        if self.paint_count_label is not None:
            self.paint_count_label.text = "Faces painted: 0"

    def report(self):
        return dose_report()

    def aim(self):
        print_aiming_report()

    def rebuild(self):
        if rebuild_bvh():
            print("[laser_test] BVH rebuilt.")
        else:
            print("[laser_test] BVH rebuild FAILED - no usable mesh found.")

    def status(self):
        state = "STOPPED" if not self.running else ("PAUSED" if self.paused else "RUNNING")
        print(f"[laser_test] state={state}  faces_painted={self.total_painted}")


laser_test = LaserTestController()
builtins.laser_test = laser_test

# --- On-screen control window -------------------------------------------------
_window = ui.Window("Laser Raycast Test Control", width=280, height=280)
with _window.frame:
    with ui.VStack(spacing=6, height=0):
        laser_test.status_label = ui.Label("State: RUNNING", height=24)
        with ui.HStack(spacing=4, height=30):
            ui.Button("Pause",  clicked_fn=lambda: laser_test.pause())
            ui.Button("Resume", clicked_fn=lambda: laser_test.resume())
        ui.Button("STOP", clicked_fn=lambda: laser_test.stop(), height=30)
        ui.Button("Clear Paint", clicked_fn=lambda: laser_test.clear_paint(), height=30)
        with ui.HStack(spacing=4, height=30):
            ui.Button("Aim Report",  clicked_fn=lambda: laser_test.aim())
            ui.Button("Rebuild BVH", clicked_fn=lambda: laser_test.rebuild())
        if DOSE_MODEL:
            ui.Button("Dose Report", clicked_fn=lambda: laser_test.report(), height=30)
            laser_test.laser_label = ui.Label("Laser: OFF", height=24)
        laser_test.last_reading_label = ui.Label("Last: -", height=24)
        laser_test.paint_count_label  = ui.Label("Faces painted: 0", height=24)

builtins._laser_test_window = _window

# --- Laser enable (std_msgs/Bool from the motion server) ---------------------
_timeline = omni.timeline.get_timeline_interface()
_laser_on = [not (DOSE_MODEL and LASER_ENABLE_TOPIC)]   # list: mutated by the callback
_ros_node = None
if DOSE_MODEL and LASER_ENABLE_TOPIC:
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Bool
    if not rclpy.ok():
        rclpy.init()
    _ros_node = rclpy.create_node("laser_cleaning_sim")
    _ros_node.create_subscription(
        Bool, LASER_ENABLE_TOPIC, lambda msg: _laser_on.__setitem__(0, bool(msg.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    builtins._laser_ros_node = _ros_node
    print(f"[dose] Listening to {LASER_ENABLE_TOPIC} for laser enable.")

if DOSE_MODEL:
    # Colours follow the dose from now on; a reused (old) paint state is reset.
    for _ps in _paint_states:
        if _ps is not None:
            _ps.dirty = True
    _write_dose_colors()

# -- Per-tick raycast + paint + debug draw ------------------------------------
_tick_count  = 0
_tick_period = 1.0 / TICK_HZ
_elapsed     = 0.0


def _draw_beam(o_world: Gf.Vec3d, end_world: Gf.Vec3d, color):
    if _draw is None:
        return
    _draw.clear_lines()
    _draw.draw_lines(
        [(o_world[0], o_world[1], o_world[2])],
        [(end_world[0], end_world[1], end_world[2])],
        [color],
        [3.0],
    )


def _build_grid_local_origins(M_e, Winv, half_size):
    """Build GRID_N x GRID_N parallel ray origins across the emitter's local
    XY plane (fixed footprint, no angle correction), transformed directly
    into the target's local space for the batched Warp launch."""
    offsets = np.linspace(-half_size, half_size, GRID_N)
    local_origins = np.empty((_N_GRID_RAYS, 3), dtype=np.float32)
    k = 0
    for gx in offsets:
        for gy in offsets:
            p_emitter_local = Gf.Vec3d(gx, gy, 0.0)
            p_world = Gf.Vec3d(M_e.Transform(p_emitter_local))
            p_target_local = Gf.Vec3d(Winv.Transform(p_world))
            local_origins[k] = [p_target_local[0], p_target_local[1], p_target_local[2]]
            k += 1
    return local_origins


def d_local_gf_to_wp(d_local_gf):
    return wp.vec3(float(d_local_gf[0]), float(d_local_gf[1]), float(d_local_gf[2]))


def _paint_footprint(M_e, Winv, d_local_gf, max_t_local):
    """Fire the GRID_N x GRID_N footprint and paint every face whose OWN ray
    hit lies inside the working-distance gate, so any part of the footprint
    over the part cleans it (the centre ray may be over a hole).
    Returns (rays_in_gate, newly_painted_faces)."""
    local_origins_np = _build_grid_local_origins(M_e, Winv, FOOTPRINT_SIZE_M * 0.5)
    wp_origins = wp.array(local_origins_np, dtype=wp.vec3, device="cuda:0")
    wp.launch(
        kernel_query_ray_grid,
        dim=_N_GRID_RAYS,
        inputs=[_warp_mesh.id, wp_origins, d_local_gf_to_wp(d_local_gf),
                float(max_t_local), _buf_grid_result, _buf_grid_face, _buf_grid_t],
        device="cuda:0",
    )
    wp.synchronize()
    hit = _buf_grid_result.numpy() == 1
    # The grid origins lie in the emitter plane, so t is the distance from the
    # emitter along the beam - in target-local units, where RAY_MAX_T metres
    # corresponds to max_t_local.
    dist_world = _buf_grid_t.numpy() * (RAY_MAX_T / max_t_local)
    in_gate = hit & (dist_world >= WORKING_DIST_MIN) & (dist_world <= WORKING_DIST_MAX)
    return int(in_gate.sum()), paint_faces(_buf_grid_face.numpy()[in_gate])


def _on_update(dt: float):
    global _tick_count, _elapsed

    if not laser_test.running:
        return
    if laser_test.paused:
        return

    _elapsed += dt
    if _elapsed < _tick_period:
        return
    _elapsed = 0.0
    _tick_count += 1

    if _warp_mesh is None:
        if _tick_count % (TICK_HZ * 2) == 0:
            print("[raycast] No mesh - re-attempting BVH build...")
            rebuild_bvh()
        return

    # Sim time since the last tick: the dose integrates on sim time, not on
    # the wall-clock tick (Isaac runs slower than real time).
    t_sim = _timeline.get_current_time()
    dt_sim = 0.0 if laser_test._last_sim_t is None else t_sim - laser_test._last_sim_t
    laser_test._last_sim_t = t_sim
    if not (0.0 < dt_sim < 1.0):            # stopped, paused, rewound or jumped
        dt_sim = 0.0
    if _ros_node is not None:
        rclpy.spin_once(_ros_node, timeout_sec=0.0)
    laser_on = _laser_on[0]

    target_prim  = stage.GetPrimAtPath(TARGET_ROOT_PATH)
    emitter_prim = stage.GetPrimAtPath(EMITTER_PATH)

    M_e = UsdGeom.Xformable(emitter_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    o_world = Gf.Vec3d(M_e.ExtractTranslation())
    d_world = Gf.Vec3d(M_e.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()

    if not target_prim.IsValid():
        _draw_beam(o_world, o_world + d_world * RAY_MAX_T, (1, 0, 0, 1))
        if _tick_count % TICK_HZ == 0:
            print(f"[raycast] tick={_tick_count:05d}  MISS  (target prim missing)")
        return

    img = UsdGeom.Imageable(target_prim)
    if img.ComputeVisibility(Usd.TimeCode.Default()) == UsdGeom.Tokens.invisible:
        _draw_beam(o_world, o_world + d_world * RAY_MAX_T, (1, 0, 0, 1))
        if _tick_count % TICK_HZ == 0:
            print(f"[raycast] tick={_tick_count:05d}  MISS  (target hidden)")
        return

    # Transform the RAY into the target's local space every tick, using its
    # live world transform. The BVH itself never moves. max_t is derived from
    # the transformed endpoint so it scales correctly if local space is not
    # metres (e.g. a mm-authored asset carrying a corrective scale).
    W    = UsdGeom.Xformable(target_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    Winv = W.GetInverse()

    # Guard: a sub-mesh moved relative to the root invalidates the BVH.
    if CHECK_SUBMESH_DRIFT and _tick_count % TICK_HZ == 0:
        if submesh_drift_detected(Winv):
            print("[bvh] Sub-mesh moved relative to the target root - BVH was stale, rebuilding.")
            print("      (Move the ROOT prim, not the mesh prim, to avoid this.)")
            rebuild_bvh()
            return

    o_local_gf   = Gf.Vec3d(Winv.Transform(o_world))
    end_world    = o_world + d_world * RAY_MAX_T
    end_local_gf = Gf.Vec3d(Winv.Transform(end_world))

    diff_local  = end_local_gf - o_local_gf
    max_t_local = diff_local.GetLength()
    d_local_gf  = diff_local.GetNormalized() if max_t_local > 1e-9 else Gf.Vec3d(0, 0, -1)

    o_wp = wp.vec3(float(o_local_gf[0]), float(o_local_gf[1]), float(o_local_gf[2]))
    d_wp = wp.vec3(float(d_local_gf[0]), float(d_local_gf[1]), float(d_local_gf[2]))

    rays_in_gate = 0
    if DOSE_MODEL:
        # --- Dose: visible faces inside the pattern, while the laser is on ---
        # Visibility follows the beam only while lasing, so dose_report() keeps
        # the cleaning direction after the arm has moved away.
        if laser_on and dt_sim > 0.0:
            _update_visibility(np.array(d_local_gf))
            _apply_dose(M_e, W, dt_sim)
            laser_test.beam_on_s += dt_sim
        laser_test._color_elapsed += _tick_period
        if laser_test._color_elapsed >= COLOR_UPDATE_S:
            laser_test._color_elapsed = 0.0
            _write_dose_colors()
            if laser_test.laser_label is not None:
                laser_test.laser_label.text = (
                    f"Laser: {'ON' if laser_on else 'OFF'}  beam-on {laser_test.beam_on_s:.1f} s")
    elif PAINT_ENABLED:
        # --- Footprint paint: every ray gated on its own hit distance ---
        rays_in_gate, n_new = _paint_footprint(M_e, Winv, d_local_gf, max_t_local)
        if n_new > 0:
            laser_test.total_painted += n_new
            if laser_test.paint_count_label is not None:
                laser_test.paint_count_label.text = f"Faces painted: {laser_test.total_painted}"

    # --- Centre ray: drives the beam line and status only ---
    wp.launch(
        kernel_query_ray_single,
        dim=1,
        inputs=[
            _warp_mesh.id, o_wp, d_wp, float(max_t_local),
            _buf_result, _buf_t, _buf_px, _buf_py, _buf_pz,
            _buf_nx, _buf_ny, _buf_nz,
        ],
        device="cuda:0",
    )
    wp.synchronize()

    hit = bool(_buf_result.numpy()[0])
    px, py, pz = float(_buf_px.numpy()[0]), float(_buf_py.numpy()[0]), float(_buf_pz.numpy()[0])
    nx, ny, nz = float(_buf_nx.numpy()[0]), float(_buf_ny.numpy()[0]), float(_buf_nz.numpy()[0])

    if not hit:
        miss_color = (0.5, 0.5, 0.5, 1) if (DOSE_MODEL and not laser_on) else (1, 0, 0, 1)
        _draw_beam(o_world, o_world + d_world * RAY_MAX_T, miss_color)     # RED (GREY: laser off)
        if laser_test.last_reading_label is not None:
            laser_test.last_reading_label.text = (
                f"Last: MISS (centre)  rays in gate {rays_in_gate}/{_N_GRID_RAYS}")
        if _tick_count % TICK_HZ == 0:
            print(f"[raycast] tick={_tick_count:05d}  MISS (centre)  "
                  f"rays_in_gate={rays_in_gate}/{_N_GRID_RAYS}  "
                  f"faces_painted={laser_test.total_painted}")
        return

    hit_local_gf = Gf.Vec3d(px, py, pz)
    hit_world_gf = Gf.Vec3d(W.Transform(hit_local_gf))

    n_local_gf = Gf.Vec3d(nx, ny, nz)
    n_world_gf = Gf.Vec3d(W.TransformDir(n_local_gf)).GetNormalized()

    dist_world = (hit_world_gf - o_world).GetLength()
    if DOSE_MODEL:
        in_gate = abs(dist_world - FOCAL_DIST_M) <= ZR_M     # centre within the focal range
    else:
        in_gate = WORKING_DIST_MIN <= dist_world <= WORKING_DIST_MAX

    color = (0, 1, 0, 1) if in_gate else (1, 1, 0, 1)   # GREEN in-gate, YELLOW out
    if DOSE_MODEL and not laser_on:
        color = (0.5, 0.5, 0.5, 1)                     # GREY: laser off
    _draw_beam(o_world, hit_world_gf, color)

    status = "HIT-IN-GATE" if in_gate else "HIT-OUT-OF-GATE"
    if laser_test.last_reading_label is not None:
        laser_test.last_reading_label.text = (
            f"Last: {status}  centre {(dist_world - FOCAL_DIST_M) * 1000:+.1f} mm from focus"
            if DOSE_MODEL else
            f"Last: {status}  {dist_world:.3f}m  rays in gate {rays_in_gate}/{_N_GRID_RAYS}")

    if _tick_count % TICK_HZ == 0:
        print(
            f"[raycast] tick={_tick_count:05d}  {status}  "
            f"dist={dist_world:.4f} m  rays_in_gate={rays_in_gate}/{_N_GRID_RAYS}  "
            f"hit=({hit_world_gf[0]:.3f},{hit_world_gf[1]:.3f},{hit_world_gf[2]:.3f})  "
            f"n_world=({n_world_gf[0]:.2f},{n_world_gf[1]:.2f},{n_world_gf[2]:.2f})  "
            f"faces_painted={laser_test.total_painted}"
        )


_app = omni.kit.app.get_app()
_sub = _app.get_update_event_stream().create_subscription_to_pop(
    lambda e: _on_update(e.payload["dt"]),
    name="laser_test_raycast",
)
builtins._laser_test_sub = _sub
laser_test._sub = _sub

print("\n[raycast] Loop started at ~{}Hz on {}".format(TICK_HZ, TARGET_ROOT_PATH))
print("          A floating 'Laser Raycast Test Control' window has opened with")
print("          Pause / Resume / STOP / Clear Paint / Aim Report / Rebuild BVH.")
print("          Beam line: RED=MISS, YELLOW=hit-out-of-gate, GREEN=hit-in-gate.")
if DOSE_MODEL:
    print(f"          DOSE MODEL: {LASER_POWER_W:.0f} W over a {FOOTPRINT_SIZE_M * 1000:.0f} mm square "
          f"({_IRRADIANCE_W_CM2:.0f} W/cm2), focus {FOCAL_DIST_M * 1000:.0f} mm from the emitter, "
          f"zR {ZR_M * 1000:.1f} mm,")
    print(f"          layer {LAYER_ENERGY_J_CM2:.0f} J/cm2, overexposure above +{OVEREXPOSE_J_CM2:.0f} J/cm2; "
          f"sim-time integration; GREY beam = laser off. laser_test.report() for totals.")
else:
    print(f"          Every tick fires a {GRID_N}x{GRID_N} ray grid across a "
          f"{FOOTPRINT_SIZE_M * 1000:.0f}mm footprint; each ray paints the face it hits")
    print(f"          when that hit is {WORKING_DIST_MIN:.2f}-{WORKING_DIST_MAX:.2f} m from the emitter.")
print("          Jog the robot or move the TARGET ROOT prim in the viewport - both")
print("          transforms are re-read live every tick, no re-run needed.\n")
