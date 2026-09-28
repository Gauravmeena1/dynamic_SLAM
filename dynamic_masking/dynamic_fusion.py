"""
dynamic_fusion.py
=================
Pixel-level dynamic-object detection = geometric motion residual  OR  semantic class mask.

Geometric half (this file)
--------------------------
For a frame *k* (the one about to be inserted into the map) and a reference
frame *k1* (its immediate predecessor from ``MotionFrameBuffer``):

    measured_flow  = FlowSeek(rgb_k, rgb_k1)                       [H, W, 2]  (px)
    rigid_flow     = project( T_rel * backproject(depth_k, K), K ) - uv         (px)
                     with  T_rel = inv(T_k1_c2w) @ T_k_c2w   (cam_k -> cam_k1)
    residual       = || measured_flow - rigid_flow ||             [H, W]
    threshold      = max( median(r) + k * 1.4826 * MAD(r),  min_threshold_px )
    raw_mask       = residual > threshold   (valid-depth pixels only)
    mask           = open/close + drop small blobs + dilate

Everything that agrees with the *static-scene* prediction is background; only
pixels whose measured motion is not explained by camera ego-motion + depth are
flagged.  No object model, no tracking, no optimisation -- pure feed-forward.

Poses are **camera-to-world** 4x4 (DA3 convention, frame 0 = I).  Depth is in
metres at the RGB resolution (RealSense aligned depth / 1000, or DA3 depth).

Fusion
------
``FusedDynamicDetector`` runs the semantic ``DynamicObjectDetector`` on rgb_k
and OR-s it with the geometric mask (``mode="union"``, the agreed design:
geometry catches unlabeled moving things, semantics catches known movable
classes even while they stand still).
"""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from typing import Callable, Dict, Optional, Tuple

import cv2
import numpy as np
import torch
import torch.nn.functional as F

FlowFn = Callable[[np.ndarray, np.ndarray], np.ndarray]   # (rgb_k, rgb_k1) -> [H, W, 2]


# ─────────────────────────────────────────────────────────────── config
@dataclass
class GeometricMaskConfig:
    adaptive_k: float = 3.0          # threshold = median + k * 1.4826 * MAD
    min_threshold_px: float = 1.0    # floor: below ~1 px the residual is flow noise, not motion
    max_threshold_px: Optional[float] = None
    depth_min_m: float = 0.15        # RealSense D435 min range
    depth_max_m: float = 6.0
    min_mask_area: int = 200         # px, drop smaller final masks
    morph_kernel: int = 7            # opening/closing kernel (px)
    dilate_px: int = 4               # safety margin around detections
    max_global_flow_px: float = 120.0  # skip the pair if the whole image moved this much
    border_px: int = 8               # ignore residual within this many px of the image border
    # depth-discontinuity handling: sensor depth is unreliable at silhouettes (mixed pixels,
    # disocclusion), which is where nearly all static-scene residual concentrates.
    edge_rel_thr: float = 0.05       # |dz|/z to a 4-neighbour above this = depth edge (<=0: off)
    edge_dilate_px: int = 3          # grow the edge band by this many px
    # occlusion test against the reference frame's depth (needs depth_k1): a point that lands
    # *behind* the surface seen in k1 is occluded there -> its measured flow is undefined.
    occlusion_rel_thr: float = 0.05  # (<=0: off)
    # relative residual: flow error grows with displacement, so additionally require
    # residual > rel_residual * |rigid_flow|  (0: off)
    rel_residual: float = 0.0
    # --- Flow4DGS-SLAM style options (Wang et al., CVPR 2026) ---
    # ego_motion: "pose"  = rigid flow from the given camera poses (DA3 / SLAM) + depth
    #             "irls"  = fit a 6-DoF twist to (flow, depth) with Cauchy-IRLS on non-semantic
    #                       pixels (Flow4DGS "camera-induced motion decomposition"); poses unused
    #             "pose_irls" = exact rigid flow from the poses (valid for the 5-20 deg pans of a
    #                       ~1 fps stream, where the linearised "irls" model breaks down), then a
    #                       small IRLS twist correction fitted to the remaining residual on
    #                       non-semantic pixels (absorbs pose / depth-scale error)   [2026-09-03]
    ego_motion: str = "pose"
    irls_iters: int = 10
    irls_cauchy_c: float = 2.0       # px, Cauchy scale
    irls_max_samples: int = 40000
    # exclude the semantic mask from the fit AND from the MAD statistics (Flow4DGS restricts
    # the fit to M_s == 0; excluding them from the statistics as well stops a frame-filling
    # mover from inflating the threshold)
    stats_exclude_semantic: bool = True
    # --- forward-backward flow consistency (2026-09-03) ---
    # a pixel whose flow k->ref and ref->k do not close (|F_fwd(u) + F_bwd(u + F_fwd(u))| >
    # max(fb_max_px, fb_rel * |F_fwd|)) has an unreliable flow estimate (occlusion, blur, texture-
    # less wall, FlowSeek failure on 100-200 px displacements).  It is dropped from `valid`, i.e.
    # it can neither vote "dynamic" nor enter the MAD statistics.  A genuinely moving object keeps
    # a consistent forward/backward flow, so recall is unaffected.  fb_max_px <= 0: off.
    fb_max_px: float = 0.0
    fb_rel: float = 0.05
    # --- static-furniture false-positive gates (2026-09-04, §12) ---
    # residual_mode "full"   = |F - F^|                       (default, and the only mode used in production)
    #               "across" = component of (F - F^) perpendicular to the camera-induced flow direction F^.
    #                          MEASURED AND REJECTED (§12.1): it made person-free false positives *worse* --
    #                          lab 0.65 % -> 0.96 %, TUM walking_xyz 0.41 % -> 0.61 %.  The implementation is
    #                          kept only so tune_fusion_masks.py can reproduce that negative result; it is no
    #                          longer selectable from run_lab_slam_fusion.py.
    residual_mode: str = "full"
    # --- low-frame-rate / blur robustness (2026-09-24) -------------------------------
    # The masker was designed for adjacent video frames (~33 ms).  On this robot the collector
    # emits keyframes (blur-select k=3 over a 15 fps stream, then a 25 px LK disparity gate), so
    # consecutive frames are a median 409 ms and a p90 2.55 s apart.  Across such a gap the flow
    # is unreliable, the residual MAD inflates and tau rises until only extreme motion survives.
    # Rather than emit a noisy mask, detect the condition and hand the frame to the semantic
    # channel alone.  The proper fix is upstream (keep the raw neighbour frame for the flow pair).
    unreliable_threshold_px: float = 0.0   # tau above this => geometry has no discriminative
                                           # power for this pair; skip it.  0 = off.
    min_sharpness: float = 0.0             # Laplacian variance of the frame below this => too
                                           # blurry for flow; skip the pair.  0 = off.
    # --- interior hole filling (2026-09-24) ------------------------------------------
    # A person's interior loses its flow vote wherever a validity gate fires (textureless
    # clothing, depth edges on the silhouette, failed forward-backward check), so the raw
    # geometric mask is a shell with holes.  Filling holes that do NOT touch the image border
    # recovers the interior without growing the mask outwards into the background.
    # --- motion compensation (2026-09-24) -------------------------------------------
    # Warp frame k+1 into frame k's viewpoint with the known camera motion BEFORE running the
    # flow, so the network measures only the independent motion instead of a ~50 px global
    # field it must then have subtracted.  Requires ego_motion="pose".  See motion_compensation.py.
    compensate_first: bool = False
    fill_holes: bool = False
    fill_close_px: int = 0                 # optional extra closing before the fill.  0 = off.
    across_min_rigid_px: float = 2.0
    # NOTE (§12.1): the blob filter (blob_edge_frac_max / blob_min_thickness_px) was REMOVED after measuring
    # that it cannot change the pipeline's output.  On the lab sequence, against no filter:
    #   blob_min_thickness_px=12  ->     0 px changed anywhere (entirely inert)
    #   blob_edge_frac_max=0.5    ->   488 geo-channel px changed out of 19.6 M (0.0025 %), and ZERO px of the
    #                                  output union mask -- everything it dropped was already covered by the
    #                                  semantic mask, so the mask actually used by the solver was unchanged.
    # It is near-unreachable by construction: depth-edge pixels are dropped from `valid` below, so a candidate
    # component contains none of them, and _clean_mask dilates by dilate_px before the test, capping the edge
    # fraction at 7/(7+2*4) = 0.467 for a component lying on the band -- under the 0.5 it shipped with.  Only
    # small components, pulled onto the band by morphological closing, ever exceed it.
    # Depth-edge suppression is done properly and upstream by edge_rel_thr.


from motion_compensation import compensated_flow  # noqa: E402


# ─────────────────────────────────────────────────────────────── geometry
def relative_motion(T_k_c2w: np.ndarray, T_k1_c2w: np.ndarray) -> Tuple[float, float]:
    """(translation [m], rotation [deg]) of cam_k -> cam_k1."""
    T_rel = np.linalg.inv(T_k1_c2w) @ T_k_c2w
    t = float(np.linalg.norm(T_rel[:3, 3]))
    c = np.clip((np.trace(T_rel[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    return t, float(np.degrees(np.arccos(c)))


def rigid_flow_from_depth(depth_m: torch.Tensor, T_k_c2w: torch.Tensor,
                          T_k1_c2w: torch.Tensor, K: torch.Tensor,
                          depth_min: float = 0.0, depth_max: float = np.inf
                          ) -> Tuple[torch.Tensor, torch.Tensor]:
    """Static-scene flow k -> k1 induced purely by camera motion.

    Returns (flow [H, W, 2], valid [H, W] bool).  Invalid = no/out-of-range depth,
    point behind the reference camera, or projected outside the reference image.
    Same math as ``ma_slam_stream.dynamic_mask.DynamicMasker.compute_expected_flow``.
    """
    H, W = depth_m.shape
    dev = depth_m.device
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    v, u = torch.meshgrid(torch.arange(H, device=dev, dtype=torch.float32),
                          torch.arange(W, device=dev, dtype=torch.float32), indexing="ij")
    Z = depth_m
    valid = (Z > depth_min) & (Z < depth_max) & torch.isfinite(Z)
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    P = torch.stack([X, Y, Z], dim=-1).reshape(-1, 3)              # cam_k
    T_rel = torch.linalg.inv(T_k1_c2w) @ T_k_c2w                  # cam_k -> cam_k1
    R, t = T_rel[:3, :3], T_rel[:3, 3]
    P1 = (P @ R.T + t).reshape(H, W, 3)
    z1 = P1[..., 2]
    valid &= z1 > 1e-6
    z1c = z1.clamp(min=1e-6)
    u1 = P1[..., 0] / z1c * fx + cx
    v1 = P1[..., 1] / z1c * fy + cy
    valid &= (u1 >= 0) & (u1 <= W - 1) & (v1 >= 0) & (v1 <= H - 1)
    flow = torch.stack([u1 - u, v1 - v], dim=-1)
    flow[~valid] = 0.0
    rigid_flow_from_depth.last = (P1, u1, v1)     # for the optional occlusion test
    return flow, valid


def depth_edge_mask(depth: torch.Tensor, rel_thr: float, dilate_px: int) -> torch.Tensor:
    """True where depth jumps by > rel_thr (relative) to a 4-neighbour, or borders a hole."""
    H, W = depth.shape
    edge = torch.zeros((H, W), dtype=torch.bool, device=depth.device)
    ok = depth > 0
    for axis in (0, 1):
        a = depth.narrow(axis, 1, depth.shape[axis] - 1)
        b = depth.narrow(axis, 0, depth.shape[axis] - 1)
        oka = ok.narrow(axis, 1, depth.shape[axis] - 1)
        okb = ok.narrow(axis, 0, depth.shape[axis] - 1)
        jump = ((a - b).abs() / torch.maximum(a, b).clamp(min=1e-6) > rel_thr) | (oka ^ okb)
        edge.narrow(axis, 1, depth.shape[axis] - 1).logical_or_(jump)
        edge.narrow(axis, 0, depth.shape[axis] - 1).logical_or_(jump)
    if dilate_px > 0:
        k = 2 * int(dilate_px) + 1
        edge = F.max_pool2d(edge.float()[None, None], k, stride=1, padding=k // 2)[0, 0] > 0.5
    return edge


def occlusion_mask(P1: torch.Tensor, u1: torch.Tensor, v1: torch.Tensor, depth_k1: torch.Tensor,
                   rel_thr: float) -> torch.Tensor:
    """True where the point transformed into cam_k1 lies behind the depth k1 actually observed
    at its projection (=> occluded in k1, measured flow undefined).  Nearest-pixel sampling."""
    H, W = depth_k1.shape
    ui = u1.round().long().clamp(0, W - 1)
    vi = v1.round().long().clamp(0, H - 1)
    z_obs = depth_k1[vi, ui]
    z1 = P1[..., 2]
    return (z_obs > 0) & (z1 > z_obs * (1.0 + rel_thr))


def image_jacobian(u: torch.Tensor, v: torch.Tensor, Z: torch.Tensor, K: torch.Tensor) -> torch.Tensor:
    """2x6 image Jacobian of the perspective flow w.r.t. the twist xi = [tx,ty,tz,wx,wy,wz]
    (small-motion model  F(u,v) = J(x) xi, Flow4DGS-SLAM eq. for the ego-motion fit)."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (u - cx) / fx
    y = (v - cy) / fy
    iz = 1.0 / Z.clamp(min=1e-6)
    z = torch.zeros_like(x)
    Ju = torch.stack([fx * iz, z, -fx * x * iz, -fx * x * y, fx * (1 + x * x), -fx * y], dim=-1)
    Jv = torch.stack([z, fy * iz, -fy * y * iz, -fy * (1 + y * y), fy * x * y, fy * x], dim=-1)
    return torch.stack([Ju, Jv], dim=-2)                       # [..., 2, 6]


def fit_ego_motion_irls(flow: torch.Tensor, depth: torch.Tensor, K: torch.Tensor, fit_mask: torch.Tensor,
                        iters: int = 10, cauchy_c: float = 2.0, max_samples: int = 40000,
                        seed: int = 0) -> Tuple[torch.Tensor, torch.Tensor, Dict]:
    """Fit the 6-DoF twist that best explains the measured flow on ``fit_mask`` pixels with
    iteratively re-weighted least squares (Cauchy weights).  Returns (xi [6], induced flow [H,W,2], info)."""
    H, W = depth.shape
    dev = depth.device
    v, u = torch.meshgrid(torch.arange(H, device=dev, dtype=torch.float32),
                          torch.arange(W, device=dev, dtype=torch.float32), indexing="ij")
    idx = torch.nonzero(fit_mask.reshape(-1), as_tuple=False).squeeze(1)
    if idx.numel() < 50:
        return torch.zeros(6, device=dev), torch.zeros(H, W, 2, device=dev), dict(n_fit=int(idx.numel()), converged=False)
    if idx.numel() > max_samples:
        g = torch.Generator(device="cpu").manual_seed(seed)
        idx = idx[torch.randperm(idx.numel(), generator=g)[:max_samples].to(dev)]
    us, vs, Zs = u.reshape(-1)[idx], v.reshape(-1)[idx], depth.reshape(-1)[idx]
    F = flow.reshape(-1, 2)[idx]                                # [n, 2]
    J = image_jacobian(us, vs, Zs, K)                           # [n, 2, 6]
    A = J.reshape(-1, 6).double()                               # [2n, 6]
    b = F.reshape(-1).double()                                  # [2n]
    w = torch.ones(idx.numel(), device=dev, dtype=torch.float64)
    xi = torch.zeros(6, device=dev, dtype=torch.float64)
    for _ in range(max(1, iters)):
        ww = w.repeat_interleave(2)
        Aw = A * ww[:, None]
        xi = torch.linalg.lstsq(Aw, (b * ww)[:, None]).solution.squeeze(1)
        r = (A @ xi - b).reshape(-1, 2).norm(dim=-1)            # per-pixel residual [n]
        w = 1.0 / (1.0 + (r / cauchy_c) ** 2)                   # Cauchy weights
    Jall = image_jacobian(u, v, depth, K)                       # [H, W, 2, 6]
    induced = (Jall.double() @ xi).float()                      # [H, W, 2]
    info = dict(n_fit=int(idx.numel()), converged=True, xi=xi.float().cpu().numpy().tolist(),
                fit_residual_median_px=float(r.median()), inlier_frac=float((r < 3 * cauchy_c).float().mean()))
    return xi.float(), induced, info


def forward_backward_error(flow_fwd: torch.Tensor, flow_bwd: torch.Tensor) -> torch.Tensor:
    """|F_fwd(u) + F_bwd(u + F_fwd(u))| per pixel [H, W]; F_fwd: k->ref, F_bwd: ref->k, both [H,W,2] px.
    Pixels whose forward target falls outside the reference image get +inf (undefined)."""
    H, W = flow_fwd.shape[:2]
    dev = flow_fwd.device
    v, u = torch.meshgrid(torch.arange(H, device=dev, dtype=torch.float32),
                          torch.arange(W, device=dev, dtype=torch.float32), indexing="ij")
    tu = u + flow_fwd[..., 0]; tv = v + flow_fwd[..., 1]
    gx = 2.0 * tu / max(W - 1, 1) - 1.0; gy = 2.0 * tv / max(H - 1, 1) - 1.0
    grid = torch.stack([gx, gy], dim=-1)[None]                                  # [1,H,W,2]
    bwd = F.grid_sample(flow_bwd.permute(2, 0, 1)[None], grid, mode="bilinear",
                        padding_mode="border", align_corners=True)[0].permute(1, 2, 0)
    err = (flow_fwd + bwd).norm(dim=-1)
    inside = (tu >= 0) & (tu <= W - 1) & (tv >= 0) & (tv <= H - 1)
    return torch.where(inside, err, torch.full_like(err, float("inf")))


def warp_mask(mask_ref: np.ndarray, flow_k_to_ref: np.ndarray) -> np.ndarray:
    """Bring a mask from the reference frame into frame k by sampling it at (u + flow)."""
    H, W = mask_ref.shape
    v, u = np.mgrid[0:H, 0:W].astype(np.float32)
    mu = u + flow_k_to_ref[..., 0]; mv = v + flow_k_to_ref[..., 1]
    return cv2.remap(mask_ref.astype(np.uint8), mu, mv, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0) > 0


def _clean_mask(mask: np.ndarray, kernel: int, dilate_px: int) -> np.ndarray:
    k = max(int(kernel), 1)
    m = mask.astype(np.uint8) * 255
    if k > 1:
        ke = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, ke)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ke)
    if dilate_px > 0:
        dk = int(dilate_px) * 2 + 1
        m = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dk, dk)))
    return m > 127


def _fill_interior_holes(mask: np.ndarray, close_px: int = 0) -> np.ndarray:
    """Fill holes that are fully enclosed by the mask, leaving its outer shape unchanged.

    Flood-fills the background inward from the image border on a 1 px padded copy; whatever
    background is left unreached was enclosed by the mask, i.e. an interior hole.  Concavities
    that open onto the border are NOT filled, so the mask cannot grow into the scene.
    """
    if not mask.any():
        return mask
    m = (mask.astype(np.uint8)) * 255
    if close_px > 1:
        ke = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(close_px), int(close_px)))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ke)
    H, W = m.shape
    # 1 = background, 0 = mask.  The padding ring is background, so a flood from (0, 0) reaches
    # every background pixel that touches the image border; whatever stays 1 is enclosed.
    bg = np.ones((H + 2, W + 2), np.uint8)
    bg[1:-1, 1:-1] = (m == 0).astype(np.uint8)
    ff = bg.copy()
    cv2.floodFill(ff, np.zeros((H + 4, W + 4), np.uint8), (0, 0), 2)
    holes = ((bg == 1) & (ff != 2))[1:-1, 1:-1]
    return (m > 127) | holes


# ─────────────────────────────────────────────────────────────── geometric detector
class GeometricMotionDetector:
    """Flow-residual motion detector.  ``flow_fn`` is any (rgb_k, rgb_k1) -> [H, W, 2] callable
    (``flowseek_flow.FlowSeekFlow`` here; RAFT from ``dynamic_mask.py`` would also fit)."""

    def __init__(self, flow_fn: FlowFn, cfg: Optional[GeometricMaskConfig] = None,
                 device: str = "cuda"):
        self.flow_fn = flow_fn
        self.cfg = cfg or GeometricMaskConfig()
        self.device = device

    @torch.no_grad()
    def compute(self, rgb_k: np.ndarray, rgb_k1: np.ndarray, depth_k_m: np.ndarray,
                T_k_c2w: Optional[np.ndarray], T_k1_c2w: Optional[np.ndarray], K: np.ndarray,
                depth_k1_m: Optional[np.ndarray] = None, sem_mask_k: Optional[np.ndarray] = None,
                measured: Optional[np.ndarray] = None, measured_bwd: Optional[np.ndarray] = None,
                extra_valid: Optional[np.ndarray] = None) -> Dict:
        """measured: flow k->k1 (computed here if None); measured_bwd: flow k1->k, only needed for
        the forward-backward check (computed here if None and cfg.fb_max_px > 0).
        extra_valid: optional bool [H, W]; False pixels cannot vote (e.g. low DA3 depth confidence)."""
        cfg = self.cfg
        H, W = depth_k_m.shape
        t0 = time.perf_counter()
        comp_info = None
        if cfg.compensate_first and measured is None:
            # Cancel the camera motion first, then measure.  `compensated_flow` returns the
            # TOTAL flow (residual + rigid), so every formula below is unchanged: the residual
            # the detector forms, |measured - rigid|, is exactly the compensated measurement.
            if cfg.ego_motion != "pose":
                raise ValueError("compensate_first needs ego_motion='pose' (the warp uses the given poses)")
            if T_k_c2w is None or T_k1_c2w is None:
                raise ValueError("compensate_first needs both camera poses")
            _dev = self.device
            _rigid_pre, _rvalid_pre = rigid_flow_from_depth(
                torch.from_numpy(np.asarray(depth_k_m, dtype=np.float32)).to(_dev),
                torch.from_numpy(np.asarray(T_k_c2w, dtype=np.float32)).to(_dev),
                torch.from_numpy(np.asarray(T_k1_c2w, dtype=np.float32)).to(_dev),
                torch.from_numpy(np.asarray(K, dtype=np.float32)).to(_dev),
                cfg.depth_min_m, cfg.depth_max_m)
            measured, _fb_ok, _wvalid, comp_info = compensated_flow(
                self.flow_fn, rgb_k, rgb_k1,
                _rigid_pre.cpu().numpy(), _rvalid_pre.cpu().numpy(),
                fb_max_px=cfg.fb_max_px, fb_rel=cfg.fb_rel)
            # the forward-backward test was already done in the compensated domain, which is
            # where the decision is actually made; skip the raw-domain one.
            measured_bwd = None
            _ev = _wvalid if _fb_ok is None else (_wvalid & _fb_ok)
            extra_valid = _ev if extra_valid is None else (np.asarray(extra_valid, bool) & _ev)
        if measured is None:
            measured = self.flow_fn(rgb_k, rgb_k1)                    # [H, W, 2] np
        if cfg.fb_max_px > 0 and measured_bwd is None:
            measured_bwd = self.flow_fn(rgb_k1, rgb_k)                # [H, W, 2] np, k1 -> k
        t_flow = time.perf_counter() - t0
        if cfg.ego_motion in ("pose", "pose_irls") and (T_k_c2w is None or T_k1_c2w is None):
            raise ValueError(f"ego_motion={cfg.ego_motion!r} needs both camera poses")
        if cfg.ego_motion != "pose":
            T_k_c2w = np.eye(4) if T_k_c2w is None else T_k_c2w
            T_k1_c2w = np.eye(4) if T_k1_c2w is None else T_k1_c2w

        dev = self.device
        depth_t = torch.from_numpy(np.asarray(depth_k_m, dtype=np.float32)).to(dev)
        rigid_t, valid_t = rigid_flow_from_depth(
            depth_t, torch.from_numpy(np.asarray(T_k_c2w, dtype=np.float32)).to(dev),
            torch.from_numpy(np.asarray(T_k1_c2w, dtype=np.float32)).to(dev),
            torch.from_numpy(np.asarray(K, dtype=np.float32)).to(dev),
            cfg.depth_min_m, cfg.depth_max_m)
        n_valid_depth = int(valid_t.sum())
        edge_t = None
        if cfg.edge_rel_thr > 0:
            edge_t = depth_edge_mask(depth_t, cfg.edge_rel_thr, cfg.edge_dilate_px)
            valid_t &= ~edge_t
        if extra_valid is not None:
            valid_t &= torch.from_numpy(np.asarray(extra_valid, bool)).to(dev)
        n_after_edge = int(valid_t.sum())
        if cfg.occlusion_rel_thr > 0 and depth_k1_m is not None:
            P1, u1, v1 = rigid_flow_from_depth.last
            d1 = torch.from_numpy(np.asarray(depth_k1_m, dtype=np.float32)).to(dev)
            valid_t &= ~occlusion_mask(P1, u1, v1, d1, cfg.occlusion_rel_thr)
        n_after_occ = int(valid_t.sum())
        measured_t = torch.from_numpy(measured).to(dev)
        # forward-backward consistency: unreliable flow can neither vote nor enter the statistics
        fb_dropped = 0
        if cfg.fb_max_px > 0 and measured_bwd is not None:
            fb_err = forward_backward_error(measured_t, torch.from_numpy(measured_bwd).to(dev))
            fb_ok = fb_err < torch.clamp(cfg.fb_rel * measured_t.norm(dim=-1), min=cfg.fb_max_px)
            fb_dropped = int((valid_t & ~fb_ok).sum())
            valid_t &= fb_ok
        sem_t = None if sem_mask_k is None else torch.from_numpy(np.asarray(sem_mask_k, bool)).to(dev)
        ego_info = {}
        if cfg.ego_motion == "irls":
            fit = valid_t.clone()
            if sem_t is not None:
                fit &= ~sem_t
            _, rigid_t, ego_info = fit_ego_motion_irls(measured_t, depth_t, torch.from_numpy(np.asarray(K, np.float32)).to(dev),
                                                       fit, cfg.irls_iters, cfg.irls_cauchy_c, cfg.irls_max_samples)
        elif cfg.ego_motion == "pose_irls":
            # exact rigid flow from the poses, plus a small linearised twist fitted to what is left
            fit = valid_t.clone()
            if sem_t is not None:
                fit &= ~sem_t
            _, corr_t, ego_info = fit_ego_motion_irls(measured_t - rigid_t, depth_t,
                                                      torch.from_numpy(np.asarray(K, np.float32)).to(dev),
                                                      fit, cfg.irls_iters, cfg.irls_cauchy_c, cfg.irls_max_samples)
            ego_info["correction_median_px"] = float(corr_t[valid_t].norm(dim=-1).median()) if valid_t.any() else 0.0
            rigid_t = rigid_t + corr_t
        diff_t = measured_t - rigid_t
        residual_full_t = diff_t.norm(dim=-1)                          # [H, W]
        if cfg.residual_mode == "across":
            rmag = rigid_t.norm(dim=-1)
            u_hat = rigid_t / rmag.clamp(min=1e-6)[..., None]
            along = (diff_t * u_hat).sum(dim=-1)
            across_t = (diff_t - along[..., None] * u_hat).norm(dim=-1)
            residual_t = torch.where(rmag >= cfg.across_min_rigid_px, across_t, residual_full_t)
        else:
            residual_t = residual_full_t
        if cfg.border_px > 0:
            b = cfg.border_px
            border = torch.zeros_like(valid_t)
            border[b:H - b, b:W - b] = True
            valid_t &= border

        stats_t = valid_t & ~sem_t if (cfg.stats_exclude_semantic and sem_t is not None) else valid_t
        r_valid = residual_t[stats_t]
        if r_valid.numel() < 100:
            median = mad = float("nan"); thr = float("inf")
        else:
            median = r_valid.median()
            mad = (r_valid - median).abs().median()
            thr = float(median + cfg.adaptive_k * 1.4826 * mad)
            median, mad = float(median), float(mad)
            thr = max(thr, cfg.min_threshold_px)
            if cfg.max_threshold_px is not None:
                thr = min(thr, cfg.max_threshold_px)

        sharpness = None
        if cfg.min_sharpness > 0:
            _g = rgb_k if rgb_k.ndim == 2 else cv2.cvtColor(np.ascontiguousarray(rgb_k), cv2.COLOR_RGB2GRAY)
            sharpness = float(cv2.Laplacian(_g, cv2.CV_64F).var())

        raw_t = (residual_t > thr) & valid_t
        if cfg.rel_residual > 0:
            raw_t &= residual_t > cfg.rel_residual * rigid_t.norm(dim=-1)
        raw = raw_t.cpu().numpy()
        global_flow = float(measured_t.norm(dim=-1).median())
        skipped = None
        if global_flow > cfg.max_global_flow_px:
            skipped = f"global flow {global_flow:.1f}px > {cfg.max_global_flow_px}px"
        elif cfg.unreliable_threshold_px > 0 and thr > cfg.unreliable_threshold_px:
            # tau this high means the residual distribution is dominated by flow error, not by
            # motion: a real walker would have to show more displacement than the method can
            # trust.  Declining is honest; emitting the mask anyway is noise.
            skipped = f"tau {thr:.1f}px > {cfg.unreliable_threshold_px}px (flow unreliable)"
        elif cfg.min_sharpness > 0 and sharpness is not None and sharpness < cfg.min_sharpness:
            skipped = f"sharpness {sharpness:.0f} < {cfg.min_sharpness} (motion blur)"
        if skipped is not None:
            mask = np.zeros((H, W), dtype=bool)
        else:
            mask = self.postprocess(raw, None if edge_t is None else edge_t.cpu().numpy())

        trans_m, rot_deg = relative_motion(np.asarray(T_k_c2w, float), np.asarray(T_k1_c2w, float))
        info = dict(
            threshold_px=thr, residual_median_px=median, residual_mad_px=mad,
            residual_p95_px=float(torch.quantile(r_valid, 0.95)) if r_valid.numel() >= 100 else float("nan"),
            global_flow_median_px=global_flow,
            rigid_flow_median_px=float(rigid_t[valid_t].norm(dim=-1).median()) if valid_t.any() else 0.0,
            valid_frac=float(valid_t.float().mean()),
            edge_dropped_frac=float((n_valid_depth - n_after_edge) / (H * W)),
            occluded_dropped_frac=float((n_after_edge - n_after_occ) / (H * W)),
            fb_dropped_frac=float(fb_dropped / (H * W)),
            raw_area_frac=float(raw.mean()), mask_area_frac=float(mask.mean()),
            translation_m=trans_m, rotation_deg=rot_deg,
            skipped=skipped, sharpness=sharpness, t_flow_s=t_flow,
            **({} if comp_info is None else comp_info), t_total_s=time.perf_counter() - t0,
            ego_motion=cfg.ego_motion, **{f"irls_{k}": v for k, v in ego_info.items() if k != "xi"},
        )
        return dict(mask=mask, raw_mask=raw, residual=residual_t.cpu().numpy(), residual_full=residual_full_t.cpu().numpy(),
                    measured_flow=measured, rigid_flow=rigid_t.cpu().numpy(), edge=None if edge_t is None else edge_t.cpu().numpy(),
                    valid=valid_t.cpu().numpy(), info=info)

    def postprocess(self, raw: np.ndarray, edge: Optional[np.ndarray]) -> np.ndarray:
        """Morphology + min area.  Shared by compute() and by the two-sided combination in
        ChunkFusionMasker.  `edge` is accepted for call-site compatibility and is unused since §12.1."""
        cfg = self.cfg
        H, W = raw.shape
        mask = _clean_mask(raw, cfg.morph_kernel, cfg.dilate_px)
        if cfg.fill_holes:
            mask = _fill_interior_holes(mask, cfg.fill_close_px)
        if mask.sum() < cfg.min_mask_area:
            return np.zeros((H, W), dtype=bool)
        return mask


# ─────────────────────────────────────────────────────────────── fusion
class FusedDynamicDetector:
    """geometric OR semantic (default).  Either half may be None."""

    MODES = ("union", "geometric", "semantic", "intersection")

    def __init__(self, geometric: Optional[GeometricMotionDetector] = None,
                 semantic=None, mode: str = "union", semantic_clean: bool = True):
        if mode not in self.MODES:
            raise ValueError(f"mode must be one of {self.MODES}")
        self.geometric = geometric
        self.semantic = semantic
        self.mode = mode
        self.semantic_clean = semantic_clean

    def detect(self, rgb_k: np.ndarray, rgb_k1: Optional[np.ndarray] = None,
               depth_k_m: Optional[np.ndarray] = None, T_k_c2w: Optional[np.ndarray] = None,
               T_k1_c2w: Optional[np.ndarray] = None, K: Optional[np.ndarray] = None,
               depth_k1_m: Optional[np.ndarray] = None) -> Dict:
        H, W = rgb_k.shape[:2]
        zeros = np.zeros((H, W), dtype=bool)
        info: Dict = {"mode": self.mode}

        sem_mask = zeros
        sem_dets = []
        if self.semantic is not None and self.mode != "geometric":
            t0 = time.perf_counter()
            sem_mask = self.semantic.get_mask(rgb_k, clean=self.semantic_clean)
            sem_dets = list(getattr(self.semantic, "last_detections", []))
            info["semantic"] = dict(t_s=time.perf_counter() - t0, area_frac=float(sem_mask.mean()),
                                    n_det=len(sem_dets), classes=sorted({d["name"] for d in sem_dets}))

        geo = None
        geo_mask = zeros
        need_pose = self.geometric is not None and self.geometric.cfg.ego_motion == "pose"
        if self.geometric is not None and self.mode != "semantic" and rgb_k1 is not None \
                and depth_k_m is not None and K is not None \
                and (not need_pose or (T_k_c2w is not None and T_k1_c2w is not None)):
            geo = self.geometric.compute(rgb_k, rgb_k1, depth_k_m, T_k_c2w, T_k1_c2w, K, depth_k1_m,
                                         sem_mask_k=sem_mask if self.semantic is not None else None)
            geo_mask = geo["mask"]
            info["geometric"] = geo["info"]

        if self.mode == "union":
            mask = geo_mask | sem_mask
        elif self.mode == "intersection":
            mask = geo_mask & sem_mask
        elif self.mode == "geometric":
            mask = geo_mask
        else:
            mask = sem_mask
        info["mask_area_frac"] = float(mask.mean())
        return dict(mask=mask, geo_mask=geo_mask, sem_mask=sem_mask, geo=geo,
                    sem_detections=sem_dets, info=info)


def config_dict(cfg: GeometricMaskConfig) -> Dict:
    return asdict(cfg)
