"""
chunk_fusion_masker.py
======================
Flow + semantic dynamic masks for a whole SLAM chunk, with the exact interface the ma_slam
solver expects from its `_dynamic_masker`:

    mask_chunk(images [T,H,W,3], depth [T,H,W], poses [T,4,4] c2w, intrinsics [3,3] or [T,3,3]) -> bool [T,H,W]

Per frame k (reference = k-1, or k+1 for the first frame):
    M_s   : YOLOv9-seg movable classes                                     (semantic)
    F     : FlowSeek flow k -> ref
    F_hat : camera-induced flow, either from the chunk poses + depth ("pose") or from a
            6-DoF twist fitted to (F, depth) by Cauchy-IRLS on M_s == 0 pixels ("irls",
            Flow4DGS-SLAM "camera-induced motion decomposition")
    M_ca  : ||F - F_hat|| > median + k * MAD   (statistics on M_s == 0 pixels)
    M_dy  = M_s  U  M_ca                                                    (union)
Optional (not in Flow4DGS): temporal propagation  M_dy[k] |= warp(M_s[k-1]) | warp(M_s[k+1])
(propagate="sem", default since 2026-09-03; "union" warps the whole v1 mask and was found to
triple the geometric false-positive area) so one missed detection does not leave a permanent
ghost in a skip-insertion map.

Images may arrive at a different resolution than depth (the solver re-reads full-res RGB
when the backend returns numpy); everything is computed at the DEPTH resolution.
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
from dynamic_fusion import GeometricMaskConfig, GeometricMotionDetector, warp_mask   # noqa: E402


def _to_np(x):
    return x.detach().cpu().numpy() if torch.is_tensor(x) else np.asarray(x)


def _clean(mask: np.ndarray, open_k: int = 3, close_k: int = 7, min_area: int = 150, dilate_k: int = 5) -> np.ndarray:
    """numpy twin of dynamic_object_mask.clean_mask: open -> close -> drop small blobs -> dilate."""
    m = mask.astype(np.uint8)
    if open_k > 1:
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((open_k, open_k), np.uint8))
    if close_k > 1:
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((close_k, close_k), np.uint8))
    if min_area > 0 and m.any():
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        keep = np.zeros(n, bool); keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_area
        m = keep[lab].astype(np.uint8)
    if dilate_k > 1:
        m = cv2.dilate(m, np.ones((dilate_k, dilate_k), np.uint8))
    return m > 0


def _dilate(mask: np.ndarray, px: int) -> np.ndarray:
    if px <= 0: return mask
    return cv2.dilate(mask.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * px + 1, 2 * px + 1))) > 0


class ChunkFusionMasker:
    def __init__(self, flow_fn=None, semantic=None, geo_cfg: Optional[GeometricMaskConfig] = None,
                 propagate="sem", device: str = "cuda", yolo_ckpt: str = os.path.join(HERE, "yolov9e-seg.pt"),
                 flowseek_size: str = "T", record: bool = True,
                 object_policy: str = "all", attach_px: int = 10, motion_thr: float = 0.3, min_valid_px: int = 50,
                 vote_iou: float = 0.3, geo_two_sided: bool = True, conf_gate_pct: float = 0.0, grow_carried: bool = True,
                 grow_depth_margin_m: float = 0.4, two_sided_strong_factor: float = 5.0, grow_max_area_ratio: float = 1.0,
                 geo_gate: str = "none", geo_anchor_px: int = 30):
        """Static-furniture gates (2026-09-04, §12).  DEFAULTS SINCE §12.1 (2026-09-08) are the
        "g_two_s5_grow" combination: geo_two_sided=True + two_sided_strong_factor=5.0 + grow_carried=True,
        with conf_gate_pct left OFF.  Chosen over plain two-sided/conf gating because it is the only
        combination that suppresses static-furniture false positives (3x-41x fewer, across seven sequences)
        *without* losing geometry-only objects YOLO cannot label: on Bonn placing_box it beats the ungated
        baseline on both axes at once (FP 0.75 % -> 0.14 %, coverage 21.73 % -> 22.28 %), and on moving_box it
        matches conf-gated suppression at one seventh of the coverage cost.
             geo_two_sided   = a pixel votes only if its residual is above threshold against BOTH k-1 and k+1
                               (raw masks intersected before morphology; chunk edges fall back to one side)
             conf_gate_pct   = pixels whose DA3 depth confidence is below this percentile of the frame cannot vote
                               (needs conf= passed to mask_chunk; 0 = off).  Left at 0: it rejects the
                               low-confidence surface of a carried box along with the furniture.
             grow_carried    = extend each person mask into connected raw-residual pixels at the person's depth
                               (carried objects without a semantic label)"""
        # geo_gate (2026-09-28): "none" = every geometric blob is kept (behaviour up to now);
        # "anchor" = a connected geometric blob is kept only if it touches a movable-class detection
        # (sem_all, dilated by geo_anchor_px).  On the 0922 lab capture the unanchored blobs were
        # static depth edges (door frames, shelf tops, box edges) flagged while the camera turned.
        # Cost: a moving object YOLO cannot label is no longer removed by motion alone.
        if geo_gate not in ("none", "anchor"):
            raise ValueError(f"geo_gate must be 'none' or 'anchor', got {geo_gate!r}")
        self.geo_gate = geo_gate; self.geo_anchor_px = int(geo_anchor_px)
        self.geo_two_sided = bool(geo_two_sided); self.conf_gate_pct = float(conf_gate_pct)
        self.grow_carried = bool(grow_carried); self.grow_depth_margin_m = float(grow_depth_margin_m)
        # two_sided_strong_factor > 0: a pixel also votes if ONE side alone has residual > factor x that side's
        # threshold (a small fast object is often matched by the flow in one direction only; depth-edge errors
        # sit just above the threshold and rarely reach 3x)
        self.two_sided_strong_factor = float(two_sided_strong_factor); self.grow_max_area_ratio = float(grow_max_area_ratio)
        """object_policy (2026-09-04):
             "all"                = every pixel of a movable class is removed (behaviour up to §10)
             "moving_or_carried"  = a *person* is always removed; any other movable-class instance is removed
                                    only if it is moving (fraction of its valid pixels with flow residual above
                                    the frame threshold >= motion_thr) or carried (its mask dilated by attach_px
                                    touches a person mask).  Decisions are smoothed by a majority vote with the
                                    matching instance (same class, IoU >= vote_iou after warping with the
                                    already-computed flows) in frames k-1 and k+1.  Objects judged static are
                                    left out of the mask and therefore mapped."""
        self.object_policy = object_policy; self.attach_px = int(attach_px); self.motion_thr = float(motion_thr)
        self.min_valid_px = int(min_valid_px); self.vote_iou = float(vote_iou)
        if flow_fn is None:
            from flowseek_flow import FlowSeekFlow
            flow_fn = FlowSeekFlow(size=flowseek_size, device=device)
        if semantic is None and yolo_ckpt:
            from dynamic_object_mask import DynamicObjectDetector
            semantic = DynamicObjectDetector(yolo_ckpt, device=device)
        self.flow_fn = flow_fn
        self.semantic = semantic
        self.geo = GeometricMotionDetector(flow_fn, geo_cfg or GeometricMaskConfig(ego_motion="irls"), device=device)
        self.propagate = propagate
        self.record = record
        self.records: List[Dict] = []          # per-frame diagnostics + masks (sem / geo / prop / union)
        self._frame_counter = 0
        # tail of the previous chunk for bridging across the 1-frame submap overlap:
        # (sem mask of its frame T-2, flow T-1 -> T-2, image of frame T-1) -- see mask_chunk
        self._prev_tail = None

    # ------------------------------------------------------------------ v2 (2026-09-03)
    # Changes vs. mask_chunk_v1 (kept below for reference):
    #   * every flow is computed once up front: fwd[k] = F(k -> k-1), nxt[k] = F(k -> k+1).  The
    #     backward flow of pair (k, k-1) is nxt[k-1], so the forward-backward reliability check in
    #     GeometricMotionDetector costs no extra FlowSeek calls.
    #   * propagate = "sem" (default): only the *semantic* masks of k-1 / k+1 are warped into k
    #     (covers a single missed YOLO detection without spreading geometric false positives);
    #     "union" = v1 behaviour; "off" / False = none; "sem_bridge" = only fill a single-frame
    #     semantic gap (k-1 and k+1 both detect, k does not) -- the mode used for the final run.
    #   * records carry residual p95 (the detector never produced a p99) and the fb-dropped fraction.
    @torch.no_grad()
    def mask_chunk(self, images, depth, poses, intrinsics, conf=None) -> torch.Tensor:
        imgs = _to_np(images); dep = _to_np(depth).astype(np.float32); pos = _to_np(poses).astype(np.float64)
        confn = None if conf is None else _to_np(conf).astype(np.float32)
        K_all = _to_np(intrinsics).astype(np.float64)
        T, H, W = dep.shape
        if imgs.dtype != np.uint8:
            imgs = (np.clip(imgs, 0, 1) * 255).round().astype(np.uint8) if imgs.max() <= 1.0 else np.clip(imgs, 0, 255).astype(np.uint8)
        if imgs.shape[1] != H or imgs.shape[2] != W:
            imgs = np.stack([cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA) for im in imgs])
        Ks = K_all if K_all.ndim == 3 else np.repeat(K_all[None], T, 0)
        prop_mode = self.propagate if isinstance(self.propagate, str) else ("union" if self.propagate else "off")

        # semantic masks.  sem_all = every movable-class pixel (used for the geometry statistics / fit
        # exclusion in every policy); sem_person = people only; inst = per-instance raw masks
        sem_all = np.zeros((T, H, W), bool); sem_person = np.zeros((T, H, W), bool); dets = [None] * T
        inst: List[List[Dict]] = [[] for _ in range(T)]
        use_instances = self.object_policy != "all" and self.semantic is not None and hasattr(self.semantic, "get_instances")
        for k in range(T):
            if self.semantic is None:
                continue
            if use_instances:
                inst[k] = self.semantic.get_instances(imgs[k])
                if inst[k]:
                    sem_all[k] = _clean(np.any([d["mask"] for d in inst[k]], axis=0))
                    pm = [d["mask"] for d in inst[k] if d["cls"] == 0]
                    if pm: sem_person[k] = _clean(np.any(pm, axis=0))
            else:
                sem_all[k] = self.semantic.get_mask(imgs[k], clean=True)
            dets[k] = [(d["name"], round(d["conf"], 2)) for d in self.semantic.last_detections]
        sem = sem_all.copy()                       # policy "all": remove everything detected

        # When the geometry runs motion-compensated, the detector must compute its own flow on
        # the WARPED pair -- handing it a pre-computed raw flow silently bypasses compensation
        # entirely (the `measured is None` guard in compute()).  The raw flows below are still
        # needed for mask warping and cross-frame object matching, so they are kept either way.
        _comp = bool(getattr(self.geo.cfg, "compensate_first", False))

        # all pairwise flows between neighbours (both directions)
        fwd = [None] * T; nxt = [None] * T
        for k in range(1, T):
            fwd[k] = self.flow_fn(imgs[k], imgs[k - 1])
        for k in range(T - 1):
            nxt[k] = self.flow_fn(imgs[k], imgs[k + 1])

        # geometric masks
        geo = np.zeros((T, H, W), bool); ginfo = [None] * T; flow_to_ref = [None] * T; gres = [None] * T
        raw_geo = np.zeros((T, H, W), bool); raw_any = np.zeros((T, H, W), bool)
        for k in range(T):
            if T == 1:
                break
            ev = None
            if confn is not None and self.conf_gate_pct > 0:
                ev = confn[k] >= np.percentile(confn[k], self.conf_gate_pct)
            if k > 0:
                ref, f_k_ref, f_ref_k = k - 1, fwd[k], nxt[k - 1]
            else:
                ref, f_k_ref, f_ref_k = 1, nxt[0], fwd[1]
            r = self.geo.compute(imgs[k], imgs[ref], dep[k], pos[k], pos[ref], Ks[k], dep[ref], sem_mask_k=sem_all[k],
                                 measured=None if _comp else f_k_ref,
                                 measured_bwd=None if _comp else f_ref_k, extra_valid=ev)
            geo[k] = r["mask"]; ginfo[k] = r["info"]; flow_to_ref[k] = f_k_ref; gres[k] = r; raw_geo[k] = r["raw_mask"]; raw_any[k] = r["raw_mask"]
            if self.geo_two_sided and 0 < k < T - 1:
                r2 = self.geo.compute(imgs[k], imgs[k + 1], dep[k], pos[k], pos[k + 1], Ks[k], dep[k + 1], sem_mask_k=sem_all[k],
                                      measured=None if _comp else nxt[k],
                                      measured_bwd=None if _comp else fwd[k + 1], extra_valid=ev)
                if not r["info"].get("skipped") and not r2["info"].get("skipped"):
                    raw_any[k] = r["raw_mask"] | r2["raw_mask"]
                    raw_geo[k] = r["raw_mask"] & r2["raw_mask"]
                    sf = self.two_sided_strong_factor
                    if sf > 0:
                        strong = ((r["residual"] > sf * r["info"]["threshold_px"]) & r["valid"]) | \
                                 ((r2["residual"] > sf * r2["info"]["threshold_px"]) & r2["valid"])
                        raw_geo[k] |= strong & raw_any[k]
                    geo[k] = self.geo.postprocess(raw_geo[k], r["edge"])
                elif r["info"].get("skipped") and not r2["info"].get("skipped"):
                    raw_geo[k] = r2["raw_mask"]; geo[k] = r2["mask"]; gres[k] = r2; ginfo[k] = r2["info"]
                ginfo[k] = dict(ginfo[k], two_sided=True, raw_area_frac=float(raw_geo[k].mean()), mask_area_frac=float(geo[k].mean()))

        # geo gate: drop geometric blobs that are not anchored to any movable-class detection
        geo_gated = np.zeros((T, H, W), bool)
        if self.geo_gate == "anchor":
            for k in range(T):
                if not geo[k].any():
                    continue
                anchor = _dilate(sem_all[k], self.geo_anchor_px) if sem_all[k].any() else None
                n_c, lab = cv2.connectedComponents(geo[k].astype(np.uint8), connectivity=8)
                keep = np.zeros((H, W), bool)
                if anchor is not None:
                    hit = np.unique(lab[anchor & geo[k]]); hit = hit[hit > 0]
                    if hit.size:
                        keep = np.isin(lab, hit)
                geo_gated[k] = geo[k] & ~keep
                geo[k] = keep

        # object policy: decide per non-person instance whether it is moving / carried (remove) or resting (keep)
        objects: List[List[Dict]] = [[] for _ in range(T)]
        if use_instances:
            for k in range(T):
                pdil = _dilate(sem_person[k], self.attach_px) if sem_person[k].any() else None
                for d in inst[k]:
                    if d["cls"] == 0:
                        continue
                    o = dict(name=d["name"], conf=d["conf"], area=int(d["mask"].sum()), motion_frac=None, attached=False)
                    o["attached"] = bool(pdil is not None and (d["mask"] & pdil).any())
                    if gres[k] is not None and not gres[k]["info"].get("skipped"):
                        sel = d["mask"] & gres[k]["valid"]
                        if int(sel.sum()) >= self.min_valid_px:
                            o["motion_frac"] = float((gres[k]["residual"][sel] > gres[k]["info"]["threshold_px"]).mean())
                    o["moving"] = bool(o["motion_frac"] is not None and o["motion_frac"] >= self.motion_thr)
                    o["own"] = o["attached"] or o["moving"]; o["mask"] = d["mask"]
                    objects[k].append(o)
            # temporal majority vote with the matching instance in k-1 / k+1 (masks warped into frame k)
            def best_match(o, cands, flow):
                if not cands or flow is None: return None
                best, biou = None, 0.0
                for c in cands:
                    if c["name"] != o["name"]: continue
                    w = warp_mask(c["mask"], flow); inter = (w & o["mask"]).sum(); uni = (w | o["mask"]).sum()
                    iou = inter / uni if uni else 0.0
                    if iou > biou: best, biou = c, iou
                return best if biou >= self.vote_iou else None
            for k in range(T):
                for o in objects[k]:
                    votes = [o["own"]]
                    if k > 0:
                        m = best_match(o, objects[k - 1], fwd[k]);  votes += [] if m is None else [m["own"]]
                    if k < T - 1:
                        m = best_match(o, objects[k + 1], nxt[k]);  votes += [] if m is None else [m["own"]]
                    o["removed"] = (sum(votes) * 2 > len(votes)) if len(votes) != 2 else o["own"]   # tie -> own
            for k in range(T):
                rm = [o["mask"] for o in objects[k] if o["removed"]]
                sem[k] = sem_person[k] | (_clean(np.any(rm, axis=0)) if rm else False)
            for k in range(T):
                for o in objects[k]: o.pop("mask", None)

        # carried objects without a label: grow each person mask into connected raw-residual pixels at the person's depth
        grown = np.zeros((T, H, W), bool)
        if self.grow_carried and T > 1:
            for k in range(T):
                if not sem_person[k].any() or gres[k] is None or gres[k]["info"].get("skipped"):
                    continue
                pd = dep[k][sem_person[k] & (dep[k] > 0)]
                if pd.size < 50: continue
                lo_d, hi_d = np.percentile(pd, 5) - self.grow_depth_margin_m, np.percentile(pd, 95) + self.grow_depth_margin_m
                # "moves like the person": measured flow close to the person's flow (median over the person, with a
                # tolerance of 4 px + 25 % of its magnitude); a wall behind the person fails this, a carried box passes
                F = gres[k]["measured_flow"]; pf = np.median(F[sem_person[k]].reshape(-1, 2), axis=0)
                tol = 4.0 + 0.25 * float(np.linalg.norm(pf))
                like_person = np.linalg.norm(F - pf, axis=-1) <= tol
                cand = (raw_any[k] | sem_person[k]) & (dep[k] >= lo_d) & (dep[k] <= hi_d) & (like_person | sem_person[k])
                n, lab = cv2.connectedComponents(cand.astype(np.uint8), connectivity=8)
                touch = np.unique(lab[sem_person[k] & cand]); touch = touch[touch > 0]
                if touch.size:
                    g = np.isin(lab, touch) & ~sem_person[k]
                    g = _clean(g) if g.any() else g
                    if g.sum() <= self.grow_max_area_ratio * sem_person[k].sum():     # a carried object is smaller than the person
                        grown[k] = g; sem[k] = sem[k] | grown[k]

        union = sem | geo
        prop = np.zeros((T, H, W), bool)
        if prop_mode in ("union", "sem") and T > 1:
            base = union.copy() if prop_mode == "union" else sem.copy()
            for k in range(T):
                if k > 0 and fwd[k] is not None and base[k - 1].any():
                    prop[k] |= warp_mask(base[k - 1], fwd[k])
                if k < T - 1 and nxt[k] is not None and base[k + 1].any():
                    prop[k] |= warp_mask(base[k + 1], nxt[k])
            union = union | prop
        elif prop_mode == "sem_bridge" and T > 1:
            # "sem" warps a neighbour's person into frames where the person has not entered yet
            # (e.g. frame 42 <- 43 in the lab run: 27 % of the image masked on background).
            # Bridge mode only fills a *gap*: k has no semantic detection, but BOTH k-1 and k+1 do
            # (the frame-48 single-frame YOLO miss).  The chunk's first frame is the previous
            # chunk's last frame (overlap 1); its k-1 neighbour lives in the previous chunk, so the
            # tail (sem[T-2], flow T-1 -> T-2) is carried over in self._prev_tail.  The caller must
            # union the overlap frame's masks of both chunks (see run_lab_slam_fusion.py).
            for k in range(0, T - 1):
                if sem[k].any() or not sem[k + 1].any():
                    continue
                if k > 0:
                    if not sem[k - 1].any():
                        continue
                    prev_sem, prev_flow = sem[k - 1], fwd[k]
                elif self._prev_tail is not None and self._prev_tail[0].any() and np.array_equal(self._prev_tail[2], imgs[0]):
                    prev_sem, prev_flow = self._prev_tail[0], self._prev_tail[1]
                else:
                    continue
                prop[k] = warp_mask(prev_sem, prev_flow) | warp_mask(sem[k + 1], nxt[k])
            union = union | prop
        if T > 1:
            self._prev_tail = (sem[T - 2].copy(), fwd[T - 1], imgs[T - 1].copy())

        if self.record:
            for k in range(T):
                g = ginfo[k] or {}
                self.records.append(dict(global_index=self._frame_counter + k, chunk_index=k,
                                         sem_area=float(sem[k].mean()), geo_area=float(geo[k].mean()),
                                         sem_all_area=float(sem_all[k].mean()), kept_area=float((sem_all[k] & ~sem[k]).mean()),
                                         objects=objects[k], object_policy=self.object_policy, grown_area=float(grown[k].mean()),
                                         prop_added=float((prop[k] & ~(sem[k] | geo[k])).mean()),
                                         union_area=float(union[k].mean()), dets=dets[k],
                                         thr_px=g.get("threshold_px"), res_med=g.get("residual_median_px"),
                                         res_p95=g.get("residual_p95_px"), res_p99=g.get("residual_p95_px"),
                                         gflow=g.get("global_flow_median_px"), rigid_flow=g.get("rigid_flow_median_px"),
                                         fb_dropped=g.get("fb_dropped_frac"), valid_frac=g.get("valid_frac"),
                                         translation_m=g.get("translation_m"), rotation_deg=g.get("rotation_deg"),
                                         irls_inlier=g.get("irls_inlier_frac"), irls_res=g.get("irls_fit_residual_median_px"),
                                         irls_corr=g.get("irls_correction_median_px"), skipped=g.get("skipped"),
                                         geo_gated_area=float(geo_gated[k].mean()),
                                         sem=sem[k].copy(), geo=geo[k].copy(), prop=prop[k].copy(), union=union[k].copy()))
        self._frame_counter += T
        return torch.from_numpy(union)

    # ------------------------------------------------------------------ v1 (2026-09-02), kept for reference
    @torch.no_grad()
    def mask_chunk_v1(self, images, depth, poses, intrinsics) -> torch.Tensor:
        imgs = _to_np(images); dep = _to_np(depth).astype(np.float32); pos = _to_np(poses).astype(np.float64)
        K_all = _to_np(intrinsics).astype(np.float64)
        T, H, W = dep.shape
        if imgs.dtype != np.uint8:
            imgs = (np.clip(imgs, 0, 1) * 255).round().astype(np.uint8) if imgs.max() <= 1.0 else np.clip(imgs, 0, 255).astype(np.uint8)
        if imgs.shape[1] != H or imgs.shape[2] != W:
            imgs = np.stack([cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA) for im in imgs])
        Ks = K_all if K_all.ndim == 3 else np.repeat(K_all[None], T, 0)

        sem = np.zeros((T, H, W), bool); geo = np.zeros((T, H, W), bool); flows = [None] * T; ginfo = [None] * T; dets = [None] * T
        for k in range(T):
            if self.semantic is not None:
                sem[k] = self.semantic.get_mask(imgs[k], clean=True)
                dets[k] = [(d["name"], round(d["conf"], 2)) for d in self.semantic.last_detections]
        for k in range(T):
            ref = k - 1 if k > 0 else (1 if T > 1 else None)
            if ref is None:
                continue
            r = self.geo.compute(imgs[k], imgs[ref], dep[k], pos[k], pos[ref], Ks[k], dep[ref], sem_mask_k=sem[k])
            geo[k] = r["mask"]; flows[k] = r["measured_flow"]; ginfo[k] = r["info"]
        union = sem | geo
        prop = np.zeros((T, H, W), bool)
        if self.propagate and T > 1:
            base = union.copy()
            for k in range(T):
                if k > 0 and flows[k] is not None:                       # flow k -> k-1 already computed
                    prop[k] |= warp_mask(base[k - 1], flows[k])
                if k < T - 1:
                    f_next = self.flow_fn(imgs[k], imgs[k + 1])          # flow k -> k+1
                    prop[k] |= warp_mask(base[k + 1], f_next)
            union = union | prop
        if self.record:
            for k in range(T):
                g = ginfo[k] or {}
                self.records.append(dict(global_index=self._frame_counter + k, chunk_index=k,
                                         sem_area=float(sem[k].mean()), geo_area=float(geo[k].mean()),
                                         prop_added=float((prop[k] & ~(sem[k] | geo[k])).mean()),
                                         union_area=float(union[k].mean()), dets=dets[k],
                                         thr_px=g.get("threshold_px"), res_med=g.get("residual_median_px"),
                                         res_p99=g.get("residual_p99_px"), gflow=g.get("global_flow_median_px"),
                                         irls_inlier=g.get("irls_inlier_frac"), irls_res=g.get("irls_fit_residual_median_px"),
                                         sem=sem[k].copy(), geo=geo[k].copy(), prop=prop[k].copy(), union=union[k].copy()))
        self._frame_counter += T
        return torch.from_numpy(union)
