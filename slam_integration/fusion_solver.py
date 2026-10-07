"""
fusion_solver.py — dynamic-object masking wired into the streaming SLAM server.
================================================================================
`FusionMaSlam` subclasses `MaSlam` and overrides exactly one method,
`process_submap`.  `ma_slam/solver.py` itself is never edited: delete this file
and revert the two lines in `ma_slam_stream/server_api.py` and the server is
back to stock.

Where the mask goes in, and why
-------------------------------
`MaSlam.process_submap` (solver.py:326) has two consecutive statements:

    line 327   out = self._infer(image_paths, depth_paths)
    line 329   sm  = self._build_submap(base, image_paths, depth_paths, out)

`_infer()` returns exactly the five things `ChunkFusionMasker.mask_chunk()`
needs — images, depth, poses, intrinsics, world_points_conf — and
`_build_submap()` is what turns confidence into map points.  Setting the
confidence of dynamic pixels to 0 in the gap between them means those points
are *never created*, rather than created and deleted later.  That gap is one
statement wide and it is the only place in the system where the network output
exists but the points do not.

The hooks (`on_submap`, `on_loop`, `on_finish`) all fire at solver.py:345 or
later — after the points exist — so they cannot be used for this.

The conf_threshold trap
-----------------------
`Submap.set_geometry` (submap.py:64) derives the point-keeping threshold from
the confidence array it is handed:

    self.conf_threshold = float(np.mean(conf)) + 1e-6

Zeroing the dynamic pixels *before* that runs drags the mean down and lowers
the threshold for the whole frame, letting extra weak points in everywhere —
not just where the movers were.  Measured masked fractions: lab 4.98 %,
TUM 18.15 %, Bonn crowd 20.65 %, so on a crowded scene the bar would drop by a
fifth.  We therefore capture the *unmasked* mean before zeroing and restore it
after the submap is built.  `run_lab_slam_fusion.py:86` does the same thing
offline; this keeps the server identical to the validated offline pipeline.
"""
from __future__ import annotations

import os
import sys
from typing import Optional

import numpy as np

try:                                   # torch is always present in this container
    import torch
except Exception:                      # pragma: no cover
    torch = None

from ma_slam.solver import MaSlam

# The masker lives in <repo>/dynamic_masking/.  This file is installed into the SLAM server
# (src/ma_slam/fusion_solver.py) as a symlink by slam_integration/install.sh, so its real location
# is <repo>/slam_integration/ and the masker is found next to it.  DYNAMIC_MASK_ROOT overrides.
DYNAMIC_ROOT = os.environ.get(
    "DYNAMIC_MASK_ROOT",
    os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "dynamic_masking"))


def _import_masker():
    if DYNAMIC_ROOT not in sys.path:
        sys.path.insert(0, DYNAMIC_ROOT)
    from chunk_fusion_masker import ChunkFusionMasker      # noqa: E402
    return ChunkFusionMasker


def build_masker(device: str = "cuda", **kw):
    """Construct the masker once, at server start-up (it loads YOLO + FlowSeek).

    Mirrors run_lab_slam_fusion.py's `g_two_s5_grow` configuration, which is the
    one the reported results were produced with.
    """
    ChunkFusionMasker = _import_masker()
    from dynamic_fusion import GeometricMaskConfig         # noqa: E402
    from dynamic_object_mask import DynamicObjectDetector  # noqa: E402

    # person_conf 0.15 (others 0.25) — run_lab_slam_fusion.py:124's validated default
    semantic = DynamicObjectDetector(
        os.path.join(DYNAMIC_ROOT, "yolov9e-seg.pt"), device=device,
        class_conf={0: 0.15})
    # Retuned 2026-09-24 against mapping_20260922_01 (413 chunk-frames, sweep_fix*.csv).
    # The shipped adaptive_k=4.0 left the geometric channel SILENT in 50 % of the frames that
    # contain a person, while spending only 0.25 % of its false-positive budget -- far too
    # conservative for this capture.  Measured recall (geometric channel fires when a person is
    # present) against false positives on the 237 person-free frames:
    #
    #     adaptive_k  guards  fill   recall   FP frames   FP area   frames declined
    #        4.0        no     no      50 %      17 %      0.0025          3     <- was shipped
    #        3.0        no     yes     60 %      28 %      0.0056          3
    #        3.0       yes     yes     61 %      25 %      0.0044         85     <- adopted
    #        2.5        no     yes     70 %      38 %      0.0090          3
    #        2.5       yes     yes     68 %      32 %      0.0064         84
    #        2.5   tighter     yes     66 %      30 %      0.0068        141
    #        3.0   tighter     yes     58 %      23 %      0.0046        147
    #
    # The guards pay for themselves: at the same k they hold recall and cut false positives,
    # because the frames they decline are the ones where the flow was untrustworthy anyway.
    # Tightening them further (6.0 px / sharpness 50) only discards good frames.
    # k=2.5 buys 7 more points of recall for 45 % more false-positive area; for a mapping
    # system a false positive deletes real structure, so k=3.0 is the default.  Pass
    # adaptive_k=2.5 through **kw if recall matters more than fidelity for a given run.
    geo_cfg = GeometricMaskConfig(
        ego_motion="pose",      # rigid flow from the DA3 poses
        adaptive_k=3.0,         # tau = median + 3 * 1.4826 * MAD
        fb_max_px=1.5,          # forward-backward flow consistency
        rel_residual=0.0,
        residual_mode="full",
        # A person's interior loses its vote wherever a validity gate fires, so the raw mask is
        # a shell.  Fills only holes fully enclosed by the mask (border flood-fill), so it
        # cannot grow outwards into the scene.
        fill_holes=True,
        fill_close_px=15,
        # This capture's frames are a median 409 ms apart (p90 2.55 s) because the collector
        # gates on 25 px of LK disparity, and 29 % of them are below Laplacian variance 50.
        # Across such a pair the flow is guesswork: decline instead of emitting noise, and let
        # the semantic channel carry the frame alone.
        unreliable_threshold_px=8.0,
        min_sharpness=30.0,
    )
    params = dict(
        # propagate: "sem_bridge", NOT "sem".  "sem" warps a neighbour's person into
        # frames the person has not entered yet — chunk_fusion_masker.py:274 records 27 %
        # of a lab frame masked on pure background that way.  "sem_bridge" only fills a
        # genuine single-frame YOLO gap (k-1 and k+1 both detect, k does not) and is the
        # mode run_lab_slam_fusion.py:123 defaults to for the reported results.
        semantic=semantic, geo_cfg=geo_cfg, propagate="sem_bridge",
        object_policy="moving_or_carried", attach_px=10, motion_thr=0.30,
        geo_two_sided=True, two_sided_strong_factor=5.0,
        grow_carried=True, conf_gate_pct=0.0,
        # 2026-09-28: keep a geometric (optical-flow) blob only if it touches a movable-class detection.
        # Unanchored blobs on the 0922 capture were static depth edges seen while the camera turned.
        # Set DYNAMIC_GEO_GATE=none (container env) to get the previous behaviour back.
        geo_gate=os.environ.get("DYNAMIC_GEO_GATE", "anchor"), geo_anchor_px=30,
        # 2026-10-01, against person pixels left in the map (map803 run5 vs SAM 3 reference:
        # 1.96 % -> 0.79 % of person pixels left): grow person masks (edge / flying pixels), fill the
        # person's detection box at the person's depth (parts hidden behind an occluder), and bridge
        # up to 3 frames without a detection (motion blur; entering/leaving through the image border).
        # Box fill also extends 0.6 x box height downwards (legs under furniture) and depth-edge
        # pixels within 12 px of the mask are removed (flying-pixel line). Set the env vars to
        # 0 / 0 / 1 / 0 / 0 for the behaviour before 2026-10-01.
        person_dilate_px=int(os.environ.get("DYNAMIC_PERSON_DILATE_PX", "5")),
        person_box_fill=os.environ.get("DYNAMIC_PERSON_BOX_FILL", "0") == "1",   # off since 2026-10-05: filled walls/desks
        bridge_max_gap=int(os.environ.get("DYNAMIC_BRIDGE_MAX_GAP", "3")),
        box_fill_down=float(os.environ.get("DYNAMIC_BOX_FILL_DOWN", "0.3")),
        edge_ring_px=int(os.environ.get("DYNAMIC_EDGE_RING_PX", "8")),
        # improvement #2 (2026-10-05): touch (default, unchanged) / touch_and_moving /
        # touch_and_moving_or_held (= moving OR held in the hand; keeps still objects beside people)
        carried_rule=os.environ.get("DYNAMIC_CARRIED_RULE", "touch"),
        leg_fill=os.environ.get("DYNAMIC_LEG_FILL", "1") == "1",
    )
    params.update(kw)
    return ChunkFusionMasker(**params)


class FusionMaSlam(MaSlam):
    """MaSlam + per-chunk dynamic-object masking.  `masker=None` == plain MaSlam."""

    def __init__(self, *args, masker=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._masker = masker
        # multi-view carving of the written map (see carve_dynamic_ply): wrap this session's
        # GraphMap.write_points so every writer (offline run(), stream _finalize) gets it, before
        # deploy reads combined_pcd.ply.
        if masker is not None and os.environ.get("DYNAMIC_CARVE", "1") == "1":
            _orig_write = self.map.write_points

            def _write_and_carve(graph, path, *a, **kw):
                if os.environ.get("DYNAMIC_DUMP_RECON", "0") == "1":
                    try:
                        dump_recon_cache(self, graph, os.path.join(os.path.dirname(path), "recon_cache"))
                    except Exception as exc:
                        print(f"[fusion] WARNING: recon cache dump failed: {type(exc).__name__}: {exc}")
                r = _orig_write(graph, path, *a, **kw)
                try:
                    carve_dynamic_ply(self, graph, path)
                except Exception as exc:                   # never lose the map over this
                    print(f"[fusion] WARNING: carving skipped: {type(exc).__name__}: {exc}")
                if os.environ.get("DYNAMIC_FUSED_MAP", "1") == "1":
                    try:
                        export_fused_map(self, graph, os.path.dirname(path))
                    except Exception as exc:
                        print(f"[fusion] WARNING: fused map skipped: {type(exc).__name__}: {exc}")
                return r
            self.map.write_points = _write_and_carve
        self.stats.setdefault("n_masked_submaps", 0)
        self.stats.setdefault("masked_frac_sum", 0.0)

    # ------------------------------------------------------------------ run
    def process_submap(self, image_paths, depth_paths):
        # The server keeps one SALAD retrieval object resident across sessions (server_api.py:152)
        # and never empties its database, so the 2nd session on a running server matched against the
        # previous recording's submap ids -> KeyError in _add_loop_constraint. Start each session
        # (first submap) with an empty database. (2026-10-05)
        r = getattr(self, "retrieval", None)
        if self.stats.get("n_submaps", 0) == 0 and r is not None and getattr(r, "_descs", None):
            print(f"[fusion] clearing {len(r._descs)} loop-retrieval entries left from a previous session")
            r._descs.clear(); r._meta.clear()
        out = self._infer(image_paths, depth_paths)            # solver.py:327

        # ---------------- dynamic masking, in the 327 -> 329 gap ----------------
        conf_mean: Optional[float] = None
        dyn_keep = None
        dyn_core = None                                        # people / moving objects only (dynamic_pcd.ply)
        dyn_ch = None                                          # per-channel masks, display only
        conf_raw_keep = np.asarray(out["world_points_conf"], dtype=np.float32).copy()
        if self._masker is not None:
            try:
                K = out.get("intrinsics")
                if K is None:
                    K = self._input_K
                dyn = self._masker.mask_chunk(
                    images=out["images"], depth=out["depth"], poses=out["poses"],
                    intrinsics=K, conf=out["world_points_conf"])
                if torch is not None and torch.is_tensor(dyn):
                    dyn = dyn.detach().cpu().numpy()
                dyn = np.asarray(dyn, dtype=bool)
                # ChunkFusionMasker(record=True) appends one record per frame holding full-size
                # sem/geo/prop masks and never drops them (~1 MB/frame for a whole run): take this
                # chunk's channels for the display, then free them.
                recs = getattr(self._masker, "records", None)
                if recs:
                    mine = recs[-dyn.shape[0]:]
                    if len(mine) == dyn.shape[0] and all("sem" in r for r in mine):
                        dyn_ch = {k: np.stack([r[k] for r in mine]) for k in ("sem", "geo", "prop")}
                        if all("ch_yolo_person" in r for r in mine):
                            dyn_core = np.zeros_like(dyn)
                            for kk in ("ch_yolo_person", "ch_person_fill", "ch_obj_removed", "ch_carried_grown", "prop", "geo"):
                                dyn_core |= np.stack([r[kk] for r in mine])
                            dyn_core = depth_consistent_core(dyn_core & dyn, np.asarray(out["depth"], np.float32))
                        # per-channel masks to <run>/mask_channels/ for diagnosis (DYNAMIC_DUMP_MASKS=1)
                        if os.environ.get("DYNAMIC_DUMP_MASKS", "0") == "1" and self.mask_viz_dir:
                            try:
                                d = os.path.join(os.path.dirname(self.mask_viz_dir), "mask_channels")
                                os.makedirs(d, exist_ok=True)
                                keys = [k for k in mine[0] if k.startswith("ch_")] + ["sem", "geo", "prop", "union"]
                                np.savez_compressed(os.path.join(d, f"chunk_{self.stats['n_submaps']:04d}.npz"),
                                                    image_paths=np.array(list(image_paths)), shape=np.array(dyn.shape),
                                                    final=np.packbits(dyn),
                                                    objects=np.array(__import__("json").dumps([r.get("objects") for r in mine], default=str)),
                                                    **{k: np.packbits(np.stack([r[k] for r in mine])) for k in keys})
                            except Exception as exc:
                                print(f"[fusion] WARNING: mask channel dump failed: {exc}")
                    recs.clear()

                conf = np.asarray(out["world_points_conf"], dtype=np.float32)
                if dyn.shape != conf.shape:                    # never silently mis-apply
                    raise ValueError(
                        f"mask shape {dyn.shape} != conf shape {conf.shape}")

                conf_mean = float(np.mean(conf))               # BEFORE zeroing
                conf_raw_keep = conf.copy()                    # unmasked, for the exports
                conf = conf.copy()
                conf[dyn] = 0.0
                out["world_points_conf"] = conf
                dyn_keep = dyn

                frac = float(dyn.mean())
                self.stats["n_masked_submaps"] += 1
                self.stats["masked_frac_sum"] += frac
                print(f"[fusion] submap masked {100.0 * frac:.2f}% of pixels "
                      f"({int(dyn.sum()):,} px)")
            except Exception as exc:                           # never kill a live run
                conf_mean = None
                print(f"[fusion] WARNING: masking skipped for this submap: "
                      f"{type(exc).__name__}: {exc}")
        # ------------------------------------------------------------------------

        base = self._next_base
        sm = self._build_submap(base, image_paths, depth_paths, out)   # solver.py:329

        if conf_mean is not None:
            # submap.py:64 derived this from the *masked* mean; put the real one back
            sm.conf_threshold = conf_mean + 1e-6
        # keep what is needed to export static-only / dynamic-only clouds later
        sm.dynamic_masks = dyn_keep
        sm.dynamic_core = dyn_core if (dyn_core is not None and dyn_keep is not None) else None
        sm.conf_raw = conf_raw_keep

        # ---- remainder copied verbatim from MaSlam.process_submap (330-345) ----
        prev = self.map.latest(ignore_lc=True)
        # The overlap frame is shared: the previous submap's last frame IS this chunk's frame 0, and its
        # points are kept in both submaps. The previous chunk decided that frame's mask without seeing
        # later frames (e.g. a person detected only from the next frame on), so apply this chunk's
        # frame-0 mask to the previous submap's copy too. Export reads sm.conf at write time, so zeroing
        # it here removes those points from the map. (2026-10-01)
        if (dyn_keep is not None and prev is not None and getattr(prev, "image_paths", None)
                and sm.image_paths and prev.image_paths[-1] == sm.image_paths[0]
                and getattr(prev, "conf", None) is not None and prev.conf[-1].shape == dyn_keep[0].shape):
            prev.conf[-1] = np.where(dyn_keep[0], 0.0, prev.conf[-1])
            if getattr(prev, "dynamic_masks", None) is not None:
                prev.dynamic_masks[-1] = prev.dynamic_masks[-1] | dyn_keep[0]
            if getattr(prev, "dynamic_core", None) is not None and sm.dynamic_core is not None:
                prev.dynamic_core[-1] = prev.dynamic_core[-1] | sm.dynamic_core[0]
        if prev is None:
            self._add_first_submap(sm)
        else:
            self._add_submap(sm, prev)   # placement + scale handled inside (overlap align)
        self.map.add(sm)
        self._next_base += sm.n
        self.stats["n_submaps"] += 1

        if self.retrieval is not None and prev is not None:
            self._loop_closure(sm)

        self.graph.optimize()

        if self.hooks is not None:
            self.hooks.on_submap(sm, self.map, self.graph)   # post-optimize poses

        self._publish_mask_viz(sm, dyn_keep, dyn_ch)

    # ------------------------------------------------------------ live mask viz
    # Set by the server Session (server_api.py). Display only: nothing below changes
    # which points enter the map.
    mask_viz_dir: Optional[str] = None     # <run>/mask_viz/  (jpg per frame)
    viz_rr = None                          # rerun module when the web viewer is on
    VIZ_MAX_REMOVED_PTS = 30_000           # hard cap per submap, for the red 3D layer
    viz_conf_coef: float = 1.0             # same point filter as the map layer (RerunViz)
    viz_max_points: int = 500_000          # same point budget as the map layer (RerunViz)

    def _publish_mask_viz(self, sm, dyn, ch=None):
        """Show what the masker removed, while mapping runs.

        * <run>/mask_viz/<frame>.jpg  — [camera | removed pixels], coloured by the channel that
          removed them: RED = semantic (person / moving-or-carried object), YELLOW = optical-flow
          motion only, BLUE = temporal bridge only (live_view.py --mask streams these)
        * <run>/mask_viz/removed.csv  — per frame: total and per-channel % of pixels
        * rerun camera/mask_overlay   — the same panel in the web viewer
        * rerun world/removed/*       — removed pixels back-projected as red 3D points
        """
        if dyn is None or (self.mask_viz_dir is None and self.viz_rr is None):
            return
        try:
            import cv2
            rr = self.viz_rr
            if self.mask_viz_dir:
                os.makedirs(self.mask_viz_dir, exist_ok=True)
            red = np.array([255, 0, 0], dtype=np.float32)
            colours = ((np.array([255, 0, 0], np.float32), (255, 0, 0)),       # semantic
                       (np.array([255, 210, 0], np.float32), (255, 210, 0)),   # motion only
                       (np.array([0, 120, 255], np.float32), (0, 120, 255)))   # bridge only
            if ch is not None and ch["sem"].shape != dyn.shape:
                ch = None
            def split(i):                                  # (sem, motion-only, bridge-only)
                if ch is None:
                    return (dyn[i], np.zeros_like(dyn[i]), np.zeros_like(dyn[i]))
                s_ = ch["sem"][i] & dyn[i]
                g_ = ch["geo"][i] & dyn[i] & ~s_
                return (s_, g_, dyn[i] & ~s_ & ~g_)
            # red 3D layer: same confidence filter and same sampling rate as the map layer, so red and
            # map points are directly comparable (before 2026-09-28 red used a lower threshold and a
            # larger budget, which over-drew removed points ~2-3x and let in low-confidence edge points)
            thr = sm.conf_threshold * self.viz_conf_coef
            n_sub = max(1, sum(1 for s_ in self.map.ordered() if not getattr(s_, "is_lc", False)))
            budget = max(1, self.viz_max_points // n_sub)
            valid = sm.conf_raw > thr
            if getattr(sm, "mask", None) is not None:
                valid &= sm.mask
            n_kept = int((valid & ~dyn).sum())
            rate = min(1.0, budget / max(1, n_kept))
            removed = []
            if self.mask_viz_dir:                          # per-frame timeline
                csv = os.path.join(self.mask_viz_dir, "removed.csv")
                new = not os.path.exists(csv)
                with open(csv, "a") as fh:
                    if new:
                        fh.write("frame,removed_pct,semantic_pct,motion_only_pct,bridge_only_pct\n")
                    for i in range(sm.n):
                        a, b_, c_ = split(i)
                        fh.write(f"{sm.key(i)},{100.0 * dyn[i].mean():.3f},{100.0 * a.mean():.3f},"
                                 f"{100.0 * b_.mean():.3f},{100.0 * c_.mean():.3f}\n")
            for i in range(sm.n):
                m = dyn[i]
                rgb = np.ascontiguousarray(sm.colors[i], dtype=np.uint8)
                over = rgb.copy()
                parts = split(i)
                for mm, (fill, edge) in zip(parts, colours):
                    if mm.any():
                        over[mm] = (0.45 * rgb[mm] + 0.55 * fill).astype(np.uint8)
                        cnts, _ = cv2.findContours(mm.astype(np.uint8), cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(over, cnts, -1, edge, 2)
                frame = sm.key(i)
                label = (f"frame {frame}  removed {100.0 * m.mean():.1f}%  "
                         f"(yolo {100.0 * parts[0].mean():.1f} / motion {100.0 * parts[1].mean():.1f})")
                for col, w in (((0, 0, 0), 3), ((255, 255, 255), 1)):
                    cv2.putText(over, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                col, w, cv2.LINE_AA)
                panel = np.concatenate([rgb, over], axis=1)
                if self.mask_viz_dir:
                    cv2.imwrite(os.path.join(self.mask_viz_dir, f"{frame:06d}.jpg"),
                                cv2.cvtColor(panel, cv2.COLOR_RGB2BGR),
                                [cv2.IMWRITE_JPEG_QUALITY, 85])
                if rr is not None:
                    rr.set_time("frame", sequence=frame)
                    rr.log("camera/mask_overlay", rr.Image(panel).compress(jpeg_quality=80))
                    sel = m & valid[i]
                    if sel.any():
                        M = self.graph.get_pose(sm.key(i)) @ np.linalg.inv(sm.poses_local[i])
                        Q = sm.points[i][sel]
                        removed.append(Q @ M[:3, :3].T + M[:3, 3])
            if rr is not None and removed:
                P = np.concatenate(removed).astype(np.float32)
                n_draw = min(self.VIZ_MAX_REMOVED_PTS, int(round(len(P) * rate)))
                if n_draw < len(P):
                    idx = np.random.default_rng(sm.base_id).choice(len(P), n_draw, replace=False)
                    P = P[idx]
                rr.log(f"world/removed/submap_{sm.base_id}",
                       rr.Points3D(P, colors=[255, 0, 0]))
        except Exception as exc:                           # never kill a live run
            print(f"[fusion] mask viz skipped: {type(exc).__name__}: {exc}")


# ---------------------------------------------------------------- extra exports
def _write_ply(path, xyz, rgb):
    hdr = ("ply\nformat binary_little_endian 1.0\n"
           f"element vertex {len(xyz)}\n"
           "property float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\n"
           "end_header\n").encode()
    v = np.empty(len(xyz), dtype=[("x","<f4"),("y","<f4"),("z","<f4"),
                                  ("red","u1"),("green","u1"),("blue","u1")])
    v["x"],v["y"],v["z"] = xyz[:,0],xyz[:,1],xyz[:,2]
    v["red"],v["green"],v["blue"] = rgb[:,0],rgb[:,1],rgb[:,2]
    with open(path,"wb") as fh:
        fh.write(hdr); fh.write(v.tobytes())


def _read_ply_xyzrgb(path):
    with open(path, "rb") as fh:
        n = 0
        while True:
            line = fh.readline().decode().strip()
            if line.startswith("element vertex"):
                n = int(line.split()[-1])
            if line == "end_header":
                break
        dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
        v = np.frombuffer(fh.read(n * dt.itemsize), dtype=dt, count=n)
    return np.stack([v["x"], v["y"], v["z"]], 1), np.stack([v["red"], v["green"], v["blue"]], 1)


def dump_recon_cache(slam, graph, out_dir):
    """Save every non-LC submap frame's raw reconstruction inputs (world points after the final
    graph, colours, raw/masked confidence, the submap's threshold, the edge/flying validity mask,
    the dynamic mask, camera pose, intrinsics, camera-frame depth, image path) so map-building
    variants (fusion, thresholds, blur weighting) can be compared offline on identical data.
    One npz per submap in <run>/recon_cache/. (2026-10-05, DYNAMIC_DUMP_RECON=1)"""
    os.makedirs(out_dir, exist_ok=True)
    n = 0
    for sm in slam.map.ordered():
        if sm.is_lc:
            continue
        W_pts, c2w = [], []
        for i in range(sm.n):
            M = graph.get_pose(sm.key(i)) @ np.linalg.inv(sm.poses_local[i])
            Q = sm.points[i].reshape(-1, 3)
            W_pts.append((Q @ M[:3, :3].T + M[:3, 3]).reshape(sm.points[i].shape).astype(np.float32))
            c2w.append(graph.get_pose(sm.key(i)))
        dyn = sm.dynamic_masks if getattr(sm, "dynamic_masks", None) is not None else np.zeros(sm.conf.shape, bool)
        np.savez(os.path.join(out_dir, f"submap_{sm.base_id:06d}.npz"),
                 points=np.stack(W_pts), colors=np.asarray(sm.colors, np.uint8),
                 conf=np.asarray(sm.conf, np.float16),
                 conf_raw=np.asarray(getattr(sm, "conf_raw", sm.conf), np.float16),
                 conf_threshold=np.float32(sm.conf_threshold),
                 valid=(np.asarray(sm.mask, bool) if getattr(sm, "mask", None) is not None else np.ones(sm.conf.shape, bool)),
                 dyn=np.asarray(dyn, bool), c2w=np.stack(c2w).astype(np.float64),
                 K=np.asarray(sm.intrinsics, np.float64),
                 depth=(np.asarray(sm.depth, np.float16) if sm.depth is not None else np.zeros(sm.conf.shape, np.float16)),
                 image_paths=np.array(sm.image_paths), base_id=sm.base_id)
        n += sm.n
    print(f"[fusion] recon cache: {n} frames -> {out_dir}")


def depth_consistent_core(core, depth, margin=0.3):
    """Keep, per connected region of the core mask, only pixels within its own depth range
    (p10 - margin .. p90 + margin): wall pixels seen through an arm/body gap or at the outline are
    not part of the person and must not land in dynamic_pcd.ply."""
    import cv2
    out = np.zeros_like(core)
    for k in range(core.shape[0]):
        if not core[k].any():
            continue
        n, lab = cv2.connectedComponents(core[k].astype(np.uint8), connectivity=8)
        dk = depth[k]
        for c in range(1, n):
            m = lab == c
            d = dk[m & (dk > 0)]
            if d.size < 50:
                out[k] |= m
                continue
            lo, hi = np.percentile(d, 10) - margin, np.percentile(d, 90) + margin
            out[k] |= m & (dk >= lo) & (dk <= hi)
    return out


def _sensor_view(slam, sm, i, dyn):
    """The D435 depth of submap frame i with the calibrated K, and the dynamic mask mapped from the
    model grid (MapAnything: scale by H_model/H_sensor, centre-crop the width) to sensor pixels.
    The model's own depth is smooth and its K is off by ~15 px (checked 2026-10-05); the sensor depth
    is what tells a person from the wall behind. None if no sensor depth / K (rgb-only runs)."""
    if os.environ.get("DYNAMIC_CARVE_DEPTH", "sensor") != "sensor":
        return None
    dps = getattr(sm, "depth_paths", None); K = getattr(slam, "_input_K", None)
    if not dps or K is None or i >= len(dps) or dps[i] is None:
        return None
    try:
        from PIL import Image
        Z = np.asarray(Image.open(dps[i]), np.float32) / float(getattr(slam, "_depth_scale", 1000.0) or 1000.0)
    except Exception:
        return None
    if Z.ndim != 2:
        return None
    Hs, Ws = Z.shape; Hm, Wm = dyn.shape
    sc = Hm / Hs; crop = (Ws * sc - Wm) / 2.0
    vv, uu = np.mgrid[0:Hs, 0:Ws]
    xm = np.round(uu * sc - crop).astype(int); ym = np.round(vv * sc).astype(int)
    inside = (xm >= 0) & (xm < Wm) & (ym >= 0) & (ym < Hm)
    dyn_s = np.zeros((Hs, Ws), bool)
    dyn_s[inside] = dyn[ym[inside], xm[inside]]
    K = np.asarray(K, np.float64).reshape(3, 3)
    return Z, [float(K[0, 0]), float(K[0, 2]), float(K[1, 1]), float(K[1, 2])], dyn_s


def export_fused_map(slam, graph, out_dir, voxel=0.005, trunc=0.015, conf_coef=0.75):
    """combined_fused.ply: TSDF fusion of every kept pixel (confident, valid, not dynamic) with the
    final poses. Stacking per-frame point maps leaves several offset copies of each surface (median
    surface thickness 9-12 mm on the 30 Sept runs); fusion averages them into one surface (5-7 mm)
    and the free space seen by other frames erases transient leftovers. (2026-10-06)"""
    import open3d as o3d
    vol = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel, sdf_trunc=trunc, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    dmax = float(getattr(slam, "_depth_max", None) or 5.0)
    n = 0
    for sm in slam.map.ordered():
        if sm.is_lc or sm.depth is None:
            continue
        thr = sm.conf_threshold * conf_coef
        for i in range(sm.n):
            keep = sm.conf[i] > thr
            if getattr(sm, "mask", None) is not None:
                keep &= sm.mask[i]
            D = np.where(keep, np.asarray(sm.depth[i], np.float32), 0).astype(np.float32)
            H, W = D.shape; K = np.asarray(sm.intrinsics[i], np.float64)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(np.ascontiguousarray(np.asarray(sm.colors[i], np.uint8))),
                o3d.geometry.Image(np.ascontiguousarray(D)), depth_scale=1.0, depth_trunc=dmax,
                convert_rgb_to_intensity=False)
            vol.integrate(rgbd, o3d.camera.PinholeCameraIntrinsic(W, H, K[0, 0], K[1, 1], K[0, 2], K[1, 2]),
                          np.linalg.inv(graph.get_pose(sm.key(i))))
            n += 1
    pc = vol.extract_point_cloud()
    xyz = np.asarray(pc.points, np.float32); rgb = (np.asarray(pc.colors) * 255 + 0.5).clip(0, 255).astype(np.uint8)
    path = os.path.join(out_dir, "combined_fused.ply")
    _write_ply(path, xyz, rgb)
    print(f"[fusion] fused map: {len(xyz):,} points from {n} frames -> {path}")
    return path


def carve_dynamic_ply(slam, graph, path, min_dyn=None, max_static=None):
    """Multi-view check of the written map (2026-10-01).

    The per-pixel mask only keeps a frame's OWN dynamic pixels out of the map. A few points still
    land on a person: depth predicted for an unmasked pixel of another frame (blur, mixed edge
    depth) can place a point exactly where a person stands. Each map point is therefore projected
    into every frame; where that frame's depth agrees with the point (|z - Z_f| < 3 cm + 2 % z) the
    frame "sees" it, either on a removed (dynamic) pixel or on a kept one. A point that is seen on a
    dynamic pixel at least `min_dyn` times and on a kept pixel at most `max_static` times (its own
    source frame is one kept observation) was never confirmed as static and is removed. Static
    surfaces a person touches (stool, box) are seen unmasked from other frames and stay.
    Removed points go to carved_pcd.ply next to the map.
    """
    min_dyn = int(os.environ.get("DYNAMIC_CARVE_MIN_DYN", "1")) if min_dyn is None else min_dyn
    max_static = int(os.environ.get("DYNAMIC_CARVE_MAX_STATIC", "1")) if max_static is None else max_static
    frames = _observation_frames(slam, graph)
    if not frames or not any(f[3].any() for f in frames.values()):
        return 0
    xyz, rgb = _read_ply_xyzrgb(path)
    n_dyn, n_sta = _count_observations(frames, xyz)
    rm = (n_dyn >= min_dyn) & (n_sta <= max_static)
    if rm.any():
        _write_ply(path, xyz[~rm].astype(np.float32), rgb[~rm].astype(np.uint8))
        _write_ply(os.path.join(os.path.dirname(path), "carved_pcd.ply"), xyz[rm].astype(np.float32), rgb[rm].astype(np.uint8))
    print(f"[fusion] carving: {int(rm.sum()):,} of {len(xyz):,} map points removed "
          f"(seen on dynamic pixels >= {min_dyn}x, on kept pixels <= {max_static}x, {len(frames)} frames)")
    return int(rm.sum())


def _observation_frames(slam, graph):
    """Per unique frame: [c2w, (fx, cx, fy, cy), depth image, dynamic mask] -- the D435 depth with the
    calibrated K when available (_sensor_view), else the model depth/K."""
    frames = {}                                    # image path -> [c2w, K, Z, dyn]; shared frames merged
    for sm in slam.map.ordered():
        if sm.is_lc or getattr(sm, "dynamic_masks", None) is None:
            continue
        for i in range(sm.n):
            key = sm.image_paths[i] if sm.image_paths else (sm.base_id, i)
            dyn = np.asarray(sm.dynamic_masks[i], bool)
            if key in frames:
                sens = _sensor_view(slam, sm, i, dyn) if frames[key][3].shape != dyn.shape else None
                frames[key][3] = frames[key][3] | (sens[2] if sens is not None else dyn)
                continue
            if sm.depth is not None:
                Z = np.asarray(sm.depth[i], np.float32)
            else:                                  # camera-frame Z from the local points
                L = np.linalg.inv(sm.poses_local[i])
                Z = (sm.points[i].reshape(-1, 3) @ L[:3, :3].T + L[:3, 3])[:, 2].reshape(dyn.shape).astype(np.float32)
            if getattr(sm, "mask", None) is not None:
                Z = np.where(sm.mask[i], Z, 0)
            if Z.shape != dyn.shape:
                continue
            Kf = [float(x) for x in np.asarray(sm.intrinsics[i]).reshape(-1)[[0, 2, 4, 5]]]
            sens = _sensor_view(slam, sm, i, dyn)                # (Z, K, dyn) at sensor resolution, or None
            if sens is not None:
                Z, Kf, dyn = sens
            frames[key] = [graph.get_pose(sm.key(i)), Kf, Z, dyn]
    return frames


def _count_observations(frames, xyz):
    """For each world point: in how many frames is it seen at the measured depth
    (|z - Z| < 3 cm + 2 % z) on a dynamic pixel, and on a kept (static) pixel."""
    dev = "cuda" if (torch is not None and torch.cuda.is_available()) else "cpu"
    P = torch.from_numpy(xyz.astype(np.float32)).to(dev)
    n_dyn = torch.zeros(len(P), dtype=torch.int32, device=dev); n_sta = torch.zeros_like(n_dyn)
    for c2w, K, Z, dyn in frames.values():
        H, W = Z.shape
        R = torch.from_numpy(c2w[:3, :3].astype(np.float32)).to(dev); t = torch.from_numpy(c2w[:3, 3].astype(np.float32)).to(dev)
        Zt = torch.from_numpy(Z).to(dev); Dt = torch.from_numpy(dyn).to(dev)
        for a in range(0, len(P), 4_000_000):
            Pc = (P[a:a + 4_000_000] - t) @ R                   # world -> camera
            z = Pc[:, 2]; zs = z.clamp(min=1e-6)
            fx, cx, fy, cy = K
            u = torch.round(fx * Pc[:, 0] / zs + cx).long(); v = torch.round(fy * Pc[:, 1] / zs + cy).long()
            ok = (z > 1e-3) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
            uc, vc = u.clamp(0, W - 1), v.clamp(0, H - 1)
            zf = Zt[vc, uc]
            seen = ok & (zf > 0) & ((z - zf).abs() < 0.03 + 0.02 * z)
            d = Dt[vc, uc]
            n_dyn[a:a + 4_000_000] += (seen & d).int(); n_sta[a:a + 4_000_000] += (seen & ~d).int()
    return n_dyn.cpu().numpy(), n_sta.cpu().numpy()


def export_split_clouds(slam, out_dir, max_points=2_000_000, conf_coef=0.75):
    """Write all_points_pcd.ply / static_only_pcd.ply / dynamic_pcd.ply.

    Same selection maths as ma_slam.map.write_points, but the per-pixel keep mask is
    intersected with (or inverted against) the dynamic mask this solver stored on each
    submap.  Mirrors run_lab_slam_fusion.py's export_cloud so the server's outputs are
    directly comparable with the offline pipeline's.
    """
    nonlc = [sm for sm in slam.map.ordered() if not sm.is_lc]
    if not nonlc or not any(getattr(sm, "dynamic_masks", None) is not None for sm in nonlc):
        return {}
    budget = max(1, max_points // max(1, len(nonlc)))
    rng = np.random.default_rng(0)
    sels = {
        "all_points_pcd.ply":   lambda sm, i: np.ones(sm.conf_raw[i].shape, bool),
        "static_only_pcd.ply":  lambda sm, i: (np.ones(sm.conf_raw[i].shape, bool)
                                               if sm.dynamic_masks is None else ~sm.dynamic_masks[i]),
        # people and moving objects only: the core detections, without the safety margins (dilation,
        # edge ring) that keep person edges out of the map but are background pixels (2026-10-06)
        "dynamic_pcd.ply":      lambda sm, i: (np.zeros(sm.conf_raw[i].shape, bool) if sm.dynamic_masks is None
                                               else sm.dynamic_core[i] if getattr(sm, "dynamic_core", None) is not None
                                               else sm.dynamic_masks[i]),
    }
    counts = {}
    for name, select in sels.items():
        pts, cols = [], []
        for sm in nonlc:
            if getattr(sm, "conf_raw", None) is None:
                continue
            thr = sm.conf_threshold * conf_coef
            P, C = [], []
            for i in range(sm.n):
                m = (sm.conf_raw[i] > thr) & select(sm, i)
                if getattr(sm, "mask", None) is not None:
                    m = m & sm.mask[i]
                if not m.any():
                    continue
                M = slam.graph.get_pose(sm.key(i)) @ np.linalg.inv(sm.poses_local[i])
                Q = sm.points[i].reshape(-1, 3)[m.reshape(-1)]
                P.append(Q @ M[:3, :3].T + M[:3, 3])
                C.append(sm.colors[i].reshape(-1, 3)[m.reshape(-1)])
            if not P:
                continue
            P = np.concatenate(P); C = np.concatenate(C)
            if len(P) > budget:
                idx = rng.choice(len(P), budget, replace=False); P, C = P[idx], C[idx]
            pts.append(P); cols.append(C)
        xyz = np.concatenate(pts).astype(np.float32) if pts else np.zeros((0,3), np.float32)
        rgb = np.concatenate(cols).astype(np.uint8) if cols else np.zeros((0,3), np.uint8)
        if name == "dynamic_pcd.ply" and len(xyz) and os.environ.get("DYNAMIC_STATIC_EVIDENCE", "1") == "1":
            # a removed point that other frames see at the same 3D spot on a KEPT pixel (nobody there) is
            # static -- e.g. a person's YOLO outline spilling onto the pillar they lean on. People are never
            # seen as empty background at their own position. (2026-10-06)
            try:
                frames = _observation_frames(slam, slam.graph)
                if frames:
                    _, n_sta = _count_observations(frames, xyz.astype(np.float64))
                    static = n_sta >= int(os.environ.get("DYNAMIC_STATIC_EVIDENCE_MIN", "2"))
                    print(f"[fusion] dynamic_pcd: {int(static.sum()):,} of {len(xyz):,} points seen as static elsewhere -> dropped")
                    xyz, rgb = xyz[~static], rgb[~static]
            except Exception as exc:
                print(f"[fusion] WARNING: static-evidence filter skipped: {type(exc).__name__}: {exc}")
        path = os.path.join(out_dir, name)
        _write_ply(path, xyz, rgb)
        counts[name] = len(xyz)
        print(f"[fusion] {name}: {len(xyz):,} points")
    return counts
