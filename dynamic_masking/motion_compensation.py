"""
motion_compensation.py — cancel the camera's own motion BEFORE the optical flow runs.

WHY
---
Today the pipeline runs FlowSeek on the raw pair and subtracts the rigid flow afterwards:

        r(u) = | FlowSeek(I_t, I_t+1)(u)  −  F_rigid(u) |

On the lab data that asks the network a very hard question: the median measured flow is
46–50 px and the median rigid flow is 45–48 px, so **96 % of what FlowSeek must explain is
just the robot driving**.  Flow networks are accurate at small displacements and degrade
quickly at large ones; the error that survives inflates the residual MAD, which raises the
adaptive threshold (median 3.9 px, p95 10.5 px) until only violent motion is detectable.
Measured on this dataset: corr(|F|, tau) = +0.39, and high-flow frames need tau 4.78 px
against 3.19 px for low-flow frames.

WHAT THIS DOES
--------------
Warp the NEXT frame back into the CURRENT frame's viewpoint using the camera motion we
already know, then run the flow on that compensated pair:

        u'            = u + F_rigid(u)                    (where a rigid point would land)
        I'_t+1(u)     = I_t+1(u')                         (backward warp / resample)
        F_res         = FlowSeek(I_t, I'_t+1)             (what the camera cannot explain)

For a static point with correct depth and pose, I'_t+1(u) ≈ I_t(u), so F_res ≈ 0.
For a moving object, F_res is exactly its independent motion.  The subtraction disappears:
the network now measures the residual directly, at a few pixels instead of fifty.

WHAT IT COSTS
-------------
* Disocclusion and out-of-frame: u' can leave the image or land on surface that was hidden.
  Those pixels get no vote — returned in `valid`.
* Depth error now corrupts the IMAGE fed to the network rather than a vector subtracted from
  its output, so a bad depth can look like texture that moved.  The depth gates still apply.
* Resampling softens I'_t+1 slightly; bicubic is used to keep that small.
"""
from __future__ import annotations
from typing import Optional, Tuple

import cv2
import numpy as np


def warp_by_flow(img: np.ndarray, flow: np.ndarray,
                 interp: int = cv2.INTER_CUBIC) -> Tuple[np.ndarray, np.ndarray]:
    """Sample `img` at u + flow(u).  Returns (warped, inside) where `inside` is False for
    pixels whose source fell outside the image and therefore carry no real evidence."""
    H, W = flow.shape[:2]
    gx, gy = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    mx = gx + flow[..., 0].astype(np.float32)
    my = gy + flow[..., 1].astype(np.float32)
    inside = (mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)
    warped = cv2.remap(img, mx, my, interp, borderMode=cv2.BORDER_REPLICATE)
    return warped, inside


def compensated_flow(flow_fn, rgb_k: np.ndarray, rgb_k1: np.ndarray,
                     rigid: np.ndarray, rigid_valid: np.ndarray,
                     fb_max_px: float = 0.0, fb_rel: float = 0.05
                     ) -> Tuple[np.ndarray, Optional[np.ndarray], dict]:
    """Run the flow on the motion-compensated pair.

    Returns
    -------
    measured_total : [H, W, 2]
        ``F_res + F_rigid`` — the TOTAL flow, so every downstream formula is unchanged.
        The residual the detector then forms, |measured_total − F_rigid|, is exactly |F_res|.
    fb_ok : [H, W] bool or None
        Forward-backward agreement measured in the COMPENSATED domain, which is where the
        decision is actually made.  None when fb checking is off.
    info : dict
        Diagnostics, including how much smaller the displacement became.
    """
    warped, inside = warp_by_flow(rgb_k1, rigid)
    res = flow_fn(rgb_k, warped)                       # the residual flow, directly

    fb_ok = None
    if fb_max_px > 0:
        res_bwd = flow_fn(warped, rgb_k)
        H, W = res.shape[:2]
        gx, gy = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
        bx = cv2.remap(res_bwd[..., 0], gx + res[..., 0], gy + res[..., 1],
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        by = cv2.remap(res_bwd[..., 1], gx + res[..., 0], gy + res[..., 1],
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        err = np.sqrt((res[..., 0] + bx) ** 2 + (res[..., 1] + by) ** 2)
        fb_ok = err < np.maximum(fb_rel * np.linalg.norm(res, axis=-1), fb_max_px)

    valid = inside & np.asarray(rigid_valid, bool)
    # NB: prefixed, because the detector's own info dict already defines rigid_flow_median_px
    # and these are merged into it.
    info = dict(
        comp_residual_median_px=float(np.median(np.linalg.norm(res[valid], axis=-1)))
            if valid.any() else float("nan"),
        comp_rigid_median_px=float(np.median(np.linalg.norm(rigid[valid], axis=-1)))
            if valid.any() else float("nan"),
        comp_warp_outside_frac=float((~inside).mean()),
        compensated=True,
    )
    return res + rigid, fb_ok, valid, info
