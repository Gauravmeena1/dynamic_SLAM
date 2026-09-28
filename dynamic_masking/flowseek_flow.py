"""
flowseek_flow.py
================
Thin wrapper around the vendored FlowSeek (ICCV 2025) optical-flow model at
``thirdparty/flowseek`` so the rest of the pipeline can ask for dense flow
between two RGB frames without caring about FlowSeek's sys.path / cwd quirks.

Facts about the upstream code this wrapper works around (read from source):
  * ``core/flowseek.py`` does ``from depth_anything_v2.dpt import ...`` -- the
    package is vendored at ``core/depth_anything_v2``, so ``core/`` must be on
    ``sys.path`` (the authors' ``demo.py`` does ``sys.path.append('core')``).
  * ``FlowSeek.__init__`` loads ``weights/depth_anything_v2_<da_size>.pth`` via
    a *relative* path -> we temporarily chdir into the FlowSeek root while
    constructing the model.
  * Inputs are ``[N, 3, H, W]`` float tensors in **0..255 RGB** (see
    ``core/datasets.py``); normalisation, the 518x518 DA-V2 resize and the /8
    padding all happen inside ``forward``.  ``.cuda()`` is hard-coded inside
    the model, so device must be the default CUDA device.
  * Output dict ``{'final': [N,2,H,W], 'flow': [...], 'info': [...]}``; flow is
    (dx, dy) in pixels, image-1 -> image-2.

Usage:
    flow_fn = FlowSeekFlow(size="M")           # T | S | M | L (S/L reuse T/M weights)
    flow = flow_fn(rgb1_uint8_hwc, rgb2_uint8_hwc)   # -> float32 [H, W, 2]
"""
from __future__ import annotations

import contextlib
import json
import os
import sys
from argparse import Namespace
from typing import Optional

import numpy as np
import torch
import torch.nn.functional as F

HERE = os.path.dirname(os.path.abspath(__file__))
FLOWSEEK_ROOT = os.path.join(HERE, "thirdparty", "flowseek")

# Fully-trained checkpoints published by the authors (scripts/get_weights.sh);
# the "TSKH" ones saw Tartan + Chairs + Things + Sintel/KITTI/HD1K, i.e. the most data.
DEFAULT_CKPT = {
    "T": "flowseek_T_TartanCT_TSKH.pth",   # only T is present in this tree; M/L unavailable (acm-only, mode 600)
    "S": "flowseek_T_TartanCT_TSKH.pth",
    "M": "flowseek_M_TartanCT_TSKH.pth",
    "L": "flowseek_M_TartanCT_TSKH.pth",
}


@contextlib.contextmanager
def _pushd(path: str):
    prev = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(prev)


def _ensure_importable(root: str = FLOWSEEK_ROOT) -> None:
    core = os.path.join(root, "core")
    for p in (core, root):
        if p not in sys.path:
            sys.path.insert(0, p)


class FlowSeekFlow:
    def __init__(self, size: str = "M", ckpt: Optional[str] = None,
                 iters: Optional[int] = None, root: str = FLOWSEEK_ROOT,
                 device: str = "cuda", scale: int = 0):
        """
        size   : FlowSeek variant, one of T/S/M/L (picks config/eval/flowseek-<size>.json).
        ckpt   : checkpoint path; default = weights/<DEFAULT_CKPT[size]> under root.
        iters  : override refinement iterations (config default: T/M=4 ... see json).
        scale  : run at 2**scale input resolution (demo.py's --scale); 0 = native.
        """
        self.root = root
        self.size = size.upper()
        self.device = device
        self.scale = int(scale)
        _ensure_importable(root)
        from flowseek import FlowSeek  # noqa: E402  (thirdparty/flowseek/core/flowseek.py)

        cfg_path = os.path.join(root, "config", "eval", f"flowseek-{self.size}.json")
        with open(cfg_path) as f:
            cfg = json.load(f)
        self.args = Namespace(**cfg)
        if iters is not None:
            self.args.iters = int(iters)
        self.iters = self.args.iters

        ckpt = ckpt or os.path.join(root, "weights", DEFAULT_CKPT[self.size])
        if not os.path.isfile(ckpt):
            raise FileNotFoundError(f"FlowSeek checkpoint missing: {ckpt} "
                                    f"(see thirdparty/flowseek/scripts/get_weights.sh)")
        self.ckpt = ckpt

        with _pushd(root):   # DA-V2 weights are loaded via a cwd-relative path
            self.model = FlowSeek(self.args)
        try:
            sd = torch.load(ckpt, map_location="cpu")
        except Exception:
            sd = torch.load(ckpt, map_location="cpu", weights_only=False)
        if isinstance(sd, dict) and "model" in sd and isinstance(sd["model"], dict):
            sd = sd["model"]
        sd = {k[7:] if k.startswith("module.") else k: v for k, v in sd.items()}
        missing, unexpected = self.model.load_state_dict(sd, strict=False)
        # dav2.* keys are loaded separately in __init__; anything else missing is a red flag.
        self.missing_keys = [k for k in missing if not k.startswith("dav2.")]
        self.unexpected_keys = list(unexpected)
        self.model = self.model.to(device).eval()

    @staticmethod
    def _to_tensor(img: np.ndarray, device: str) -> torch.Tensor:
        if img.ndim != 3 or img.shape[2] != 3:
            raise ValueError(f"expected [H, W, 3] RGB uint8, got {img.shape}")
        t = torch.from_numpy(np.ascontiguousarray(img)).permute(2, 0, 1)[None].float()
        return t.to(device)

    @torch.no_grad()
    def flow_tensor(self, img1: torch.Tensor, img2: torch.Tensor) -> torch.Tensor:
        """img1/img2: [N,3,H,W] float 0..255 on device. Returns [N,2,H,W] flow (px)."""
        if self.scale != 0:
            f = 2.0 ** self.scale
            img1 = F.interpolate(img1, scale_factor=f, mode="bilinear", align_corners=False)
            img2 = F.interpolate(img2, scale_factor=f, mode="bilinear", align_corners=False)
        out = self.model(img1, img2, iters=self.iters, test_mode=True)
        flow = out["final"]
        if self.scale != 0:
            f = 0.5 ** self.scale
            flow = F.interpolate(flow, scale_factor=f, mode="bilinear", align_corners=False) * f
        return flow

    @torch.no_grad()
    def __call__(self, rgb1: np.ndarray, rgb2: np.ndarray) -> np.ndarray:
        """rgb1, rgb2: [H, W, 3] uint8 RGB.  Returns float32 [H, W, 2] flow (dx, dy) px, 1 -> 2."""
        t1 = self._to_tensor(rgb1, self.device)
        t2 = self._to_tensor(rgb2, self.device)
        flow = self.flow_tensor(t1, t2)[0]                 # [2, H, W]
        return flow.permute(1, 2, 0).float().cpu().numpy()


def flow_to_rgb(flow: np.ndarray, clip: Optional[float] = None) -> np.ndarray:
    """Middlebury colour wheel visualisation (uses FlowSeek's own utils)."""
    _ensure_importable()
    from utils.flow_viz import flow_to_image  # thirdparty/flowseek/core/utils/flow_viz.py
    return flow_to_image(flow.astype(np.float32), clip_flow=clip, convert_to_bgr=False)
