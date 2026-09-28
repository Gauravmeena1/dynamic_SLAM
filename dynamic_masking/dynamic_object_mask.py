"""
dynamic_object_mask.py
======================
Semantic half of the dynamic-object detector: a single-frame YOLO instance
segmenter (yolov9e-seg) restricted to categories that are *movable* in an
indoor robot map (people and the things people carry).

This is the standalone form of the ``DynamicObjectDetector`` that lives inline
in ``src/ma_slam/solver_gau.py`` -- same class list, same post-processing, same
``get_mask(rgb, clean=True) -> bool[H, W]`` contract, so the two stay
interchangeable.  It needs no pose, depth or optical flow.

    det = DynamicObjectDetector("yolov9e-seg.pt", device="cuda")
    mask = det.get_mask(rgb_uint8_hwc)        # True = dynamic (semantic)
"""
from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np
import torch
import torch.nn.functional as F

try:
    from scipy import ndimage as _ndi
    _HAVE_SCIPY = True
except Exception:  # pragma: no cover
    _HAVE_SCIPY = False

# COCO class ids treated as dynamic / movable.
DYNAMIC_CLASSES: List[int] = [
    0,   # person
    24,  # backpack
    25,  # umbrella
    26,  # handbag
    28,  # suitcase
    39,  # bottle
    63,  # laptop
    65,  # remote
    67,  # cell phone
    73,  # book
]
DYNAMIC_CLASS_NAMES: Dict[int, str] = {
    0: "person", 24: "backpack", 25: "umbrella", 26: "handbag", 28: "suitcase",
    39: "bottle", 63: "laptop", 65: "remote", 67: "cell phone", 73: "book",
}


def _dilate(mask: torch.Tensor, k: int) -> torch.Tensor:
    if k <= 1:
        return mask
    m = mask.float()[None, None]
    return (F.max_pool2d(m, k, stride=1, padding=k // 2) > 0.5)[0, 0]


def _erode(mask: torch.Tensor, k: int) -> torch.Tensor:
    if k <= 1:
        return mask
    m = mask.float()[None, None]
    return (-F.max_pool2d(-m, k, stride=1, padding=k // 2) > 0.5)[0, 0]


def clean_mask(mask: torch.Tensor,
               open_k: int = 3,
               close_k: int = 7,
               min_area: int = 150,
               dilate_k: int = 5) -> torch.Tensor:
    """Morphological open -> close -> drop small blobs -> dilate (edge safety margin)."""
    m = _dilate(_erode(mask, open_k), open_k)
    m = _erode(_dilate(m, close_k), close_k)
    if min_area > 0 and _HAVE_SCIPY and m.any():
        lab, n = _ndi.label(m.detach().cpu().numpy())
        if n > 0:
            counts = np.bincount(lab.ravel())
            counts[0] = 0
            keep = np.isin(lab, np.flatnonzero(counts >= min_area))
            m = torch.from_numpy(keep).to(m.device)
    return _dilate(m, dilate_k)


class DynamicObjectDetector:
    """YOLO-seg wrapper producing a boolean per-pixel mask of known movable classes."""

    def __init__(self, checkpoint_path: str, device: str = "cuda",
                 classes: Optional[List[int]] = None, conf: float = 0.25,
                 class_conf: Optional[Dict[int, float]] = None):
        """class_conf (2026-09-03): optional per-class confidence overrides, e.g. {0: 0.15} keeps
        motion-blurred people (lab frame 48 scored 0.17 < 0.25) while every other class stays at
        `conf`.  YOLO runs at the lowest threshold and the rest is filtered here."""
        from ultralytics import YOLO  # imported lazily: heavy, optional
        self.model = YOLO(checkpoint_path)
        self.device = device
        self.classes = list(classes) if classes is not None else list(DYNAMIC_CLASSES)
        self.conf = conf
        self.class_conf = dict(class_conf) if class_conf else {}
        self.last_detections: List[Dict] = []   # [{cls, name, conf, area}] from the last call

    @torch.no_grad()
    def get_instances(self, rgb_frame: np.ndarray) -> List[Dict]:
        """Per-instance output (2026-09-04, for the moving-or-carried object policy): one dict per detection
        {cls, name, conf, mask (bool [H, W], raw, not cleaned)}.  Same thresholds as get_mask; also refreshes
        last_detections so callers that only read that keep working."""
        H, W = rgb_frame.shape[:2]
        run_conf = min([self.conf] + list(self.class_conf.values()))
        results = self.model.predict(source=rgb_frame, classes=self.classes, conf=run_conf,
                                     save=False, verbose=False, device=self.device)
        out: List[Dict] = []
        for r in results:
            if r.masks is None:
                continue
            cls_ids = r.boxes.cls.tolist() if r.boxes is not None else [None] * len(r.masks.data)
            confs = r.boxes.conf.tolist() if r.boxes is not None else [None] * len(r.masks.data)
            for m, c, p in zip(r.masks.data, cls_ids, confs):
                if self.class_conf and c is not None and p is not None \
                        and p < self.class_conf.get(int(c), self.conf):
                    continue
                m_resized = F.interpolate(m[None, None].float(), size=(H, W), mode="nearest")[0, 0].bool().cpu().numpy()
                out.append({"cls": None if c is None else int(c),
                            "name": DYNAMIC_CLASS_NAMES.get(int(c), str(c)) if c is not None else "?",
                            "conf": None if p is None else float(p), "area": int(m_resized.sum()), "mask": m_resized})
        self.last_detections = [{k: v for k, v in d.items() if k != "mask"} for d in out]
        return out

    @torch.no_grad()
    def get_mask(self, rgb_frame: np.ndarray, clean: bool = True) -> np.ndarray:
        """rgb_frame: [H, W, 3] uint8 RGB.  Returns bool [H, W], True = dynamic."""
        H, W = rgb_frame.shape[:2]
        run_conf = min([self.conf] + list(self.class_conf.values()))
        results = self.model.predict(
            source=rgb_frame, classes=self.classes, conf=run_conf,
            save=False, verbose=False, device=self.device,
        )
        mask = torch.zeros((H, W), dtype=torch.bool)
        dets: List[Dict] = []
        for r in results:
            if r.masks is None:
                continue
            cls_ids = r.boxes.cls.tolist() if r.boxes is not None else [None] * len(r.masks.data)
            confs = r.boxes.conf.tolist() if r.boxes is not None else [None] * len(r.masks.data)
            for m, c, p in zip(r.masks.data, cls_ids, confs):
                if self.class_conf and c is not None and p is not None \
                        and p < self.class_conf.get(int(c), self.conf):
                    continue                                   # below this class's own threshold
                m_resized = F.interpolate(
                    m[None, None].float(), size=(H, W), mode="nearest"
                )[0, 0].bool().cpu()
                mask |= m_resized
                dets.append({"cls": None if c is None else int(c),
                             "name": DYNAMIC_CLASS_NAMES.get(int(c), str(c)) if c is not None else "?",
                             "conf": None if p is None else float(p),
                             "area": int(m_resized.sum())})
        self.last_detections = dets
        if clean and mask.any():
            mask = clean_mask(mask)
        return mask.numpy()
