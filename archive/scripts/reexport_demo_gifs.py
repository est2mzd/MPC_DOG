#!/usr/bin/env python3
"""Re-export demo GIFs/MP4s from existing frame PNGs (no sim re-run)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
sys.path.insert(0, str(ROOT / "scripts"))

from demo_capture_common import GIF_FRAME_MS, save_gif_and_mp4

TAGS = [
    "s01_flat",
    "s02_tune",
    "s03_boxes",
    "s03_perlin",
    "s04_flat",
    "s04_uphill",
    "s04_downhill",
]


def main() -> int:
    for tag in TAGS:
        frame_dir = ASSETS / f"frames_{tag}"
        if not frame_dir.is_dir():
            print(f"skip {tag}: no {frame_dir}")
            continue
        saved = sorted(frame_dir.glob("*.png"))
        if not saved:
            print(f"skip {tag}: empty frames")
            continue
        gif_path = ASSETS / f"demo_{tag}.gif"
        gif_frames, playback_s = save_gif_and_mp4(saved, gif_path)
        meta_path = ASSETS / f"demo_{tag}.meta.json"
        meta = {}
        if meta_path.is_file():
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        meta["n_gif_frames"] = len(gif_frames)
        meta["gif_playback_s"] = round(playback_s, 2)
        meta["gif_frame_ms"] = GIF_FRAME_MS
        meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
        print(f"{tag}: {len(saved)} png -> {playback_s:.1f}s GIF")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
