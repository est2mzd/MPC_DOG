"""Shared GIF/MP4 export settings for workshop demo captures."""
from __future__ import annotations

from pathlib import Path

import numpy as np

# Workshop requirement: GIF playback >= 10 s (GIF stores duration in 1/100 s units)
GIF_TARGET_DURATION_S = 10.0
GIF_FRAME_MS = 100  # centisecond-quantized → exact 10 ms units
GIF_N_FRAMES = int(GIF_TARGET_DURATION_S * 1000 / GIF_FRAME_MS)  # 100 frames = 10.0 s
MP4_FPS = 10


def pick_frames(saved: list[Path], n: int = GIF_N_FRAMES) -> list[Path]:
    """Evenly sample frames across the full run (not just the opening seconds)."""
    if len(saved) <= n:
        return saved
    idx = np.linspace(0, len(saved) - 1, n, dtype=int)
    return [saved[i] for i in idx]


def save_gif_and_mp4(
    saved: list[Path],
    gif_path: Path,
    *,
    n_gif: int = GIF_N_FRAMES,
    frame_ms: int = GIF_FRAME_MS,
) -> tuple[list[Path], float]:
    """Write GIF + MP4; return (gif_frame_paths, playback_duration_s)."""
    from PIL import Image

    gif_frames = pick_frames(saved, n=n_gif)
    frames = [Image.open(p).convert("RGB") for p in gif_frames]
    frames[0].save(
        gif_path,
        save_all=True,
        append_images=frames[1:],
        duration=frame_ms,
        loop=0,
        optimize=False,
    )
    playback_s = len(gif_frames) * frame_ms / 1000.0

    mp4_path = gif_path.with_suffix(".mp4")
    try:
        import imageio.v3 as iio

        stack = np.stack([np.array(Image.open(p).convert("RGB")) for p in gif_frames])
        iio.imwrite(mp4_path, stack, fps=MP4_FPS, codec="libx264")
    except Exception as exc:
        print(f"mp4 skip ({gif_path.name}): {exc}")

    return gif_frames, playback_s
