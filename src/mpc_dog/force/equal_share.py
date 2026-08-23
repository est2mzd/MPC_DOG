"""Stance GRF: split weight equally among stance feet. No horizon."""

from __future__ import annotations

import numpy as np


def equal_share(mass_kg: float, n_stance: int | None = 4) -> np.ndarray:
    """World-frame GRF (4, 3). +z supports the robot. Horizontal 0."""
    ns = 4 if n_stance is None else int(n_stance)
    if ns < 1:
        raise ValueError("need at least one stance foot")
    g = 9.81
    fz = mass_kg * g / ns
    grf = np.zeros((4, 3), dtype=np.float64)
    grf[:, 2] = fz
    return grf
