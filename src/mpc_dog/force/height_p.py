"""Height P on total Fz, then EqualShare among stance feet. No horizon."""

from __future__ import annotations

import numpy as np


def height_p_share(
    mass_kg: float,
    z: float,
    z_ref: float,
    vz: float,
    kp: float,
    kd: float,
    contact: np.ndarray | None = None,
) -> np.ndarray:
    """World-frame GRF (4, 3). ``sum Fz = mg + kp(z_ref - z) - kd vz``."""
    c = np.ones(4, dtype=np.float64) if contact is None else np.asarray(contact, dtype=np.float64).reshape(4)
    ns = int(np.count_nonzero(c > 0.5))
    if ns < 1:
        raise ValueError("need at least one stance foot")
    g = 9.81
    fz_total = mass_kg * g + kp * (z_ref - z) - kd * vz
    fz_total = max(fz_total, 0.0)
    grf = np.zeros((4, 3), dtype=np.float64)
    grf[c > 0.5, 2] = fz_total / ns
    return grf
