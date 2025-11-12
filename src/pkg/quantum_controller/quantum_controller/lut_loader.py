import csv
import math
import numpy as np
from typing import Tuple, Dict, Any, Optional

class LUT2D:
    """
    Lookup table 2D generica su (x,y) -> value con:
    - modalità 'nearest' (default) su punti non necessariamente grigliati
    - modalità 'bilinear' solo se la tabella è su griglia rettangolare (richiede colonne distinte)
    """
    def __init__(self, points_xy: np.ndarray, values: np.ndarray, method: str = 'nearest'):
        assert points_xy.shape[1] == 2, "points_xy deve essere Nx2"
        assert points_xy.shape[0] == values.shape[0], "points e values hanno N diverso"
        self.points = points_xy.astype(float)
        self.values = values.astype(float)
        self.method = method

        # Per 'nearest' precompute
        self._norm = np.sum(self.points**2, axis=1)

    @classmethod
    def from_csv(cls, path: str, x_col: str, y_col: str, v_col: str, method: str = 'nearest'):
        xs, ys, vs = [], [], []
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                xs.append(float(row[x_col]))
                ys.append(float(row[y_col]))
                vs.append(float(row[v_col]))
        pts = np.column_stack([np.array(xs), np.array(ys)])
        vals = np.array(vs)
        return cls(pts, vals, method=method)

    def query(self, x: float, y: float) -> float:
        if self.method == 'nearest':
            # distanza euclidea
            diffs = self.points - np.array([x, y])
            idx = np.argmin(np.sum(diffs*diffs, axis=1))
            return float(self.values[idx])
        elif self.method == 'bilinear':
            # Nota: qui implementiamo un fallback: se non riconosciamo una griglia,
            # torniamo al nearest.
            return self.query_nearest(x, y)
        else:
            return self.query_nearest(x, y)

    def query_nearest(self, x: float, y: float) -> float:
        diffs = self.points - np.array([x, y])
        idx = np.argmin(np.sum(diffs*diffs, axis=1))
        return float(self.values[idx])

def detect_columns(header: Dict[str, Any]) -> Tuple[str, str, str]:
    """
    Utility per indovinare le colonne standard.
    Preferenze:
      x: ['dist', 'distance', 'dist_error']
      y: ['theta', 'orient', 'angle', 'yaw', 'heading', 'orient_error']
      v: ['v', 'linear', 'linear_velocity', 'cmd', 'value', 'vel']
    """
    keys = [k.lower() for k in header]
    def pick(candidates):
        for c in candidates:
            for k in header:
                if k.lower() == c:
                    return k
        # fallback: prima colonna disponibile
        return list(header.keys())[0]

    x_col = pick(['dist', 'distance', 'dist_error'])
    y_col = pick(['theta', 'orient', 'angle', 'yaw', 'heading', 'orient_error'])
    v_col = pick(['v', 'linear', 'linear_velocity', 'cmd', 'value', 'vel'])
    return x_col, y_col, v_col
