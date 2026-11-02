"""Valide la jacobienne analytique H_angle par différences finies centrées.

Lancer :  pytest -v
Les points sont choisis pour que les DEUX dérivées soient testées, y compris à des endroits
où l'une d'elles est nulle, et près de la coupure de l'angle en ±pi.
"""
import numpy as np
import pytest
from ekf import H_angle, h_angle, wrap_angle

# (x, y, xs, ys)
POINTS = [
    (60.0, 40.0, 0.0, 0.0),
    (-30.0, 80.0, 0.0, 0.0),
    (80.0, -20.0, 15.0, 35.0),
    (100.0, 0.0, 0.0, 0.0),     # visée plein est : dérivée en x nulle
    (-100.0, 1e-3, 0.0, 0.0),   # près de la coupure à ±pi
]
EPS = 1e-6


def derivee_numerique(x, y, xs, ys, axe):
    dx, dy = (EPS, 0.0) if axe == 0 else (0.0, EPS)
    ecart = h_angle(x + dx, y + dy, xs, ys) - h_angle(x - dx, y - dy, xs, ys)
    return wrap_angle(ecart) / (2 * EPS)


@pytest.mark.parametrize("x,y,xs,ys", POINTS)
@pytest.mark.parametrize("axe", [0, 1])
def test_jacobienne_vs_numerique(x, y, xs, ys, axe):
    analytique = H_angle(x, y, xs, ys)[0, axe]
    numerique = derivee_numerique(x, y, xs, ys, axe)
    print(f"point=({x},{y}) capteur=({xs},{ys}) axe={'xy'[axe]} analytique={analytique:.8f} numerique={numerique:.8f}")
    assert analytique == pytest.approx(numerique, abs=1e-7)


def test_vitesses_non_observees():
    assert np.all(H_angle(60.0, 40.0, 0.0, 0.0)[0, 2:] == 0.0)


def test_norme_du_gradient_vaut_inverse_distance():
    x, y, xs, ys = 60.0, 40.0, 10.0, -5.0
    r = np.hypot(x - xs, y - ys)
    assert np.linalg.norm(H_angle(x, y, xs, ys)[0, :2]) == pytest.approx(1 / r)