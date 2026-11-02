"""Métriques : erreurs, RMSE par fenêtre, géométrie des capteurs."""
import numpy as np
from ekf import wrap_angle


def erreur_position(X_hist, x, y):
    """Erreur de position (m) à chaque instant. X_hist : (..., n_temps, 4)."""
    return np.hypot(X_hist[..., 0] - x, X_hist[..., 1] - y)


def rmse_fenetres(E, temps, fenetres):
    """RMSE de E (n_runs, n_temps) sur chaque fenêtre (t0, t1), puis sur toute la durée."""
    out = []
    for t0, t1 in fenetres:
        masque = (temps >= t0) & (temps < t1)
        out.append(np.sqrt(np.mean(E[:, masque] ** 2)))
    out.append(np.sqrt(np.mean(E**2)))
    return out


def angle_lignes_de_visee(x, y, cap_a, cap_b):
    """Angle (deg) entre les deux lignes de visée. 90 = idéal ; proche de 0 = lignes confondues."""
    a = np.arctan2(y - cap_a["pos"][1], x - cap_a["pos"][0])
    b = np.arctan2(y - cap_b["pos"][1], x - cap_b["pos"][0])
    d = np.degrees(np.abs(wrap_angle(a - b)))
    return np.minimum(d, 180 - d)


def distance_moyenne(x, y, capteurs):
    """Distance capteur-drone moyennée sur le temps et sur les capteurs (m)."""
    return float(np.mean([np.hypot(x - c["pos"][0], y - c["pos"][1]) for c in capteurs]))


def erreurs_radiale_laterale(X_hist, x, y, capteur):
    """Décompose l'erreur de position selon la ligne de visée d'un capteur.

    Radiale : le long de la visée (non observable par un capteur d'angle seul).
    Latérale : perpendiculairement à la visée.
    """
    ux, uy = x - capteur["pos"][0], y - capteur["pos"][1]
    norme = np.hypot(ux, uy)
    ux, uy = ux / norme, uy / norme
    ex, ey = X_hist[..., 0] - x, X_hist[..., 1] - y
    return ex * ux + ey * uy, -ex * uy + ey * ux