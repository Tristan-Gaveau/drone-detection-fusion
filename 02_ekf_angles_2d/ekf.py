"""Filtre de Kalman étendu (EKF) : suivi 2D d'un drone à partir de mesures d'angle.

État      : X = [x, y, vx, vy]^T  (modèle à vitesse constante)
Mesure    : angle de relèvement  theta = atan2(y - ys, x - xs)  (non linéaire en x, y)
Jacobienne: H = [-(y-ys)/r^2, (x-xs)/r^2, 0, 0],  r^2 = (x-xs)^2 + (y-ys)^2
"""
import numpy as np

# Estimation initiale arbitraire, indépendante de la vérité terrain (sert de repli et de test de sensibilité).
X0_DEFAUT = np.array([[50.0], [50.0], [0.0], [0.0]])
P0_DEFAUT = np.diag([50.0**2, 50.0**2, 10.0**2, 10.0**2])


def wrap_angle(a):
    """Ramène un angle dans [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def get_F(dt):
    return np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def get_Q(var_a, dt):
    """Bruit de processus : accélération aléatoire de variance var_a (m/s^2)^2."""
    a, b, c = dt**4 / 4, dt**3 / 2, dt**2
    return var_a * np.array([[a, 0, b, 0], [0, a, 0, b], [b, 0, c, 0], [0, b, 0, c]])


def h_angle(x, y, xs, ys):
    """Fonction de mesure : angle de relèvement du drone vu du capteur."""
    return np.arctan2(y - ys, x - xs)


def H_angle(x, y, xs, ys):
    """Jacobienne de h_angle par rapport à l'état [x, y, vx, vy]."""
    dx, dy = x - xs, y - ys
    r2 = max(dx**2 + dy**2, 1e-6)
    return np.array([[-dy / r2, dx / r2, 0.0, 0.0]])


def ekf_predict(X, P, F, Q):
    return F @ X, F @ P @ F.T + Q


def ekf_update(X, P, z, xs, ys, R):
    """Correction avec une mesure d'angle z (rad). Forme de Joseph pour P (reste symétrique)."""
    x, y = X[0, 0], X[1, 0]
    innovation = wrap_angle(z - h_angle(x, y, xs, ys))
    H = H_angle(x, y, xs, ys)
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    X_new = X + K * innovation
    I_KH = np.eye(4) - K @ H
    P_new = I_KH @ P @ I_KH.T + K @ R @ K.T
    return X_new, P_new


def init_depuis_mesures(z0, capteurs, r0=100.0, sigma_pos=60.0, sigma_vit=10.0):
    """État initial construit à partir des premières mesures d'angle (jamais de la vérité terrain).

    - 2 capteurs ou plus, lignes de visée assez différentes : intersection des deux demi-droites (triangulation).
    - 1 seul capteur (ou lignes quasi parallèles) : point à la distance supposée r0 sur la ligne de visée.
    - aucune mesure : estimation arbitraire X0_DEFAUT.
    La vitesse initiale est nulle, d'incertitude sigma_vit.
    """
    P0 = np.diag([sigma_pos**2, sigma_pos**2, sigma_vit**2, sigma_vit**2])
    valides = [k for k in range(len(capteurs)) if not np.isnan(z0[k])]
    if len(valides) >= 2:
        i, j = valides[:2]
        d_i, d_j = np.array([np.cos(z0[i]), np.sin(z0[i])]), np.array([np.cos(z0[j]), np.sin(z0[j])])
        A = np.column_stack([d_i, -d_j])
        if abs(np.linalg.det(A)) > 0.2:  # |sin(angle entre les visées)| > 0.2, soit environ 11.5 degrés
            t = np.linalg.solve(A, capteurs[j]["pos"] - capteurs[i]["pos"])
            if t[0] > 0 and t[1] > 0:  # le drone est devant les deux capteurs
                p = capteurs[i]["pos"] + t[0] * d_i
                return np.array([[p[0]], [p[1]], [0.0], [0.0]]), P0
    if valides:
        k = valides[0]
        p = capteurs[k]["pos"] + r0 * np.array([np.cos(z0[k]), np.sin(z0[k])])
        return np.array([[p[0]], [p[1]], [0.0], [0.0]]), P0
    return X0_DEFAUT.copy(), P0_DEFAUT.copy()


def run_ekf(mesures, capteurs, var_a, dt, X0=None, P0=None):
    """Exécute l'EKF. mesures : tableau (n_capteurs, n_temps) ; un np.nan = capteur absent.

    Les capteurs sont traités l'un après l'autre à chaque instant (mise à jour séquentielle).
    Si X0 est None, l'état initial est construit à partir des premières mesures (init_depuis_mesures).
    Retourne (X_hist de forme (n_temps, 4), P_hist de forme (n_temps, 4, 4)).
    """
    if X0 is None:
        X, P = init_depuis_mesures(mesures[:, 0], capteurs)
    else:
        X = X0.copy()
        P = (P0_DEFAUT if P0 is None else P0).copy()
    F, Q = get_F(dt), get_Q(var_a, dt)
    n_t = mesures.shape[1]
    X_hist, P_hist = np.zeros((n_t, 4)), np.zeros((n_t, 4, 4))
    for i in range(n_t):
        X, P = ekf_predict(X, P, F, Q)
        for k, capteur in enumerate(capteurs):
            z = mesures[k, i]
            if not np.isnan(z):
                xs, ys = capteur["pos"]
                X, P = ekf_update(X, P, z, xs, ys, capteur["R"])
        X_hist[i], P_hist[i] = X.ravel(), P
    return X_hist, P_hist