"""Simulation : trajectoire du drone et mesures d'angle bruitées."""
import numpy as np


def make_capteur(x, y, sigma_deg=2.0):
    """Capteur d'angle en (x, y) ; bruit gaussien d'écart-type sigma_deg (degrés)."""
    return {"pos": np.array([x, y], dtype=float), "R": np.array([[np.radians(sigma_deg) ** 2]])}


def generer_trajectoire(dt, duree, t_virage=20.0, v_avant=(5.0, 3.0), v_apres=(-2.0, 6.0)):
    """Drone à vitesse constante, avec un virage brutal à t_virage."""
    temps = np.arange(0, duree, dt)
    x, y = np.zeros(len(temps)), np.zeros(len(temps))
    vx, vy = v_avant
    for i in range(1, len(temps)):
        if temps[i] >= t_virage:
            vx, vy = v_apres
        x[i], y[i] = x[i - 1] + vx * dt, y[i - 1] + vy * dt
    return temps, x, y


def simuler_mesures(temps, x, y, capteurs, seed=42):
    """Mesures d'angle bruitées, forme (n_capteurs, n_temps). Une graine = une réalisation du bruit."""
    rng = np.random.default_rng(seed)
    mesures = []
    for c in capteurs:
        xs, ys = c["pos"]
        vrai = np.arctan2(y - ys, x - xs)
        bruit = rng.normal(0, np.sqrt(c["R"][0, 0]), len(temps))
        mesures.append((vrai + bruit + np.pi) % (2 * np.pi) - np.pi)
    return np.array(mesures)