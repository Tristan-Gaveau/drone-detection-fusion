"""Tests de cohérence du filtre complet."""
import numpy as np
from ekf import run_ekf
from simulate import make_capteur, generer_trajectoire, simuler_mesures
from metrics import erreur_position


def test_mesures_quasi_parfaites_donnent_une_erreur_faible():
    temps, x, y = generer_trajectoire(0.5, 40)
    cfg = [make_capteur(20, 100, 0.01), make_capteur(150, -20, 0.01)]
    X, _ = run_ekf(simuler_mesures(temps, x, y, cfg, seed=0), cfg, 1.0, 0.5)
    assert erreur_position(X, x, y)[temps >= 10].max() < 1.5


def test_sans_mesure_la_covariance_augmente():
    temps, x, y = generer_trajectoire(0.5, 10)
    cfg = [make_capteur(20, 100), make_capteur(150, -20)]
    m = np.full((2, len(temps)), np.nan)
    _, P = run_ekf(m, cfg, 1.0, 0.5)
    assert np.trace(P[-1]) > np.trace(P[0])