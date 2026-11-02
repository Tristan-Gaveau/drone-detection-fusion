"""Analyses complémentaires citées dans le README (coupure et profondeur initiale)."""
import numpy as np
from ekf import run_ekf, init_depuis_mesures
from simulate import generer_trajectoire, simuler_mesures
from metrics import erreurs_radiale_laterale, erreur_position
import experiments as ex

temps, x, y = generer_trajectoire(ex.DT, ex.DUREE)
fenetre = (temps >= 20) & (temps < 30)

print("=== Coupure : erreur de la fusion sur 20-30 s, selon la visée du capteur 2")
for nom, cfg in ex.CONFIGS.items():
    for libelle, coupure in [("sans coupure", None), ("capteur 1 coupé 15-25 s", (0, 15, 25))]:
        X = ex.monte_carlo(cfg, temps, x, y, coupure=coupure)
        rad, lat = erreurs_radiale_laterale(X, x, y, cfg[1])
        print(f"placement {nom} | {libelle:<24} | radiale {np.sqrt(np.mean(rad[:, fenetre]**2)):5.1f} m"
              f" | latérale {np.sqrt(np.mean(lat[:, fenetre]**2)):5.1f} m")

print("\n=== Capteur 1 seul (placement B) : départ à la distance r0 sur la ligne de visée")
cfg = ex.CONFIGS["B"]
print("distance vraie à t=0 :", round(float(np.hypot(x[0] - cfg[0]["pos"][0], y[0] - cfg[0]["pos"][1]))), "m")
for r0 in [30, 60, 100, 150, 200]:
    Xs = np.zeros((ex.N_RUNS, len(temps), 4))
    for s in range(ex.N_RUNS):
        m = simuler_mesures(temps, x, y, cfg, seed=s)
        m[1, :] = np.nan
        X0, P0 = init_depuis_mesures(m[:, 0], cfg, r0=r0)
        Xs[s], _ = run_ekf(m, cfg, ex.VAR_A, ex.DT, X0, P0)
    E = erreur_position(Xs, x, y)
    rad, lat = erreurs_radiale_laterale(Xs, x, y, cfg[0])
    print(f"r0={r0:>3} m | RMSE {np.sqrt(np.mean(E**2)):5.1f} | radiale {np.sqrt(np.mean(rad**2)):5.1f}"
          f" | latérale {np.sqrt(np.mean(lat**2)):4.1f} | radiale moyenne signée {rad.mean():6.1f}")