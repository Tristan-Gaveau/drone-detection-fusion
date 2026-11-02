"""Étude des limites de l'EKF : incertitude annoncée, erreur réelle et NEES."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from ekf import run_ekf
from simulate import generer_trajectoire, simuler_mesures
import experiments as ex

import sys
VAR_A = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0

NOM = "B"                      # placement étudié : "A" ou "B"
temps, x, y = generer_trajectoire(ex.DT, ex.DUREE)
cfg = ex.CONFIGS[NOM]

n_t = len(temps)
sigma_annonce = np.zeros((ex.N_RUNS, n_t))
erreur_reelle = np.zeros((ex.N_RUNS, n_t))
nees = np.zeros((ex.N_RUNS, n_t))

for s in range(ex.N_RUNS):
    m = simuler_mesures(temps, x, y, cfg, seed=s)
    X, P = run_ekf(m, cfg, VAR_A, ex.DT)
    for i in range(n_t):
        e = np.array([X[i, 0] - x[i], X[i, 1] - y[i]])
        P_pos = P[i, 0:2, 0:2]
        sigma_annonce[s, i] = np.sqrt(np.trace(P_pos))
        erreur_reelle[s, i] = np.linalg.norm(e)
        nees[s, i] = e @ np.linalg.inv(P_pos) @ e

rmse = np.sqrt(np.mean(erreur_reelle**2, axis=0))
sigma_moy = np.sqrt(np.mean(sigma_annonce**2, axis=0))
nees_moy = nees.mean(axis=0)

# Figure 1 : erreur réelle vs incertitude annoncée
fig, ax = plt.subplots(figsize=(10, 4.5))
ax.plot(temps, rmse, label="Erreur réelle (RMSE)")
ax.plot(temps, sigma_moy, label="Incertitude annoncée par le filtre")
ax.set_xlabel("Temps (s)"); ax.set_ylabel("Position (m)")
ax.set_title(f"Placement {NOM} : erreur réelle vs incertitude annoncée")
ax.grid(True); ax.legend()
fig.tight_layout()
fig.savefig(ex.SORTIE / f"etude_sigma_vs_rmse_{NOM}_va{VAR_A}.png", dpi=130)
plt.close(fig)

# Figure 2 : NEES moyenne (référence = 2 pour un filtre cohérent)
fig, ax = plt.subplots(figsize=(10, 4.5))
ax.plot(temps, nees_moy, label="NEES moyenne")
ax.axhline(2, color="k", ls="--", label="Valeur attendue (filtre cohérent) = 2")
ax.set_xlabel("Temps (s)"); ax.set_ylabel("NEES")
ax.set_title(f"Placement {NOM} : cohérence du filtre (NEES)")
ax.grid(True); ax.legend()
fig.tight_layout()
fig.savefig(ex.SORTIE / f"etude_nees_{NOM}_va{VAR_A}.png", dpi=130)
plt.close(fig)

i22 = int(22 / ex.DT)
print(f"NEES moyenne à t = 22 s : {nees_moy[i22]:.2f}")