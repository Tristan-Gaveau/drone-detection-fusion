"""Filtre de Kalman 1D (état = position, vitesse) : suivi d'un drone avec capteur brouillé.

Scénario : le drone avance à 10 m/s, passe à 20 m/s à t = 20 s ; à partir de t = 30 s
le capteur de position est brouillé (écart-type 5 m -> 50 m).
On compare deux filtres : l'un ignore le brouillage, l'autre en est prévenu (R augmenté).
Résultats moyennés sur 100 simulations (graine fixée).
"""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DUREE, DT = 60, 1
TEMPS = np.arange(DUREE)
SIGMA_NORMAL, FACTEUR_BROUILLAGE, T_BROUILLAGE, T_MANOEUVRE = 5.0, 10.0, 30, 20
VAR_A, N_SIM, GRAINE = 2.0, 100, 42
SORTIE = Path(__file__).parent / "results"

F = np.array([[1, DT], [0, 1]])
H = np.array([[1, 0]])
Q = VAR_A * np.array([[DT**4 / 4, DT**3 / 2], [DT**3 / 2, DT**2]])


def vraie_position():
    vitesse = np.where(TEMPS < T_MANOEUVRE, 10.0, 20.0)
    pos = np.zeros(DUREE)
    for i in range(1, DUREE):
        pos[i] = pos[i - 1] + vitesse[i - 1] * DT
    return pos


def kalman_2d(mesures, R_tab):
    """Retourne (positions estimées, historique des gains K)."""
    X = np.zeros((2, 1))
    P = np.eye(2) * 1000.0
    pos, gains = np.zeros(DUREE), np.zeros((DUREE, 2))
    for i in range(DUREE):
        X, P = F @ X, F @ P @ F.T + Q
        R = np.array([[R_tab[i]]])
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        X = X + K @ (np.array([[mesures[i]]]) - H @ X)
        P = (np.eye(2) - K @ H) @ P
        pos[i], gains[i] = X[0, 0], K.ravel()
    return pos, gains


def main():
    SORTIE.mkdir(exist_ok=True)
    verite = vraie_position()
    R_normal = SIGMA_NORMAL**2
    R_anomalie = (SIGMA_NORMAL * FACTEUR_BROUILLAGE) ** 2
    R_non_prevenu = np.full(DUREE, R_normal)
    R_prevenu = R_non_prevenu.copy()
    R_prevenu[TEMPS >= T_BROUILLAGE] = R_anomalie

    rng = np.random.default_rng(GRAINE)
    erreurs = {"Mesures brutes": [], "Filtre non prévenu": [], "Filtre prévenu": []}
    for sim in range(N_SIM):
        bruit = rng.normal(0, SIGMA_NORMAL, DUREE)
        bruit[T_BROUILLAGE:] = rng.normal(0, SIGMA_NORMAL * FACTEUR_BROUILLAGE, DUREE - T_BROUILLAGE)
        mesures = verite + bruit
        pos_np, _ = kalman_2d(mesures, R_non_prevenu)
        pos_p, gains_p = kalman_2d(mesures, R_prevenu)
        erreurs["Mesures brutes"].append(mesures - verite)
        erreurs["Filtre non prévenu"].append(pos_np - verite)
        erreurs["Filtre prévenu"].append(pos_p - verite)
        if sim == 0:
            ex = (mesures, pos_np, pos_p, gains_p)

    lignes = ["| Estimateur | RMSE total (m) | RMSE pour t >= 30 s (m) |", "|---|---|---|"]
    for nom, e in erreurs.items():
        e = np.array(e)
        lignes.append(f"| {nom} | {np.sqrt(np.mean(e**2)):.1f} | {np.sqrt(np.mean(e[:, T_BROUILLAGE:]**2)):.1f} |")
    tableau = "\n".join(lignes)
    entete = (f"Paramètres : {N_SIM} simulations, graine {GRAINE}, var_a = {VAR_A}, "
              f"sigma = {SIGMA_NORMAL} m puis {SIGMA_NORMAL * FACTEUR_BROUILLAGE:.0f} m à t = {T_BROUILLAGE} s, "
              f"manoeuvre (10 -> 20 m/s) à t = {T_MANOEUVRE} s.\n\n")
    (SORTIE / "results_1d.md").write_text(entete + tableau + "\n", encoding="utf-8")
    print(entete + tableau)

    mesures, pos_np, pos_p, gains = ex
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax1.plot(TEMPS, verite, "g", lw=2, label="Vérité")
    ax1.scatter(TEMPS, mesures, c="red", alpha=0.4, s=15, label="Mesures")
    ax1.plot(TEMPS, pos_np, "orange", ls="--", label="Filtre non prévenu")
    ax1.plot(TEMPS, pos_p, "b", label="Filtre prévenu (R augmenté)")
    for t, c, lab in [(T_MANOEUVRE, "purple", "Manoeuvre"), (T_BROUILLAGE, "gray", "Brouillage")]:
        ax1.axvline(t, color=c, ls=":", label=lab)
        ax2.axvline(t, color=c, ls=":")
    ax1.set_ylabel("Position (m)")
    ax1.set_title("Kalman 1D : une simulation (graine 42)")
    ax1.legend(); ax1.grid(True)
    ax2.plot(TEMPS, gains[:, 0], "b", label="K position")
    ax2.plot(TEMPS, gains[:, 1], "teal", ls="--", label="K vitesse")
    ax2.set_xlabel("Temps (s)"); ax2.set_ylabel("Gain de Kalman"); ax2.legend(); ax2.grid(True)
    fig.tight_layout()
    fig.savefig(SORTIE / "kalman_1d.png", dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()