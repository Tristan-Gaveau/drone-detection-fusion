"""Figures."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse


def ellipses_incertitude(ax, X_hist, P_hist, step, color, label):
    """Ellipses à 3 sigmas de la covariance de position, tous les `step` instants."""
    premiere = True
    for i in range(0, len(X_hist), step):
        vals, vecs = np.linalg.eigh(P_hist[i, 0:2, 0:2])
        ordre = vals.argsort()[::-1]
        vals, vecs = vals[ordre], vecs[:, ordre]
        angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
        largeur, hauteur = 2 * 3 * np.sqrt(np.abs(vals))
        ax.add_patch(Ellipse((X_hist[i, 0], X_hist[i, 1]), largeur, hauteur, angle=angle, ec=color,
                             fc="none", ls="--", alpha=0.6, label=label if premiere else None))
        premiere = False


def fig_erreur_vs_temps(chemin, temps, courbes, titre, marqueurs=(), zones=()):
    """courbes : {nom: (E[n_runs, n_temps], couleur)} ; moyenne et bande 10-90 %."""
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for nom, (E, couleur) in courbes.items():
        ax.plot(temps, E.mean(axis=0), color=couleur, label=f"{nom} (moyenne)")
        ax.fill_between(temps, np.percentile(E, 10, axis=0), np.percentile(E, 90, axis=0),
                        color=couleur, alpha=0.15)
    for t, couleur, texte in marqueurs:
        ax.axvline(t, color=couleur, ls=":", label=texte)
    for t0, t1, texte in zones:
        ax.axvspan(t0, t1, color="gray", alpha=0.12, label=texte)
    ax.set_xlabel("Temps (s)"); ax.set_ylabel("Erreur de position (m)")
    ax.set_title(titre); ax.grid(True); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(chemin, dpi=130); plt.close(fig)


def fig_angles(chemin, temps, angles, titre):
    fig, ax = plt.subplots(figsize=(10, 3.8))
    for nom, (a, couleur) in angles.items():
        ax.plot(temps, a, color=couleur, label=nom)
    ax.set_xlabel("Temps (s)"); ax.set_ylabel("Angle entre lignes de visée (deg)")
    ax.set_title(titre); ax.grid(True); ax.legend()
    fig.tight_layout(); fig.savefig(chemin, dpi=130); plt.close(fig)


def fig_trajectoires(chemin, x, y, capteurs, series, titre):
    """series : liste de (nom, X_hist, P_hist, couleur)."""
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.plot(x, y, "k", lw=2, label="Vérité")
    for nom, X, P, couleur in series:
        ax.plot(X[:, 0], X[:, 1], color=couleur, label=nom)
        ellipses_incertitude(ax, X, P, step=15, color=couleur, label=f"Ellipses 3σ ({nom})")
    for k, c in enumerate(capteurs):
        ax.scatter(*c["pos"], marker="^", s=130, label=f"Capteur {k + 1}")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_title(titre)
    ax.axis("equal"); ax.grid(True); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(chemin, dpi=130); plt.close(fig)