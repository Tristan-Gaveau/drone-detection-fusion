"""Expériences : toutes les comparaisons, moyennées sur N_RUNS simulations.

Usage :  python experiments.py
Produit : results/results.md (tableaux) et results/*.png (figures).
"""
from pathlib import Path
import numpy as np

from ekf import run_ekf, X0_DEFAUT, P0_DEFAUT
from simulate import make_capteur, generer_trajectoire, simuler_mesures
from metrics import (erreur_position, rmse_fenetres, angle_lignes_de_visee,
                     distance_moyenne, erreurs_radiale_laterale)
from plotting import fig_erreur_vs_temps, fig_angles, fig_trajectoires

DT, DUREE = 0.5, 40
N_RUNS, VAR_A, SIGMA_DEG = 100, 1.0, 2.0
FENETRES = [(0, 10), (10, 20), (20, 30), (30, 40)]
SORTIE = Path(__file__).parent / "results"

# Deux placements de capteurs. Les noms décrivent la disposition, pas sa qualité :
# la qualité se mesure (angle entre lignes de visée, distance, RMSE).
CONFIGS = {
    "A": [make_capteur(20, 20, SIGMA_DEG), make_capteur(100, 20, SIGMA_DEG)],
    "B": [make_capteur(20, 100, SIGMA_DEG), make_capteur(150, -20, SIGMA_DEG)],
}
COULEURS = {"A": "tab:blue", "B": "tab:orange"}
X0_VERITE = np.array([[0.0], [0.0], [0.0], [0.0]])  # départ exact (pour tester la sensibilité)


def monte_carlo(config, temps, x, y, actifs=None, coupure=None, X0=None, P0=None):
    """N_RUNS simulations (graines 0..N_RUNS-1). Retourne X_all de forme (N_RUNS, n_temps, 4).

    actifs  : indices des capteurs utilisés (les autres sont éteints).
    coupure : (indice_capteur, t_debut, t_fin) -> mesures absentes sur cet intervalle.
    """
    actifs = range(len(config)) if actifs is None else actifs
    dt = temps[1] - temps[0]
    X_all = np.zeros((N_RUNS, len(temps), 4))
    for s in range(N_RUNS):
        m = simuler_mesures(temps, x, y, config, seed=s)
        for k in range(len(config)):
            if k not in actifs:
                m[k, :] = np.nan
        if coupure is not None:
            k, t0, t1 = coupure
            m[k, (temps >= t0) & (temps <= t1)] = np.nan
        X_all[s], _ = run_ekf(m, config, VAR_A, dt, X0, P0)
    return X_all


def md_table(entetes, lignes):
    out = ["| " + " | ".join(entetes) + " |", "|" + "---|" * len(entetes)]
    out += ["| " + " | ".join(str(c) for c in ligne) + " |" for ligne in lignes]
    return "\n".join(out)


def ligne_rmse(nom, X_all, temps, x, y):
    E = erreur_position(X_all, x, y)
    return [nom] + [f"{v:.1f}" for v in rmse_fenetres(E, temps, FENETRES)]


def main():
    SORTIE.mkdir(exist_ok=True)
    temps, x, y = generer_trajectoire(DT, DUREE)
    ent_fen = [f"RMSE {a}-{b} s" for a, b in FENETRES] + ["RMSE total"]
    md = [f"# Résultats\n\nParamètres : {N_RUNS} simulations (graines 0 à {N_RUNS - 1}), "
          f"var_a = {VAR_A}, bruit d'angle = {SIGMA_DEG}°, dt = {DT} s, durée = {DUREE} s, "
          f"virage du drone à t = 20 s. Erreurs en mètres.\n"]

    # --- 1. Géométrie des capteurs -------------------------------------------------
    md.append("## 1. Géométrie des capteurs\n")
    lignes, courbes, angles, marqueurs, X_geo = [], {}, {}, [], {}
    for nom, cfg in CONFIGS.items():
        ang = angle_lignes_de_visee(x, y, cfg[0], cfg[1])
        X_all = monte_carlo(cfg, temps, x, y)
        X_geo[nom] = X_all
        E = erreur_position(X_all, x, y)
        courbes[f"Config {nom}"] = (E, COULEURS[nom])
        angles[f"Config {nom}"] = (ang, COULEURS[nom])
        marqueurs.append((temps[ang.argmin()], COULEURS[nom], f"Angle minimal config {nom}"))
        lignes.append([f"Config {nom}", f"{ang.min():.1f}° (t = {temps[ang.argmin()]:.1f} s)",
                       f"{distance_moyenne(x, y, cfg):.0f} m"]
                      + [f"{v:.1f}" for v in rmse_fenetres(E, temps, FENETRES)])
    md.append(md_table(["Placement", "Angle min. entre visées", "Distance moy. capteur-drone"] + ent_fen, lignes))
    md.append("\nCapteurs : A = (20, 20) et (100, 20) ; B = (20, 100) et (150, -20).\n")
    md.append("![geometrie](angles_lignes_de_visee.png)\n![erreur](erreur_vs_temps_geometrie.png)\n")
    fig_angles(SORTIE / "angles_lignes_de_visee.png", temps, angles,
               "Angle entre les deux lignes de visée (90° = idéal, 0° = lignes confondues)")
    fig_erreur_vs_temps(SORTIE / "erreur_vs_temps_geometrie.png", temps, courbes,
                        f"Erreur de position, moyenne sur {N_RUNS} simulations (bande = percentiles 10-90)",
                        marqueurs=marqueurs)

    # --- 2. Architectures : capteurs seuls, fusion, coupures -----------------------
    for nom, cfg in CONFIGS.items():
        md.append(f"## 2{nom.lower()}. Architectures, placement {nom}\n")
        cas = {
            "Fusion (2 capteurs)": X_geo[nom],
            "Capteur 1 seul": monte_carlo(cfg, temps, x, y, actifs=[0]),
            "Capteur 2 seul": monte_carlo(cfg, temps, x, y, actifs=[1]),
            "Fusion, capteur 1 coupé de 10 à 20 s": monte_carlo(cfg, temps, x, y, coupure=(0, 10, 20)),
            "Fusion, capteur 1 coupé de 15 à 25 s (autour du virage)": monte_carlo(cfg, temps, x, y, coupure=(0, 15, 25)),
        }
        md.append(md_table(["Cas"] + ent_fen, [ligne_rmse(n, X, temps, x, y) for n, X in cas.items()]))
        # Capteur seul : erreur le long de la visée vs perpendiculaire
        rl = []
        for n, k in [("Capteur 1 seul", 0), ("Capteur 2 seul", 1)]:
            rad, lat = erreurs_radiale_laterale(cas[n], x, y, cfg[k])
            rl.append([n, f"{np.sqrt(np.mean(rad**2)):.1f}", f"{np.sqrt(np.mean(lat**2)):.1f}"])
        md.append("\nUn capteur d'angle seul : erreur selon la visée (radiale) et perpendiculairement (latérale).\n")
        md.append(md_table(["Cas", "RMSE radiale", "RMSE latérale"], rl))
        md.append("")
        if nom == "B":
            fig_erreur_vs_temps(
                SORTIE / "erreur_vs_temps_coupures_B.png", temps,
                {"Fusion": (erreur_position(cas["Fusion (2 capteurs)"], x, y), "tab:green"),
                 "Capteur 1 coupé 10-20 s": (erreur_position(cas["Fusion, capteur 1 coupé de 10 à 20 s"], x, y), "tab:purple"),
                 "Capteur 1 coupé 15-25 s": (erreur_position(cas["Fusion, capteur 1 coupé de 15 à 25 s (autour du virage)"], x, y), "tab:red")},
                "Placement B : effet d'une coupure du capteur 1 (moyenne sur les simulations)",
                marqueurs=[(20, "black", "Virage du drone")])
            md.append("![coupures](erreur_vs_temps_coupures_B.png)\n")

    # --- 3. Sensibilité à l'initialisation ------------------------------------------
    md.append("## 3. Sensibilité à l'initialisation\n")
    md.append("Départ « mesures » : construit à partir des premières mesures (défaut). "
              "Départ « arbitraire » : (50, 50) m avec une incertitude de 50 m. "
              "Départ « exact » : la vraie position (0, 0), irréaliste, donné à titre de référence.\n")
    for nom, cfg in CONFIGS.items():
        md.append(f"### Placement {nom}\n")
        lignes = []
        for libelle, actifs in [("Fusion", None), ("Capteur 1 seul", [0])]:
            for depart, X0, P0 in [("mesures", None, None),
                                   ("arbitraire (50, 50)", X0_DEFAUT, P0_DEFAUT),
                                   ("exact (0, 0)", X0_VERITE, P0_DEFAUT)]:
                X = monte_carlo(cfg, temps, x, y, actifs=actifs, X0=X0, P0=P0)
                lignes.append(ligne_rmse(f"{libelle}, départ {depart}", X, temps, x, y))
        md.append(md_table(["Cas"] + ent_fen, lignes) + "\n")

    # --- 4. Exemple d'une simulation : trajectoires et ellipses d'incertitude ------
    cfg = CONFIGS["B"]
    m = simuler_mesures(temps, x, y, cfg, seed=0)
    m1 = m.copy(); m1[1, :] = np.nan
    X1, P1 = run_ekf(m1, cfg, VAR_A, DT)
    X2, P2 = run_ekf(m, cfg, VAR_A, DT)
    fig_trajectoires(SORTIE / "trajectoires_ellipses_B.png", x, y, cfg,
                     [("1 capteur", X1, P1, "tab:red"), ("Fusion", X2, P2, "tab:green")],
                     "Placement B : une simulation (graine 0), ellipses d'incertitude à 3σ")
    md.append("## 4. Exemple d'une simulation (graine 0)\n\n![trajectoires](trajectoires_ellipses_B.png)\n")

    (SORTIE / "results.md").write_text("\n".join(md), encoding="utf-8")
    print("\n".join(md))


if __name__ == "__main__":
    main()