# Kalman 1D : suivi d'un drone avec capteur brouillé

python kalman_1d.py

Produit `results/results_1d.md` (RMSE sur 100 simulations, graine 42) et `results/kalman_1d.png` (une simulation et l'évolution des gains K).

Le RMSE est calculé sur l'ensemble des simulations (racine de la moyenne des carrés de toutes les erreurs), et non comme moyenne des RMSE de chaque simulation.

Voir le `README.md` à la racine du dépôt pour la méthode et l'interprétation.