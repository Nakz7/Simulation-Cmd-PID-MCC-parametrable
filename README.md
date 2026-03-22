# Simulation Cmd PID MCC paramétrable

Application PyQt6 de supervision industrielle permettant de simuler un moteur à courant continu (MCC) asservi en vitesse par un régulateur PID. Elle intègre le paramétrage complet des grandeurs physiques, le calcul des limites théoriques (saturation) ainsi qu'un module de planification et d'exécution automatisée de campagnes d'essais (DoE).

## Fonctionnalités
- **Simulation temps réel d’un moteur CC :** Résolution par intégration numérique (méthode d'Euler).
- **Contrôle PID industriel :** 
  - Activation/désactivation indépendante des actions P, I et D.
  - Sécurité Anti-windup contre l'emballement de l'intégrateur.
- **Analyse des performances dynamiques :** Détection et calcul en temps réel du temps de montée (10-90%), du dépassement maximal (%) et de l'erreur statique (tr/min).
- **Paramétrage physique du moteur :**
  - Inertie du rotor ($J$) avec sécurité anti-division par zéro
  - Frottement visqueux ($b$)
  - Constante électromécanique ($K$)
  - Résistance ($R$) et Inductance ($L$) d'induit
  - Tension d'alimentation maximale (limites de saturation de l'actionneur)
- **Calculs prédictifs :** Affichage de la vitesse maximale théorique absolue atteignable à pleine charge.
- **Supervision graphique (PyQtGraph) :** Visualisation dynamique (oscilloscope) de la consigne et de la mesure, incluant la remise à l'échelle automatique (AutoRange).
- **Module Mesures & Essais (DoE) :**
  - Grille de données interactive de type "Excel" (glisser-déposer, copier-coller de plages multiples, sélection complète par colonnes).
  - Validation syntaxique stricte des saisies pour prévenir les crashs.
  - Lancement accéléré et automatisé des plans d'expériences en tâche de fond avec auto-remplissage des réponses (performances mesurées pour chaque essai).

## Installationbash
pip install -r requirements.txt
```

## Lancement
```bash
python simulation_moteur_industriel.py
```

## Dépendances
- Python 3.10+
- PyQt6
- pyqtgraph
- numpy
```
