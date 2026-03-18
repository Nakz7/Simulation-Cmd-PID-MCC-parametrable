# Simulation Cmd PID MCC paramétrable

Application PyQt6 de supervision industrielle permettant de simuler un moteur à courant continu (MCC) asservi en vitesse par un régulateur PID, avec paramétrage des grandeurs physiques et calcul des limites théoriques.

## Fonctionnalités
- Simulation temps réel d’un moteur CC
- Contrôle PID avec activation/désactivation des actions P, I, D
- Anti-windup
- Réglage de la consigne par slider
- Paramétrage physique du moteur :
  - inertie J
  - frottement visqueux b
  - constante K
  - résistance R
  - inductance L
  - tension maximale
- Affichage de la vitesse maximale théorique
- Visualisation temps réel avec PyQtGraph

## Installation
```bash
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
