# Documentation Technique et Modélisation Physique : Simulateur d'Asservissement de Moteur CC

## 1. Description du Système et Objectifs de l'Étude
Ce simulateur a été développé pour étudier le comportement dynamique et l'asservissement en vitesse d'un moteur à courant continu (CC). Conformément aux objectifs de l'ingénierie d'essais [1], ce système cyber-physique est divisé en deux sous-systèmes interdépendants :
1. **Le sous-système physique (Plant) :** Le moteur CC, modélisé par ses équations électromécaniques, qui convertit une tension d'entrée en une vitesse de rotation.
2. **Le sous-système de contrôle (Controller) :** L'algorithme PID, chargé de calculer l'effort de commande optimal pour annuler l'erreur entre la consigne de l'opérateur et la vitesse réelle mesurée.

L'objectif de ce modèle est de fournir un banc d'essai virtuel permettant de valider les performances du correcteur, de tester sa robustesse face aux variations des paramètres physiques (frottements, inertie) et de préparer une campagne de plans d'expériences (Design of Experiments).[1, 1]

## 2. Modélisation Mathématique du Moteur à Courant Continu
Le cœur du simulateur repose sur la résolution numérique d'un système d'équations différentielles linéaires couplées.[1] 

Sur le plan électrique, le circuit d'induit est régi par la loi des mailles. La tension d'alimentation $V(t)$ s'oppose à la chute de tension dans la résistance $R$, à la tension aux bornes de l'inductance $L$, et à la force contre-électromotrice (FCEM) induite par la rotation :
$$V(t) = R \cdot i(t) + L \frac{di(t)}{dt} + K \cdot \omega(t)$$

Sur le plan mécanique, l'équation fondamentale de la dynamique en rotation relie le couple moteur proportionnel au courant ($K \cdot i(t)$), le couple de frottement visqueux ($b \cdot \omega(t)$) et le moment d'inertie du rotor $J$ :
$$J \frac{d\omega(t)}{dt} = K \cdot i(t) - b \cdot \omega(t)$$

*Note : La constante de couple et la constante de FCEM sont considérées comme égales ($K$) dans le système international d'unités.*

Dans le code Python, ce système continu est discrétisé et résolu à chaque pas de temps $dt$ (typiquement 0.01 s) en utilisant la méthode d'intégration numérique d'Euler, permettant de calculer les nouvelles valeurs d'état du courant $i(t)$ et de la vitesse angulaire $\omega(t)$.

## 3. Algorithme de Contrôle : Le Régulateur PID Industriel
Pour asservir la vitesse du moteur, un régulateur Proportionnel-Intégral-Dérivé (PID) est implémenté. Le signal de commande $u(t)$ (la tension $V$) est généré en fonction de l'erreur $e(t) = \omega_{consigne}(t) - \omega_{mesure}(t)$ :
$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

L'implémentation logicielle intègre des sécurités industrielles indispensables :
*   **Débrayage des actions :** L'interface permet d'isoler les termes P, I et D pour analyser leur influence individuelle sur le régime transitoire (dépassement, temps de réponse).
*   **Protection Anti-Windup :** Si la tension calculée par le PID dépasse les limites physiques de l'alimentation (par exemple $\pm 24\text{ V}$), l'intégration de l'erreur est suspendue. Cela empêche l'accumulateur intégral de diverger et évite de fortes instabilités lors de la désaturation.

## 4. Analyse des Phénomènes Observés (Interprétation des Graphiques)
L'utilisation de l'interface graphique a permis de mettre en évidence deux comportements distincts du système, illustrés par les captures d'écran de la simulation :

### A. Régime Linéaire et Poursuite de Consigne (Première Image)
Sur la première capture, la consigne est fixée à $2.3\text{ rad/s}$. La courbe bleue (vitesse mesurée) rejoint parfaitement la courbe rouge en pointillés (consigne) après une courte phase transitoire, sans erreur statique. 
**Interprétation :** Le besoin en énergie pour atteindre cette vitesse est inférieur à la capacité maximale de l'alimentation (24V). Le système opère dans sa zone de linéarité. Les gains du PID ($K_p=135$, $K_i=110$, $K_d=12$) démontrent ici leur efficacité pour stabiliser le système rapidement.

### B. Saturation Physique de l'Actionneur (Seconde Image)
Sur la seconde capture, une consigne de $25.0\text{ tr/min}$ a été demandée. Cependant, la courbe de mesure bleue s'aplatit et plafonne aux alentours de $22.9\text{ tr/min}$, incapable de rejoindre la consigne rouge.
**Interprétation Physique :** Ce graphique illustre parfaitement le phénomène de **saturation de l'actionneur**. Le régulateur PID exige une tension supérieure à 24V pour vaincre l'inertie et les frottements afin d'atteindre 25 tr/min. Le système bride la tension à $V_{max} = 24\text{ V}$. 
À cette tension maximale, la vitesse limite théorique du moteur (en régime permanent, où $\frac{di}{dt}=0$ et $\frac{d\omega}{dt}=0$) est dictée par la physique du système :
$$\omega_{max} = \frac{V_{max} \cdot K}{R \cdot b + K^2}$$
Avec les paramètres nominaux ($R=1.0$, $b=0.1$, $K=0.01$), on obtient $\omega_{max} \approx 2.397\text{ rad/s}$.
En convertissant cette valeur en tours par minute :
$$\omega_{max(rpm)} = 2.397 \times \frac{60}{2\pi} \approx 22.89\text{ tr/min}$$
L'observation graphique valide donc rigoureusement le modèle mathématique sous-jacent. Le mécanisme *Anti-Windup* du code a joué son rôle : la courbe se stabilise proprement sous la consigne sans osciller de manière chaotique.

## 5. Exploitation pour la Planification Expérimentale (DoE)
Ce simulateur constitue la base de travail idéale pour entamer la "Partie II" de l'étude.[1] Grâce à l'onglet "Paramétrage Physique & Limites", l'expérimentateur peut désormais :
*   Identifier les facteurs influents en réalisant un plan de criblage ou un plan factoriel (par exemple, comment les variations de charge $J$ ou d'usure $b$ dégradent le temps de réponse).[1]
*   Utiliser la simulation déterministe pour générer les jeux de données (Overshoot, Settling Time) nécessaires au remplissage des matrices d'essais orthogonales (comme la table L9 de Taguchi).[1, 1]
*   Optimiser les paramètres du correcteur pour garantir une robustesse maximale face aux perturbations modélisées.