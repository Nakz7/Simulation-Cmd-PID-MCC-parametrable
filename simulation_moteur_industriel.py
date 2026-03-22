#!/usr/bin/env python3
# -*- coding: utf-8 -*-

## @file simulation_moteur_industriel.py
#  @brief Application de supervision industrielle pour la simulation d'un moteur CC asservi.
#  @details Ce module implémente une interface permettant de paramétrer la
#  physique du moteur, de calculer ses limites théoriques (saturation), de simuler
#  son asservissement via un régulateur PID en temps réel, et d'extraire les
#  indicateurs de performance dynamiques. Il inclut un outil de grille (DoE) pour
#  les campagnes d'essais.
#  @author Neil Legendre-Ferreira Da Costa
#  @date 2026-03-22

import sys
import numpy as np
import pyqtgraph as pg
from PyQt6 import QtWidgets, QtCore, QtGui


## @class DcMotorPhysicalModel
#  @brief Modèle mathématique et physique d'un moteur à courant continu.
#  @details Résout les équations différentielles électromécaniques couplées
#  par intégration numérique (méthode d'Euler).
class DcMotorPhysicalModel:
    ## @brief Constructeur du modèle physique.
    #  @param inertia_j Moment d'inertie du rotor (kg.m^2).
    #  @param friction_b Coefficient de frottement visqueux (N.m.s).
    #  @param constant_k Constante électromécanique (V/rad/s ou N.m/A).
    #  @param resistance_r Résistance de l'induit (Ohm).
    #  @param inductance_l Inductance de l'induit (H).
    def __init__(self, inertia_j=0.01, friction_b=0.1, constant_k=0.01, resistance_r=1.0, inductance_l=0.5):
        self.inertia_j = inertia_j
        self.friction_b = friction_b
        self.constant_k = constant_k
        self.resistance_r = resistance_r
        self.inductance_l = inductance_l

        ## @var speed_rad_s
        #  Vitesse angulaire instantanée du rotor en radians par seconde.
        self.speed_rad_s = 0.0

        ## @var current_amp
        #  Courant d'induit instantané en Ampères.
        self.current_amp = 0.0

    ## @brief Met à jour les paramètres physiques du moteur à la volée.
    def update_physical_parameters(self, inertia_j, friction_b, constant_k, resistance_r, inductance_l):
        self.inertia_j = inertia_j
        self.friction_b = friction_b
        self.constant_k = constant_k
        self.resistance_r = resistance_r
        self.inductance_l = inductance_l

    ## @brief Exécute un pas de calcul de la simulation physique.
    #  @param applied_voltage_v Tension d'alimentation appliquée à l'induit (Volts).
    #  @param time_step_s Pas de temps de l'intégration numérique (secondes).
    #  @return La nouvelle vitesse angulaire atteinte (rad/s).
    def execute_simulation_step(self, applied_voltage_v, time_step_s):
        # Calcul des dérivées temporelles
        current_derivative = (
                                         applied_voltage_v - self.constant_k * self.speed_rad_s - self.resistance_r * self.current_amp) / self.inductance_l
        speed_derivative = (self.constant_k * self.current_amp - self.friction_b * self.speed_rad_s) / self.inertia_j

        # Intégration d'Euler
        self.current_amp += current_derivative * time_step_s
        self.speed_rad_s += speed_derivative * time_step_s

        return self.speed_rad_s


## @class PidControllerIndustrial
#  @brief Régulateur PID industriel incluant une protection Anti-Windup dynamique.
class PidControllerIndustrial:
    ## @brief Constructeur du régulateur.
    #  @param gain_p Gain de l'action proportionnelle.
    #  @param gain_i Gain de l'action intégrale.
    #  @param gain_d Gain de l'action dérivée.
    #  @param saturation_limit_v Limite absolue de tension de sortie (+/- Volts).
    def __init__(self, gain_p, gain_i, gain_d, saturation_limit_v=24.0):
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.gain_d = gain_d
        self.saturation_max_v = saturation_limit_v
        self.saturation_min_v = -saturation_limit_v

        self.error_integral = 0.0
        self.previous_error = 0.0

    ## @brief Met à jour les limites de saturation de l'actionneur.
    def update_saturation_limits(self, max_voltage_v):
        self.saturation_max_v = max_voltage_v
        self.saturation_min_v = -max_voltage_v

    ## @brief Met à jour les gains du régulateur PID à la volée.
    def update_gains(self, gain_p, gain_i, gain_d):
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.gain_d = gain_d

    ## @brief Calcule le signal de commande avec possibilité de débrayer les actions.
    #  @param setpoint_target Valeur de consigne cible.
    #  @param measured_feedback Valeur mesurée par le capteur.
    #  @param time_step_s Pas de temps de la boucle (secondes).
    #  @param enable_p Active le terme Proportionnel.
    #  @param enable_i Active le terme Intégral.
    #  @param enable_d Active le terme Dérivé.
    #  @return Effort de commande borné (Volts).
    def compute_control_effort(self, setpoint_target, measured_feedback, time_step_s, enable_p=True, enable_i=True,
                               enable_d=True):
        current_error = setpoint_target - measured_feedback

        # Action Proportionnelle
        term_p = self.gain_p * current_error if enable_p else 0.0

        # Action Intégrale
        if enable_i:
            self.error_integral += current_error * time_step_s
            term_i = self.gain_i * self.error_integral
        else:
            self.error_integral = 0.0
            term_i = 0.0

        # Action Dérivée
        error_derivative = (current_error - self.previous_error) / time_step_s
        term_d = self.gain_d * error_derivative if enable_d else 0.0

        command_output = term_p + term_i + term_d

        # Anti-Windup
        if command_output > self.saturation_max_v:
            command_output = self.saturation_max_v
            if enable_i: self.error_integral -= current_error * time_step_s
        elif command_output < self.saturation_min_v:
            command_output = self.saturation_min_v
            if enable_i: self.error_integral -= current_error * time_step_s

        self.previous_error = current_error
        return command_output


## @class ExperimentDataGrid
#  @brief Grille de données avancée pour les plans d'expériences.
#  @details Implémente le copier/coller (depuis/vers Excel), le glisser-déposer (Drag & Drop),
#  la sélection par colonnes, le comportement de la touche Entrée et une vérification stricte.
class ExperimentDataGrid(QtWidgets.QTableWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Configuration du Drag and Drop et de la sélection
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.viewport().setAcceptDrops(True)
        self.setDragDropOverwriteMode(True)  # Permet d'écraser la cellule de destination (Excel-like)
        self.setDropIndicatorShown(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectItems)
        self.setDragDropMode(QtWidgets.QAbstractItemView.DragDropMode.DragDrop)

        # Rendre les en-têtes cliquables pour la sélection de colonnes
        self.horizontalHeader().setSectionsClickable(True)
        self.horizontalHeader().sectionClicked.connect(self._on_header_clicked)

        # Connexion du signal de modification pour la validation
        self.itemChanged.connect(self._on_item_changed)

    ## @brief Sélectionne la colonne entière lors du clic sur son en-tête.
    #  @param logical_index Index de la colonne cliquée.
    def _on_header_clicked(self, logical_index):
        self.selectColumn(logical_index)

    ## @brief Surcharge de l'événement de dépôt (Drop) pour assurer la sécurité de la grille.
    #  @details Comble les vides laissés par le déplacement des cellules avec "0.0".
    def dropEvent(self, event):
        super().dropEvent(event)
        self.blockSignals(True)
        for r in range(self.rowCount()):
            for c in range(self.columnCount()):
                if self.item(r, c) is None or self.item(r, c).text().strip() == "":
                    self.setItem(r, c, QtWidgets.QTableWidgetItem("0.0"))
        self.blockSignals(False)

    ## @brief Intercepte les événements clavier pour gérer le copier-coller et l'appui sur Entrée.
    def keyPressEvent(self, event):
        if event.matches(QtGui.QKeySequence.StandardKey.Copy):
            self._copy_to_clipboard()
        elif event.matches(QtGui.QKeySequence.StandardKey.Paste):
            self._paste_from_clipboard()
        elif event.key() == QtCore.Qt.Key.Key_Delete:
            self._delete_selection()
        elif event.key() in (QtCore.Qt.Key.Key_Return, QtCore.Qt.Key.Key_Enter):
            row = self.currentRow()
            col = self.currentColumn()
            super().keyPressEvent(event)  # Laisse le composant finir l'édition en cours

            # Déplacement intelligent vers la ligne suivante dans la même colonne
            if row < self.rowCount() - 1:
                self.setCurrentCell(row + 1, col)
        else:
            super().keyPressEvent(event)

    ## @brief Copie la sélection actuelle dans le presse-papiers (format TSV).
    def _copy_to_clipboard(self):
        selection = self.selectedRanges()
        if not selection:
            return

        sel = selection
        copy_text = ""
        for row in range(sel.topRow(), sel.bottomRow() + 1):
            row_data = ''
            for col in range(sel.leftColumn(), sel.rightColumn() + 1):
                item = self.item(row, col)
                row_data.append(item.text() if item else "")
            copy_text += "\t".join(row_data) + "\n"

        QtGui.QGuiApplication.clipboard().setText(copy_text)

    ## @brief Colle le contenu du presse-papiers dans la grille avec comportement intelligent (Remplissage).
    def _paste_from_clipboard(self):
        text = QtGui.QGuiApplication.clipboard().text().strip('\n')
        if not text:
            return

        selection = self.selectedRanges()
        if not selection:
            return

        sel = selection
        start_row = sel.topRow()
        start_col = sel.leftColumn()

        self.blockSignals(True)
        rows = text.split('\n')

        # Comportement "Excel-like" : Si on copie une seule valeur et qu'on a sélectionné plusieurs cellules
        if len(rows) == 1 and '\t' not in rows:
            single_val = rows.strip()
            for sel_range in selection:
                for r in range(sel_range.topRow(), sel_range.bottomRow() + 1):
                    for c in range(sel_range.leftColumn(), sel_range.rightColumn() + 1):
                        item = self.item(r, c)
                        if not item:
                            item = QtWidgets.QTableWidgetItem()
                            self.setItem(r, c, item)
                        item.setText(single_val)
                        self._validate_cell_value(item)
        else:
            # Comportement normal : On colle la plage aux coordonnées de départ
            for r_idx, row_str in enumerate(rows):
                cells = row_str.split('\t')
                for c_idx, cell_text in enumerate(cells):
                    current_row = start_row + r_idx
                    current_col = start_col + c_idx

                    if current_row < self.rowCount() and current_col < self.columnCount():
                        item = self.item(current_row, current_col)
                        if not item:
                            item = QtWidgets.QTableWidgetItem()
                            self.setItem(current_row, current_col, item)
                        item.setText(cell_text.strip())
                        self._validate_cell_value(item)

        self.blockSignals(False)

    ## @brief Supprime le contenu des cellules sélectionnées.
    def _delete_selection(self):
        self.blockSignals(True)
        for item in self.selectedItems():
            item.setText("0.0")
        self.blockSignals(False)

    ## @brief Callback déclenché à chaque modification de cellule.
    def _on_item_changed(self, item):
        self.blockSignals(True)
        self._validate_cell_value(item)
        self.blockSignals(False)

    ## @brief Vérifie la cohérence numérique d'une cellule et corrige les erreurs.
    def _validate_cell_value(self, item):
        if not item or not item.text().strip():
            item.setText("0.0") if item else None
            return

        # Remplacement de la virgule par un point pour la conversion float
        text = item.text().replace(',', '.')

        try:
            val = float(text)
            item.setText(str(val))  # Standardisation du format
        except ValueError:
            QtWidgets.QMessageBox.warning(
                self,
                "Erreur de Syntaxe",
                f"La valeur saisie '{item.text()}' n'est pas numérique.\nLa cellule a été réinitialisée à 0.0."
            )
            item.setText("0.0")


## @class IndustrialSupervisionDashboard
#  @brief IHM principale intégrant un système d'onglets pour la supervision.
class IndustrialSupervisionDashboard(QtWidgets.QMainWindow):
    ## @brief Initialise le dashboard, les modèles et l'architecture UI.
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Supervision Industrielle - Asservissement & Paramétrage")
        self.resize(1100, 700)

        # Modèles métiers
        self.motor_model = DcMotorPhysicalModel()
        self.pid_controller = PidControllerIndustrial(gain_p=135.0, gain_i=110.0, gain_d=12.0)

        # Paramètres temporels globaux
        self.time_step_s = 0.01
        self.elapsed_time_s = 0.0

        # Variables pour l'analyse des performances en temps réel
        self.initial_speed_rpm = 0.0
        self.maximum_speed_rpm = 0.0
        self.time_at_10_percent_s = None
        self.time_at_90_percent_s = None

        # Consigne
        self.target_speed_rpm = 100.0
        self.target_speed_rad_s = self.target_speed_rpm * (2 * np.pi) / 60.0

        # Buffers de traçage (Fenêtre glissante)
        self.max_display_points = 500
        self.buffer_time = [0.0] * self.max_display_points
        self.buffer_measured_rpm = [0.0] * self.max_display_points
        self.buffer_target_rpm = [0.0] * self.max_display_points

        self._build_main_interface()
        self._initialize_realtime_timer()
        self._calculate_theoretical_maximum()

    ## @brief Construit le widget central et le gestionnaire d'onglets.
    def _build_main_interface(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)

        self.tab_manager = QtWidgets.QTabWidget()

        # Création des onglets
        self.tab_simulation = QtWidgets.QWidget()
        self.tab_physics = QtWidgets.QWidget()
        self.tab_experiments = QtWidgets.QWidget()

        self._populate_simulation_tab()
        self._populate_physics_tab()
        self._populate_experiments_tab()

        self.tab_manager.addTab(self.tab_simulation, "Simulation & Contrôle PID")
        self.tab_manager.addTab(self.tab_physics, "Paramétrage Physique & Limites")
        self.tab_manager.addTab(self.tab_experiments, "Mesures & Essais (DoE)")

        main_layout.addWidget(self.tab_manager)

    ## @brief Construit l'interface de l'onglet de Simulation (Boutons, Graphes, Métriques).
    def _populate_simulation_tab(self):
        layout = QtWidgets.QHBoxLayout(self.tab_simulation)
        control_panel = QtWidgets.QVBoxLayout()

        # Boutons de commande
        self.btn_run_simulation = QtWidgets.QPushButton("Démarrer Simulation")
        self.btn_run_simulation.clicked.connect(self.start_simulation_loop)

        self.btn_pause_simulation = QtWidgets.QPushButton("Mettre en Pause")
        self.btn_pause_simulation.clicked.connect(self.stop_simulation_loop)

        self.btn_reset_states = QtWidgets.QPushButton("Réinitialiser Système")
        self.btn_reset_states.clicked.connect(self.reset_simulation_states)

        # Contrôle des actions PID
        self.checkbox_p = QtWidgets.QCheckBox("Action Proportionnelle (P)")
        self.checkbox_p.setChecked(True)
        self.checkbox_i = QtWidgets.QCheckBox("Action Intégrale (I)")
        self.checkbox_i.setChecked(True)
        self.checkbox_d = QtWidgets.QCheckBox("Action Dérivée (D)")
        self.checkbox_d.setChecked(True)

        # Consigne Utilisateur
        self.lbl_target_display = QtWidgets.QLabel(f"Consigne Vitesse: {self.target_speed_rpm} tr/min")
        self.slider_target = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider_target.setMinimum(0)
        self.slider_target.setMaximum(500)
        self.slider_target.setValue(int(self.target_speed_rpm))
        self.slider_target.valueChanged.connect(self.on_target_slider_changed)

        # --- Panneau d'Analyse des Performances ---
        self.group_metrics = QtWidgets.QGroupBox("Analyse des Performances")
        metrics_layout = QtWidgets.QVBoxLayout()

        self.lbl_rise_time = QtWidgets.QLabel("Temps de montée (10-90%) : -- s")
        self.lbl_overshoot = QtWidgets.QLabel("Dépassement maximal : -- %")
        self.lbl_steady_error = QtWidgets.QLabel("Erreur statique : -- tr/min")

        style = "font-weight: bold; color: #FFFFFF; font-size: 13px; padding: 2px;"
        self.lbl_rise_time.setStyleSheet(style)
        self.lbl_overshoot.setStyleSheet(style)
        self.lbl_steady_error.setStyleSheet(style)
        self.group_metrics.setStyleSheet(
            "QGroupBox { font-weight: bold; border: 1px solid #95A5A6; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }")

        metrics_layout.addWidget(self.lbl_rise_time)
        metrics_layout.addWidget(self.lbl_overshoot)
        metrics_layout.addWidget(self.lbl_steady_error)
        self.group_metrics.setLayout(metrics_layout)

        # Assemblage du panneau
        control_panel.addWidget(self.btn_run_simulation)
        control_panel.addWidget(self.btn_pause_simulation)
        control_panel.addWidget(self.btn_reset_states)
        control_panel.addSpacing(15)
        control_panel.addWidget(self.checkbox_p)
        control_panel.addWidget(self.checkbox_i)
        control_panel.addWidget(self.checkbox_d)
        control_panel.addSpacing(15)
        control_panel.addWidget(self.lbl_target_display)
        control_panel.addWidget(self.slider_target)
        control_panel.addSpacing(10)
        control_panel.addWidget(self.group_metrics)
        control_panel.addStretch()

        # Zone Graphique
        self.plot_widget = pg.PlotWidget(title="Oscilloscope - Réponse Temporelle")
        self.plot_widget.setBackground('w')
        self.plot_widget.setLabel('left', 'Vitesse', units='tr/min')
        self.plot_widget.setLabel('bottom', 'Temps', units='s')
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True, y=True)

        pen_target = pg.mkPen(color='r', style=QtCore.Qt.PenStyle.DashLine, width=2)
        pen_measure = pg.mkPen(color='b', width=2)
        self.curve_target = self.plot_widget.plot(self.buffer_time, self.buffer_target_rpm, name="Consigne",
                                                  pen=pen_target)
        self.curve_measure = self.plot_widget.plot(self.buffer_time, self.buffer_measured_rpm, name="Mesure",
                                                   pen=pen_measure)

        layout.addLayout(control_panel, 1)
        layout.addWidget(self.plot_widget, 4)

    ## @brief Construit l'interface de l'onglet des Paramètres (Physique & PID).
    def _populate_physics_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_physics)
        form_layout = QtWidgets.QFormLayout()

        def create_spinbox(value, step, decimals, maximum=1000.0):
            spinbox = QtWidgets.QDoubleSpinBox()
            spinbox.setDecimals(decimals)
            spinbox.setSingleStep(step)
            spinbox.setRange(0.0, maximum)
            spinbox.setValue(value)
            return spinbox

        # Paramètres PID
        self.spin_gain_p = create_spinbox(135.0, 1.0, 2)
        self.spin_gain_i = create_spinbox(110.0, 1.0, 2)
        self.spin_gain_d = create_spinbox(12.0, 0.1, 2)

        # Paramètres Physiques
        self.spin_inertia_j = create_spinbox(0.01, 0.001, 4)
        self.spin_friction_b = create_spinbox(0.1, 0.01, 4)
        self.spin_constant_k = create_spinbox(0.01, 0.001, 4)
        self.spin_resistance_r = create_spinbox(1.0, 0.1, 2)
        self.spin_inductance_l = create_spinbox(0.5, 0.01, 3)
        self.spin_voltage_max = create_spinbox(24.0, 1.0, 1)

        lbl_pid_title = QtWidgets.QLabel("--- Paramètres du Régulateur PID ---")
        lbl_pid_title.setStyleSheet("font-weight: bold; color: #2980B9; margin-top: 10px;")
        form_layout.addRow(lbl_pid_title)
        form_layout.addRow("Gain Proportionnel (Kp):", self.spin_gain_p)
        form_layout.addRow("Gain Intégral (Ki):", self.spin_gain_i)
        form_layout.addRow("Gain Dérivé (Kd):", self.spin_gain_d)

        lbl_phys_title = QtWidgets.QLabel("--- Paramètres Physiques du Moteur ---")
        lbl_phys_title.setStyleSheet("font-weight: bold; color: #C0392B; margin-top: 20px;")
        form_layout.addRow(lbl_phys_title)
        form_layout.addRow("Moment d'inertie J (kg.m²):", self.spin_inertia_j)
        form_layout.addRow("Frottement visqueux b (N.m.s):", self.spin_friction_b)
        form_layout.addRow("Constante K (V.s/rad):", self.spin_constant_k)
        form_layout.addRow("Résistance d'induit R (Ω):", self.spin_resistance_r)
        form_layout.addRow("Inductance d'induit L (H):", self.spin_inductance_l)
        form_layout.addRow("Tension d'Alimentation Max V (Volts):", self.spin_voltage_max)

        self.btn_apply_physics = QtWidgets.QPushButton("Appliquer les Paramètres & Calculer les Limites")
        self.btn_apply_physics.setStyleSheet("font-weight: bold; padding: 10px; margin-top: 15px;")
        self.btn_apply_physics.clicked.connect(self.on_apply_physical_parameters)

        self.lbl_theoretical_max = QtWidgets.QLabel("Vitesse Maximale Théorique : -- rad/s (-- tr/min)")
        self.lbl_theoretical_max.setStyleSheet("font-size: 14px; color: #333; margin-top: 15px;")

        layout.addLayout(form_layout)
        layout.addWidget(self.btn_apply_physics)
        layout.addWidget(self.lbl_theoretical_max)
        layout.addStretch()

    ## @brief Construit l'interface de l'onglet Mesures & Essais (Matrice de plan d'expériences).
    def _populate_experiments_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_experiments)

        # Barre de contrôle supérieure
        control_layout = QtWidgets.QHBoxLayout()
        lbl_rows = QtWidgets.QLabel("Nombre d'essais configurés :")
        lbl_rows.setStyleSheet("font-weight: bold;")

        self.spin_experiment_rows = QtWidgets.QSpinBox()
        self.spin_experiment_rows.setRange(1, 1000)
        self.spin_experiment_rows.setValue(9)
        self.spin_experiment_rows.setMinimumWidth(70)
        self.spin_experiment_rows.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        btn_apply_rows = QtWidgets.QPushButton("Mettre à jour la grille")
        btn_apply_rows.clicked.connect(self.on_update_experiment_grid_rows)

        self.btn_run_experiments = QtWidgets.QPushButton("Lancer les essais")
        self.btn_run_experiments.setStyleSheet(
            "font-weight: bold; background-color: #27AE60; color: white; padding: 5px 15px;")
        self.btn_run_experiments.clicked.connect(self.on_run_experiments)

        control_layout.addWidget(lbl_rows)
        control_layout.addWidget(self.spin_experiment_rows)
        control_layout.addWidget(btn_apply_rows)
        control_layout.addSpacing(20)
        control_layout.addWidget(self.btn_run_experiments)
        control_layout.addStretch()

        # Grille interactive (ExperimentDataGrid)
        self.grid_experiments = ExperimentDataGrid()
        self.grid_experiments.setColumnCount(7)

        # Ajout de la liste d'étiquettes correspondantes
        labels = ["K_p", "K_i", "K_d", "J","Dépassement", "Temps de montée", "Erreur statique"]
        self.grid_experiments.setHorizontalHeaderLabels(labels)

        # Ajustement du style et de la taille des colonnes
        header = self.grid_experiments.horizontalHeader()
        header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.grid_experiments.setStyleSheet("QTableWidget { gridline-color: #BDC3C7; }")

        self.on_update_experiment_grid_rows()  # Initialisation des lignes

        layout.addLayout(control_layout)
        layout.addWidget(self.grid_experiments)

    ## @brief Met à jour le nombre de lignes de la grille d'essais sans effacer les données existantes.
    def on_update_experiment_grid_rows(self):
        new_row_count = self.spin_experiment_rows.value()
        self.grid_experiments.setRowCount(new_row_count)

        # Initialisation des cellules vides par "0.0"
        self.grid_experiments.blockSignals(True)
        for r in range(new_row_count):
            for c in range(self.grid_experiments.columnCount()):
                if not self.grid_experiments.item(r, c):
                    self.grid_experiments.setItem(r, c, QtWidgets.QTableWidgetItem("0.0"))
        self.grid_experiments.blockSignals(False)

    ## @brief Configure le séquenceur temps réel.
    def _initialize_realtime_timer(self):
        self.simulation_timer = QtCore.QTimer()
        self.simulation_timer.setInterval(int(self.time_step_s * 1000))
        self.simulation_timer.timeout.connect(self.execute_simulation_cycle)

    ## @brief Callback lié au mouvement du slider de consigne.
    def on_target_slider_changed(self, value):
        self.target_speed_rpm = float(value)
        self.target_speed_rad_s = self.target_speed_rpm * (2 * np.pi) / 60.0
        self.lbl_target_display.setText(f"Consigne Vitesse: {self.target_speed_rpm:.1f} tr/min")

        self.initial_speed_rpm = self.buffer_measured_rpm[-1] if len(self.buffer_measured_rpm) > 0 else 0.0
        self.maximum_speed_rpm = self.initial_speed_rpm
        self.time_at_10_percent_s = None
        self.time_at_90_percent_s = None

        self.lbl_rise_time.setText("Temps de montée (10-90%) : -- s")
        self.lbl_overshoot.setText("Dépassement maximal : -- %")

    ## @brief Callback pour appliquer les modifications physiques et calculer la limite.
    def on_apply_physical_parameters(self):
        kp = self.spin_gain_p.value()
        ki = self.spin_gain_i.value()
        kd = self.spin_gain_d.value()

        j = self.spin_inertia_j.value()
        b = self.spin_friction_b.value()
        k = self.spin_constant_k.value()
        r = self.spin_resistance_r.value()
        l = self.spin_inductance_l.value()
        v_max = self.spin_voltage_max.value()

        # Sécurité Division par Zéro
        if j <= 0.0:
            QtWidgets.QMessageBox.critical(self, "Erreur de Physique",
                                           "Le moment d'inertie J ne peut pas être nul ou négatif.\nVeuillez saisir une valeur valide pour éviter une division par zéro.")
            self.spin_inertia_j.setValue(0.01)
            return

        self.pid_controller.update_gains(kp, ki, kd)
        self.pid_controller.update_saturation_limits(v_max)
        self.motor_model.update_physical_parameters(j, b, k, r, l)

        self._calculate_theoretical_maximum()

    ## @brief Exécute de manière automatisée et accélérée tous les essais renseignés dans la grille DoE.
    def on_run_experiments(self):
        b = self.spin_friction_b.value()
        k = self.spin_constant_k.value()
        r_res = self.spin_resistance_r.value()
        l = self.spin_inductance_l.value()
        v_max = self.spin_voltage_max.value()

        target_rpm = self.target_speed_rpm
        target_rad = self.target_speed_rad_s
        dt = self.time_step_s
        sim_duration_s = 10.0

        enable_p = self.checkbox_p.isChecked()
        enable_i = self.checkbox_i.isChecked()
        enable_d = self.checkbox_d.isChecked()

        self.grid_experiments.blockSignals(True)

        for row in range(self.grid_experiments.rowCount()):
            try:
                kp = float(self.grid_experiments.item(row, 0).text())
                ki = float(self.grid_experiments.item(row, 1).text())
                kd = float(self.grid_experiments.item(row, 2).text())
                j_inertia = float(self.grid_experiments.item(row, 3).text())
            except (ValueError, AttributeError):
                continue

            if j_inertia <= 0.0:
                QtWidgets.QMessageBox.critical(
                    self,
                    "Interruption des Essais",
                    f"Erreur à la ligne {row + 1} : L'inertie J est nulle ou négative.\n"
                    "L'algorithme de simulation a été interrompu pour éviter une division par zéro."
                )
                self.grid_experiments.blockSignals(False)
                return

            test_motor = DcMotorPhysicalModel(inertia_j=j_inertia, friction_b=b, constant_k=k, resistance_r=r_res,
                                              inductance_l=l)
            test_pid = PidControllerIndustrial(gain_p=kp, gain_i=ki, gain_d=kd, saturation_limit_v=v_max)

            t = 0.0
            max_rpm = 0.0
            t10 = None
            t90 = None
            step_amp = target_rpm

            while t < sim_duration_s:
                voltage = test_pid.compute_control_effort(target_rad, test_motor.speed_rad_s, dt, enable_p, enable_i,
                                                          enable_d)
                speed_rad = test_motor.execute_simulation_step(voltage, dt)
                speed_rpm = speed_rad * 60.0 / (2 * np.pi)

                if speed_rpm > max_rpm:
                    max_rpm = speed_rpm

                if step_amp > 1.0:
                    if t10 is None and speed_rpm >= 0.10 * step_amp:
                        t10 = t
                    if t90 is None and speed_rpm >= 0.90 * step_amp:
                        t90 = t

                t += dt

            overshoot = max(0.0, ((max_rpm - target_rpm) / target_rpm) * 100.0) if target_rpm > 0 else 0.0
            steady_error = target_rpm - speed_rpm

            # Gérer visuellement le cas d'une saturation physique lors de l'essai
            if t90 is not None and t10 is not None:
                rise_time_str = f"{(t90 - t10):.3f}"
            else:
                rise_time_str = "Saturé"

            self.grid_experiments.setItem(row, 4, QtWidgets.QTableWidgetItem(f"{overshoot:.2f}"))
            self.grid_experiments.setItem(row, 5, QtWidgets.QTableWidgetItem(rise_time_str))
            self.grid_experiments.setItem(row, 6, QtWidgets.QTableWidgetItem(f"{steady_error:.2f}"))

        self.grid_experiments.blockSignals(False)
        QtWidgets.QMessageBox.information(self, "Plan d'Expériences Terminé",
                                          "Tous les essais ont été simulés avec succès.")

    ## @brief Calcule et affiche la vitesse de saturation théorique du moteur.
    def _calculate_theoretical_maximum(self):
        b = self.motor_model.friction_b
        r = self.motor_model.resistance_r
        k = self.motor_model.constant_k
        v_max = self.pid_controller.saturation_max_v

        denominator = (r * b) + (k ** 2)

        if denominator > 0:
            omega_max_rad_s = (v_max * k) / denominator
            rpm_max = omega_max_rad_s * 60.0 / (2 * np.pi)
            text = f"Vitesse Maximale Théorique (Pleine Charge) :\n{omega_max_rad_s:.2f} rad/s\n{rpm_max:.1f} tr/min"
        else:
            text = "Erreur de paramètres (Dénominateur nul)."

        self.lbl_theoretical_max.setText(text)

    def start_simulation_loop(self):
        self.simulation_timer.start()

    def stop_simulation_loop(self):
        self.simulation_timer.stop()

    ## @brief Réinitialise l'état physique, le PID, et replace le graphe aux conditions initiales.
    def reset_simulation_states(self):
        self.simulation_timer.stop()
        self.motor_model.speed_rad_s = 0.0
        self.motor_model.current_amp = 0.0
        self.pid_controller.error_integral = 0.0
        self.pid_controller.previous_error = 0.0
        self.elapsed_time_s = 0.0

        self.initial_speed_rpm = 0.0
        self.maximum_speed_rpm = 0.0
        self.time_at_10_percent_s = None
        self.time_at_90_percent_s = None
        self.lbl_rise_time.setText("Temps de montée (10-90%) : -- s")
        self.lbl_overshoot.setText("Dépassement maximal : -- %")
        self.lbl_steady_error.setText("Erreur statique : -- tr/min")

        self.buffer_time = [0.0] * self.max_display_points
        self.buffer_measured_rpm = [0.0] * self.max_display_points
        self.buffer_target_rpm = [0.0] * self.max_display_points
        self._refresh_plot_curves()

        self.plot_widget.enableAutoRange(axis='xy')

    ## @brief Cycle d'exécution principal appelé par le Timer.
    def execute_simulation_cycle(self):
        self.elapsed_time_s += self.time_step_s

        measured_feedback_rad_s = self.motor_model.speed_rad_s

        command_voltage_v = self.pid_controller.compute_control_effort(
            self.target_speed_rad_s,
            measured_feedback_rad_s,
            self.time_step_s,
            enable_p=self.checkbox_p.isChecked(),
            enable_i=self.checkbox_i.isChecked(),
            enable_d=self.checkbox_d.isChecked()
        )

        new_speed_rad_s = self.motor_model.execute_simulation_step(command_voltage_v, self.time_step_s)
        new_speed_rpm = new_speed_rad_s * 60.0 / (2 * np.pi)

        # 1. Erreur Statique
        error_rpm = self.target_speed_rpm - new_speed_rpm
        self.lbl_steady_error.setText(f"Erreur statique : {error_rpm:+.2f} tr/min")

        # 2. Dépassement & Temps de Montée
        step_amplitude = self.target_speed_rpm - self.initial_speed_rpm
        if step_amplitude > 1.0:
            if new_speed_rpm > self.maximum_speed_rpm:
                self.maximum_speed_rpm = new_speed_rpm
                if self.target_speed_rpm > 0:
                    overshoot_pct = ((self.maximum_speed_rpm - self.target_speed_rpm) / self.target_speed_rpm) * 100.0
                    self.lbl_overshoot.setText(f"Dépassement maximal : {max(0.0, overshoot_pct):.2f} %")

            threshold_10 = self.initial_speed_rpm + 0.10 * step_amplitude
            threshold_90 = self.initial_speed_rpm + 0.90 * step_amplitude

            if self.time_at_10_percent_s is None and new_speed_rpm >= threshold_10:
                self.time_at_10_percent_s = self.elapsed_time_s

            if self.time_at_90_percent_s is None and new_speed_rpm >= threshold_90:
                self.time_at_90_percent_s = self.elapsed_time_s
                if self.time_at_10_percent_s is not None:
                    rise_time_s = self.time_at_90_percent_s - self.time_at_10_percent_s
                    self.lbl_rise_time.setText(f"Temps de montée (10-90%) : {rise_time_s:.3f} s")

        self.buffer_time.pop(0)
        self.buffer_time.append(self.elapsed_time_s)
        self.buffer_measured_rpm.pop(0)
        self.buffer_measured_rpm.append(new_speed_rpm)
        self.buffer_target_rpm.pop(0)
        self.buffer_target_rpm.append(self.target_speed_rpm)

        self._refresh_plot_curves()

    ## @brief Pousse les buffers de données mis à jour vers l'IHM PyQtGraph.
    def _refresh_plot_curves(self):
        self.curve_measure.setData(self.buffer_time, self.buffer_measured_rpm)
        self.curve_target.setData(self.buffer_time, self.buffer_target_rpm)


## @brief Point d'entrée de l'application.
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    supervision_dashboard = IndustrialSupervisionDashboard()
    supervision_dashboard.show()
    sys.exit(app.exec())
