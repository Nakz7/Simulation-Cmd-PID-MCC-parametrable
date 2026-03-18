#!/usr/bin/env python3
# -*- coding: utf-8 -*-

## @file simulation_moteur_industriel.py
#  @brief Application de supervision industrielle pour la simulation d'un moteur CC asservi.
#  @details Ce module implémente une interface permettant de paramétrer la
#  physique du moteur, de calculer ses limites théoriques (saturation) et de simuler
#  son asservissement via un régulateur PID en temps réel.
#  @author Neil Legendre-Ferreira Da Costa
#  @date 2026-03-18

import sys
import numpy as np
import pyqtgraph as pg
from PyQt6 import QtWidgets, QtCore

## @class DcMotorPhysicalModel
#  @brief Modèle mathématique et physique d'un moteur à courant continu.
#  @details Résout les équations différentielles electromechanical couplées
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


## @class IndustrialSupervisionDashboard
#  @brief IHM principale intégrant un système d'onglets pour la supervision.
class IndustrialSupervisionDashboard(QtWidgets.QMainWindow):
    ## @brief Initialise le dashboard, les modèles et l'architecture UI.
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Supervision Industrielle - Asservissement & Paramétrage")
        self.resize(1100, 650)

        # Modèles métiers
        self.motor_model = DcMotorPhysicalModel()
        self.pid_controller = PidControllerIndustrial(gain_p=135.0, gain_i=110.0, gain_d=12.0)

        # Paramètres temporels globaux
        self.time_step_s = 0.01
        self.elapsed_time_s = 0.0

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

        self._populate_simulation_tab()
        self._populate_physics_tab()

        self.tab_manager.addTab(self.tab_simulation, "Simulation & Contrôle PID")
        self.tab_manager.addTab(self.tab_physics, "Paramétrage Physique & Limites")

        main_layout.addWidget(self.tab_manager)

    ## @brief Construit l'interface de l'onglet de Simulation (Boutons, Graphes).
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

        # Assemblage du panneau
        control_panel.addWidget(self.btn_run_simulation)
        control_panel.addWidget(self.btn_pause_simulation)
        control_panel.addWidget(self.btn_reset_states)
        control_panel.addSpacing(20)
        control_panel.addWidget(self.checkbox_p)
        control_panel.addWidget(self.checkbox_i)
        control_panel.addWidget(self.checkbox_d)
        control_panel.addSpacing(20)
        control_panel.addWidget(self.lbl_target_display)
        control_panel.addWidget(self.slider_target)
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

    ## @brief Construit l'interface de l'onglet des Paramètres Physiques.
    def _populate_physics_tab(self):
        layout = QtWidgets.QVBoxLayout(self.tab_physics)
        form_layout = QtWidgets.QFormLayout()

        # Helpers pour créer des champs numériques (QDoubleSpinBox) uniformes
        def create_spinbox(value, step, decimals, maximum=1000.0):
            spinbox = QtWidgets.QDoubleSpinBox()
            spinbox.setDecimals(decimals)
            spinbox.setSingleStep(step)
            spinbox.setRange(0.0001, maximum)
            spinbox.setValue(value)
            return spinbox

        self.spin_inertia_j = create_spinbox(0.01, 0.001, 4)
        self.spin_friction_b = create_spinbox(0.1, 0.01, 4)
        self.spin_constant_k = create_spinbox(0.01, 0.001, 4)
        self.spin_resistance_r = create_spinbox(1.0, 0.1, 2)
        self.spin_inductance_l = create_spinbox(0.5, 0.01, 3)
        self.spin_voltage_max = create_spinbox(24.0, 1.0, 1)  # Alimentation max (V)

        form_layout.addRow("Moment d'inertie J (kg.m²):", self.spin_inertia_j)
        form_layout.addRow("Frottement visqueux b (N.m.s):", self.spin_friction_b)
        form_layout.addRow("Constante K (V.s/rad):", self.spin_constant_k)
        form_layout.addRow("Résistance d'induit R (Ω):", self.spin_resistance_r)
        form_layout.addRow("Inductance d'induit L (H):", self.spin_inductance_l)
        form_layout.addRow("Tension d'Alimentation Max V (Volts):", self.spin_voltage_max)

        self.btn_apply_physics = QtWidgets.QPushButton("Appliquer les Paramètres & Calculer les Limites")
        self.btn_apply_physics.setStyleSheet("font-weight: bold; padding: 10px;")
        self.btn_apply_physics.clicked.connect(self.on_apply_physical_parameters)

        self.lbl_theoretical_max = QtWidgets.QLabel("Vitesse Maximale Théorique : -- rad/s (-- tr/min)")
        self.lbl_theoretical_max.setStyleSheet("font-size: 14px; color: #333; margin-top: 15px;")

        layout.addLayout(form_layout)
        layout.addWidget(self.btn_apply_physics)
        layout.addWidget(self.lbl_theoretical_max)
        layout.addStretch()

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

    ## @brief Callback pour appliquer les modifications physiques et calculer la limite.
    def on_apply_physical_parameters(self):
        # Récupération des données UI
        j = self.spin_inertia_j.value()
        b = self.spin_friction_b.value()
        k = self.spin_constant_k.value()
        r = self.spin_resistance_r.value()
        l = self.spin_inductance_l.value()
        v_max = self.spin_voltage_max.value()

        # Injection dans les modèles
        self.motor_model.update_physical_parameters(j, b, k, r, l)
        self.pid_controller.update_saturation_limits(v_max)

        self._calculate_theoretical_maximum()

    ## @brief Calcule et affiche la vitesse de saturation théorique du moteur.
    def _calculate_theoretical_maximum(self):
        b = self.motor_model.friction_b
        r = self.motor_model.resistance_r
        k = self.motor_model.constant_k
        v_max = self.pid_controller.saturation_max_v

        # omega_max = (V * K) / (R * b + K^2)
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

    def reset_simulation_states(self):
        self.simulation_timer.stop()
        self.motor_model.speed_rad_s = 0.0
        self.motor_model.current_amp = 0.0
        self.pid_controller.error_integral = 0.0
        self.pid_controller.previous_error = 0.0
        self.elapsed_time_s = 0.0

        self.buffer_time = [0.0] * self.max_display_points
        self.buffer_measured_rpm = [0.0] * self.max_display_points
        self.buffer_target_rpm = [0.0] * self.max_display_points
        self._refresh_plot_curves()

    ## @brief Cycle d'exécution principal appelé par le Timer.
    def execute_simulation_cycle(self):
        self.elapsed_time_s += self.time_step_s

        # Lecture du capteur
        measured_feedback_rad_s = self.motor_model.speed_rad_s

        # Calcul de commande avec état de l'IHM
        command_voltage_v = self.pid_controller.compute_control_effort(
            self.target_speed_rad_s,
            measured_feedback_rad_s,
            self.time_step_s,
            enable_p=self.checkbox_p.isChecked(),
            enable_i=self.checkbox_i.isChecked(),
            enable_d=self.checkbox_d.isChecked()
        )

        # Application au modèle physique
        new_speed_rad_s = self.motor_model.execute_simulation_step(command_voltage_v, self.time_step_s)
        new_speed_rpm = new_speed_rad_s * 60.0 / (2 * np.pi)

        # Mise à jour des FIFO
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