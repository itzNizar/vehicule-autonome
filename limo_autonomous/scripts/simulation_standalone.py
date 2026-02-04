#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Simulation standalone du vehicule Agilex Limo en mode Ackermann
Validation de la loi de commande en coordonnees polaires
Compatible Python 2.7

Auteur: Bureau d'etude ENSEM
Date: Fevrier 2026
"""

from __future__ import print_function, division
import math

# Essayer d'importer matplotlib (optionnel)
try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrow, Rectangle
    import matplotlib.animation as animation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib non disponible, mode texte uniquement")

# =============================================================================
# PARAMETRES DU ROBOT LIMO
# =============================================================================
L = 0.2          # Empattement (m)
V_MAX = 1.0      # Vitesse max (m/s)
GAMMA_MAX = 0.46 # Angle de braquage max (rad) = 26.6 deg

# =============================================================================
# GAINS DU CONTROLEUR
# Conditions de stabilite: k_rho > 0, k_alpha > k_rho, k_beta < 0
# =============================================================================
K_RHO = 1.0      # Gain sur la distance
K_ALPHA = 3.0    # Gain sur l'angle vers la cible
K_BETA = -0.5    # Gain sur l'orientation finale


def normalize_angle(angle):
    """Normalise un angle dans l'intervalle [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(value, min_val, max_val):
    """Limite une valeur entre min et max"""
    return max(min_val, min(max_val, value))


def cartesian_to_polar(x_r, y_r, theta_r, x_star, y_star, theta_star):
    """
    Conversion des coordonnees cartesiennes vers polaires
    
    Entrees:
        x_r, y_r, theta_r: Position et orientation actuelles du robot
        x_star, y_star, theta_star: Position et orientation cibles
        
    Sorties:
        rho: Distance a la cible
        alpha: Angle vers la cible par rapport a l'orientation du robot
        beta: Orientation finale desiree
    """
    dx = x_star - x_r
    dy = y_star - y_r
    
    rho = math.sqrt(dx**2 + dy**2)
    
    # Angle de la droite robot->cible par rapport a l'axe x
    angle_to_target = math.atan2(dy, dx)
    
    # Alpha: angle entre l'orientation du robot et la direction vers la cible
    alpha = normalize_angle(angle_to_target - theta_r)
    
    # Beta: erreur d'orientation finale
    beta = normalize_angle(theta_star - angle_to_target)
    
    return rho, alpha, beta


def control_law(rho, alpha, beta, k_rho=K_RHO, k_alpha=K_ALPHA, k_beta=K_BETA):
    """
    Loi de commande en coordonnees polaires
    
    v = k_rho * rho
    w = k_alpha * alpha + k_beta * beta
    gamma = atan(w * L / v)  pour v != 0
    
    Retourne: v (vitesse), gamma (angle de braquage)
    """
    # Vitesse longitudinale proportionnelle a la distance
    v = k_rho * rho
    
    # Entree virtuelle pour l'orientation
    w = k_alpha * alpha + k_beta * beta
    
    # Gestion du cas ou le robot est derriere la cible (alpha > pi/2)
    # Dans ce cas, on recule
    if abs(alpha) > math.pi / 2:
        v = -v
        # Ajuster alpha pour le cas marche arriere
        if alpha > 0:
            alpha = alpha - math.pi
        else:
            alpha = alpha + math.pi
        w = k_alpha * alpha + k_beta * beta
    
    # Calcul de l'angle de braquage
    if abs(v) > 1e-6:
        gamma = math.atan(w * L / v)
    else:
        gamma = 0.0
    
    # Saturations
    v = clamp(v, -V_MAX, V_MAX)
    gamma = clamp(gamma, -GAMMA_MAX, GAMMA_MAX)
    
    return v, gamma


def bicycle_model(state, v, gamma, L=L):
    """
    Modele bicyclette - Equations dynamiques
    
    dx/dt = v * cos(theta)
    dy/dt = v * sin(theta)
    dtheta/dt = (v / L) * tan(gamma)
    """
    x, y, theta = state
    
    dx = v * math.cos(theta)
    dy = v * math.sin(theta)
    dtheta = (v / L) * math.tan(gamma)
    
    return [dx, dy, dtheta]


def simulate(x0, y0, theta0, x_star, y_star, theta_star, 
             dt=0.05, t_max=30.0, threshold=0.02):
    """
    Simulation complete de la trajectoire
    
    Entrees:
        x0, y0, theta0: Position et orientation initiales
        x_star, y_star, theta_star: Position et orientation cibles
        dt: Pas de temps (s)
        t_max: Temps max de simulation (s)
        threshold: Seuil de convergence pour rho (m)
        
    Sorties:
        t_hist: Historique du temps
        state_hist: Historique des etats [x, y, theta]
        control_hist: Historique des commandes [v, gamma]
        polar_hist: Historique des coordonnees polaires [rho, alpha, beta]
    """
    # Initialisation
    state = [x0, y0, theta0]
    t = 0.0
    
    # Historiques
    t_hist = []
    state_hist = []
    control_hist = []
    polar_hist = []
    
    while t < t_max:
        # Conversion en coordonnees polaires
        rho, alpha, beta = cartesian_to_polar(
            state[0], state[1], state[2],
            x_star, y_star, theta_star
        )
        
        t_hist.append(t)
        state_hist.append(list(state))
        polar_hist.append([rho, alpha, beta])
        
        # Verification de la convergence
        if rho < threshold:
            print("Convergence atteinte en t = {:.2f}s".format(t))
            print("  Position finale: ({:.4f}, {:.4f})".format(state[0], state[1]))
            print("  Orientation finale: {:.2f} deg".format(math.degrees(state[2])))
            print("  Erreur de position: {:.4f} m".format(rho))
            print("  Erreur d'orientation: {:.2f} deg".format(
                math.degrees(abs(normalize_angle(state[2] - theta_star)))))
            break
        
        # Loi de commande
        v, gamma = control_law(rho, alpha, beta)
        control_hist.append([v, gamma])
        
        # Integration (Euler)
        dstate = bicycle_model(state, v, gamma)
        state[0] = state[0] + dstate[0] * dt
        state[1] = state[1] + dstate[1] * dt
        state[2] = normalize_angle(state[2] + dstate[2] * dt)
        
        t += dt
    
    return t_hist, state_hist, control_hist, polar_hist


def plot_results(t_hist, state_hist, control_hist, polar_hist,
                 x_star, y_star, theta_star, title=""):
    """
    Affichage des resultats de simulation
    """
    if not HAS_MATPLOTLIB:
        print("Matplotlib non disponible - pas de graphique")
        return None
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('Simulation Limo - Mode Ackermann {}'.format(title), fontsize=14)
    
    # Convertir en listes pour indexation
    state_x = [s[0] for s in state_hist]
    state_y = [s[1] for s in state_hist]
    state_theta = [s[2] for s in state_hist]
    
    polar_rho = [p[0] for p in polar_hist]
    polar_alpha = [p[1] for p in polar_hist]
    polar_beta = [p[2] for p in polar_hist]
    
    ctrl_v = [c[0] for c in control_hist]
    ctrl_gamma = [c[1] for c in control_hist]
    
    # 1. Trajectoire dans le plan (x, y)
    ax1 = axes[0][0]
    ax1.plot(state_x, state_y, 'b-', linewidth=2, label='Trajectoire')
    ax1.plot(state_x[0], state_y[0], 'go', markersize=10, label='Depart')
    ax1.plot(x_star, y_star, 'r*', markersize=15, label='Cible')
    
    # Fleches orientation
    arrow_len = 0.2
    ax1.arrow(state_x[0], state_y[0],
              arrow_len * math.cos(state_theta[0]),
              arrow_len * math.sin(state_theta[0]),
              head_width=0.05, color='green')
    ax1.arrow(x_star, y_star,
              arrow_len * math.cos(theta_star),
              arrow_len * math.sin(theta_star),
              head_width=0.05, color='red')
    
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_title('Trajectoire')
    ax1.legend()
    ax1.axis('equal')
    ax1.grid(True)
    
    # 2. Evolution de la position (x, y) dans le temps
    ax2 = axes[0][1]
    ax2.plot(t_hist, state_x, 'r-', label='x')
    ax2.plot(t_hist, state_y, 'b-', label='y')
    ax2.axhline(y=x_star, color='r', linestyle='--', alpha=0.5)
    ax2.axhline(y=y_star, color='b', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Temps (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Temps')
    ax2.legend()
    ax2.grid(True)
    
    # 3. Evolution de l'orientation
    ax3 = axes[0][2]
    theta_deg = [math.degrees(t) for t in state_theta]
    ax3.plot(t_hist, theta_deg, 'g-', linewidth=2)
    ax3.axhline(y=math.degrees(theta_star), color='r', linestyle='--', 
                label='Cible: {:.1f} deg'.format(math.degrees(theta_star)))
    ax3.set_xlabel('Temps (s)')
    ax3.set_ylabel('Orientation theta (deg)')
    ax3.set_title('Orientation vs Temps')
    ax3.legend()
    ax3.grid(True)
    
    # 4. Coordonnees polaires - distance
    ax4 = axes[1][0]
    ax4.plot(t_hist, polar_rho, 'b-', label='rho (distance)')
    ax4.set_xlabel('Temps (s)')
    ax4.set_ylabel('rho (m)')
    ax4.set_title('Distance a la cible')
    ax4.legend()
    ax4.grid(True)
    
    # 5. Angles polaires
    ax5 = axes[1][1]
    alpha_deg = [math.degrees(a) for a in polar_alpha]
    beta_deg = [math.degrees(b) for b in polar_beta]
    ax5.plot(t_hist, alpha_deg, 'r-', label='alpha')
    ax5.plot(t_hist, beta_deg, 'g-', label='beta')
    ax5.set_xlabel('Temps (s)')
    ax5.set_ylabel('Angle (deg)')
    ax5.set_title('Angles polaires alpha et beta')
    ax5.legend()
    ax5.grid(True)
    
    # 6. Commandes
    ax6 = axes[1][2]
    t_ctrl = t_hist[:len(control_hist)]
    gamma_deg = [math.degrees(g) for g in ctrl_gamma]
    ax6.plot(t_ctrl, ctrl_v, 'b-', label='v (m/s)')
    ax6.plot(t_ctrl, gamma_deg, 'r-', label='gamma (deg)')
    ax6.axhline(y=V_MAX, color='b', linestyle='--', alpha=0.3)
    ax6.axhline(y=-V_MAX, color='b', linestyle='--', alpha=0.3)
    ax6.axhline(y=math.degrees(GAMMA_MAX), color='r', linestyle='--', alpha=0.3)
    ax6.axhline(y=-math.degrees(GAMMA_MAX), color='r', linestyle='--', alpha=0.3)
    ax6.set_xlabel('Temps (s)')
    ax6.set_ylabel('Commande')
    ax6.set_title('Commandes v et gamma')
    ax6.legend()
    ax6.grid(True)
    
    plt.tight_layout()
    return fig


def run_test_scenarios():
    """
    Execute plusieurs scenarios de test
    """
    scenarios = [
        # (x0, y0, theta0, x*, y*, theta*, description)
        (0, 0, 0, 2, 0, 0, "Ligne droite"),
        (0, 0, 0, 2, 2, math.pi/2, "Diagonale avec rotation"),
        (0, 0, 0, -1, 1, math.pi, "Cible derriere"),
        (1, 1, math.pi/4, 0, 0, 0, "Retour a l'origine"),
        (0, 0, 0, 1, 0, math.pi, "Position proche, demi-tour"),
    ]
    
    print("=" * 60)
    print("SIMULATION DU ROBOT LIMO - MODE ACKERMANN")
    print("=" * 60)
    print("")
    print("Parametres du robot:")
    print("  Empattement L = {} m".format(L))
    print("  Vitesse max = {} m/s".format(V_MAX))
    print("  Angle braquage max = {:.1f} deg".format(math.degrees(GAMMA_MAX)))
    print("")
    print("Gains du controleur:")
    print("  k_rho = {}".format(K_RHO))
    print("  k_alpha = {}".format(K_ALPHA))
    print("  k_beta = {}".format(K_BETA))
    print("")
    print("Verification stabilite:")
    print("  k_rho > 0: {} {}".format(K_RHO > 0, "OK" if K_RHO > 0 else "ERREUR"))
    print("  k_alpha > k_rho: {} {}".format(K_ALPHA > K_RHO, "OK" if K_ALPHA > K_RHO else "ERREUR"))
    print("  k_beta < 0: {} {}".format(K_BETA < 0, "OK" if K_BETA < 0 else "ERREUR"))
    
    figs = []
    
    for i, (x0, y0, theta0, x_star, y_star, theta_star, desc) in enumerate(scenarios):
        print("")
        print("=" * 60)
        print("Scenario {}: {}".format(i+1, desc))
        print("  Depart: ({}, {}), theta = {:.1f} deg".format(x0, y0, math.degrees(theta0)))
        print("  Cible:  ({}, {}), theta* = {:.1f} deg".format(x_star, y_star, math.degrees(theta_star)))
        print("-" * 60)
        
        t_hist, state_hist, control_hist, polar_hist = simulate(
            x0, y0, theta0, x_star, y_star, theta_star
        )
        
        if HAS_MATPLOTLIB:
            fig = plot_results(
                t_hist, state_hist, control_hist, polar_hist,
                x_star, y_star, theta_star,
                title="- Scenario {}: {}".format(i+1, desc)
            )
            if fig:
                figs.append(fig)
    
    if HAS_MATPLOTLIB:
        plt.show()
    
    return figs


if __name__ == "__main__":
    print("")
    print("=" * 60)
    print("LIMO - SIMULATION DE LA LOI DE COMMANDE")
    print("=" * 60)
    
    # Executer les scenarios de test
    run_test_scenarios()
