#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Noeud ROS1 - Détection et évitement d'obstacles via LiDAR
Compatible Python 2.7 (ROS Melodic)

Rôle :
  - Subscribes au topic /scan (sensor_msgs/LaserScan) publié par le LiDAR du Limo.
  - Découpe le champ de vision en trois secteurs : gauche, centre, droite.
  - Si un obstacle est détecté dans le secteur centre en dessous du seuil de
    distance d'arrêt, publie un message sur /obstacle_detected et force
    /cmd_vel à zéro (arrêt d'urgence).
  - Publie les distances minimales par secteur sur /obstacle/min_distances
    pour visualisation en rqt_plot.

Paramètres (lancé via launch ou rosparam) :
  ~stop_distance   : distance d'arrêt (m)        [0.3]
  ~slow_distance   : distance de ralentissement   [0.6]
  ~fov_center_deg  : demi-angle du secteur central (deg) [30]
  ~control_rate    : fréquence de la boucle (Hz)  [20]

Auteur : Bureau d'etude ENSEM
Date   : Février 2026
"""

from __future__ import print_function, division
import rospy
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Vector3   # réutilisé pour [left, center, right]


# ---------------------------------------------------------------------------
# Constantes par défaut
# ---------------------------------------------------------------------------
DEFAULT_STOP_DISTANCE  = 0.30   # m – arrêt immédiat si obstacle < cette distance
DEFAULT_SLOW_DISTANCE  = 0.60   # m – ralentissement si obstacle < cette distance
DEFAULT_FOV_CENTER_DEG = 30.0   # degrés – demi-angle du secteur "centre"
DEFAULT_CONTROL_RATE   = 20     # Hz


class ObstacleAvoidanceNode(object):
    """
    Noeud principal de détection / évitement d'obstacles.

    Algorithme simplifié :
      1. On reçoit le LaserScan brut.
      2. On filtre les mesures invalides (inf, NaN, 0, hors max_range).
      3. On découpe en trois secteurs selon les angles.
      4. On calcule la distance minimale par secteur.
      5. Si min_center < stop_distance  → arrêt + publication d'alerte.
         Si min_center < slow_distance  → signal de ralentissement.
    """

    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=False)

        # ---- Paramètres --------------------------------------------------------
        self.stop_distance  = rospy.get_param('~stop_distance',  DEFAULT_STOP_DISTANCE)
        self.slow_distance  = rospy.get_param('~slow_distance',  DEFAULT_SLOW_DISTANCE)
        self.fov_center_deg = rospy.get_param('~fov_center_deg', DEFAULT_FOV_CENTER_DEG)
        self.control_rate   = rospy.get_param('~control_rate',   DEFAULT_CONTROL_RATE)

        self.fov_center_rad = math.radians(self.fov_center_deg)

        # ---- État interne -------------------------------------------------------
        self.obstacle_detected   = False       # True si arrêt d'urgence actif
        self.min_distances       = [99.0, 99.0, 99.0]  # [gauche, centre, droite]
        self.last_scan_received  = False

        # ---- Publishers ---------------------------------------------------------
        # Boolean : True si un obstacle bloquant est détecté
        self.obstacle_pub = rospy.Publisher(
            '/obstacle_detected', Bool, queue_size=10)

        # Float64 : distance minimale centre (pour rqt_plot)
        self.min_dist_center_pub = rospy.Publisher(
            '/obstacle/min_dist_center', Float64, queue_size=10)
        self.min_dist_left_pub   = rospy.Publisher(
            '/obstacle/min_dist_left',  Float64, queue_size=10)
        self.min_dist_right_pub  = rospy.Publisher(
            '/obstacle/min_dist_right', Float64, queue_size=10)

        # String : etat textuel pour le superviseur
        self.status_pub = rospy.Publisher(
            '/obstacle/status', String, queue_size=10)

        # cmd_vel : utilisé UNIQUEMENT pour forcer l'arrêt
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # ---- Subscribers --------------------------------------------------------
        rospy.Subscriber('/scan', LaserScan, self._scan_callback)

        # ---- Log ----------------------------------------------------------------
        rospy.loginfo("=" * 55)
        rospy.loginfo("OBSTACLE AVOIDANCE NODE - INITIALISE")
        rospy.loginfo("=" * 55)
        rospy.loginfo("  stop_distance  : {:.2f} m".format(self.stop_distance))
        rospy.loginfo("  slow_distance  : {:.2f} m".format(self.slow_distance))
        rospy.loginfo("  fov_center     : +/- {:.0f} deg".format(self.fov_center_deg))
        rospy.loginfo("=" * 55)

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------
    def _scan_callback(self, scan):
        """
        Traitement d'un message LaserScan.

        Le LiDAR Limo publie typiquement 360 points de -pi à +pi.
        angle_min, angle_max et angle_increment sont fournis dans le message.
        """
        self.last_scan_received = True

        angle_min = scan.angle_min          # rad
        angle_inc = scan.angle_increment    # rad
        ranges   = scan.ranges             # liste de distances (m)
        range_max = scan.range_max

        # Pré-filtrage : on remplace les valeurs invalides par +inf
        filtered = []
        for r in ranges:
            if r <= 0.0 or r >= range_max or math.isnan(r) or math.isinf(r):
                filtered.append(float('inf'))
            else:
                filtered.append(r)

        # Découpage en secteurs
        left_min   = float('inf')
        center_min = float('inf')
        right_min  = float('inf')

        for i, dist in enumerate(filtered):
            angle = angle_min + i * angle_inc   # angle en rad, référence robot

            # Normaliser dans [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            if -self.fov_center_rad <= angle <= self.fov_center_rad:
                # Secteur centre (droit devant)
                if dist < center_min:
                    center_min = dist
            elif angle > self.fov_center_rad:
                # Secteur gauche
                if dist < left_min:
                    left_min = dist
            else:
                # Secteur droite
                if dist < right_min:
                    right_min = dist

        self.min_distances = [left_min, center_min, right_min]

    # ------------------------------------------------------------------
    # Boucle principale
    # ------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            if not self.last_scan_received:
                rate.sleep()
                continue

            left, center, right = self.min_distances

            # --- Publication des distances (rqt_plot) -----------------------
            self.min_dist_left_pub.publish(Float64(data=left   if left   != float('inf') else 9.9))
            self.min_dist_center_pub.publish(Float64(data=center if center != float('inf') else 9.9))
            self.min_dist_right_pub.publish(Float64(data=right  if right  != float('inf') else 9.9))

            # --- Décision d'arrêt -----------------------------------------------
            if center < self.stop_distance:
                # ARRÊT D'URGENCE – obstacle en dessous du seuil critique
                if not self.obstacle_detected:
                    rospy.logwarn("OBSTACLE DÉTECTÉ à {:.2f} m (centre) – ARRÊT".format(center))

                self.obstacle_detected = True

                # Forcer cmd_vel à zéro
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)

                self.obstacle_pub.publish(Bool(data=True))
                self.status_pub.publish(String(data="OBSTACLE_STOP"))

            elif center < self.slow_distance:
                # Zone de ralentissement – on segnale mais on ne force pas l'arrêt
                self.obstacle_detected = False
                self.obstacle_pub.publish(Bool(data=False))
                self.status_pub.publish(String(data="OBSTACLE_SLOW"))

            else:
                # Voie libre
                self.obstacle_detected = False
                self.obstacle_pub.publish(Bool(data=False))
                self.status_pub.publish(String(data="CLEAR"))

            rate.sleep()


# ---------------------------------------------------------------------------
# Entry-point
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        node = ObstacleAvoidanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
