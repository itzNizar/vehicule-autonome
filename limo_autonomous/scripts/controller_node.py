#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Noeud ROS1 - Controleur du vehicule Agilex Limo
Compatible Python 2.7 (ROS Melodic)

Auteur: Bureau d'etude ENSEM
Date: Fevrier 2026
"""

from __future__ import print_function, division
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64

# Pour la conversion quaternion -> euler sans tf
def quaternion_to_euler(x, y, z, w):
    """
    Conversion quaternion vers angles d'Euler (roll, pitch, yaw)
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """
    Conversion angles d'Euler vers quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return x, y, z, w


def normalize_angle(angle):
    """Normalise un angle dans [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(value, min_val, max_val):
    """Limite une valeur entre min et max"""
    return max(min_val, min(max_val, value))


class LimoController:
    """
    Controleur pour le robot Limo en mode Ackermann
    Utilise une loi de commande en coordonnees polaires
    """
    
    def __init__(self):
        # Initialisation du noeud ROS
        rospy.init_node('limo_controller', anonymous=False)
        
        # Parametres du robot
        self.L = rospy.get_param('~wheelbase', 0.2)
        self.v_max = rospy.get_param('~v_max', 0.5)
        self.gamma_max = rospy.get_param('~gamma_max', 0.46)
        
        # Gains du controleur
        self.k_rho = rospy.get_param('~k_rho', 1.0)
        self.k_alpha = rospy.get_param('~k_alpha', 3.0)
        self.k_beta = rospy.get_param('~k_beta', -0.5)
        
        # Seuils
        self.position_threshold = rospy.get_param('~position_threshold', 0.05)
        self.angle_threshold = rospy.get_param('~angle_threshold', 0.1)
        
        # Frequence de controle
        self.control_rate = rospy.get_param('~control_rate', 20)
        
        # Variables d'etat
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        
        self.enabled = False
        self.goal_received = False
        self.goal_reached = False
        self.obstacle_blocked = False   # mis à True par obstacle_avoidance_node
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state_pub = rospy.Publisher('/controller/state', String, queue_size=10)
        self.polar_pub = rospy.Publisher('/controller/polar_coords', Vector3, queue_size=10)
        self.distance_pub = rospy.Publisher('/controller/distance', Float64, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/goal_pose', PoseStamped, self.goal_callback)
        rospy.Subscriber('/controller/enable', Bool, self.enable_callback)
        rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_callback)
        
        # Log
        rospy.loginfo("=" * 50)
        rospy.loginfo("LIMO CONTROLLER - INITIALIZED")
        rospy.loginfo("=" * 50)
        rospy.loginfo("Wheelbase L: {} m".format(self.L))
        rospy.loginfo("Max velocity: {} m/s".format(self.v_max))
        rospy.loginfo("Max steering angle: {:.1f} deg".format(math.degrees(self.gamma_max)))
        rospy.loginfo("Gains: k_rho={}, k_alpha={}, k_beta={}".format(
            self.k_rho, self.k_alpha, self.k_beta))
        rospy.loginfo("=" * 50)
        
        self._check_stability_conditions()
    
    def _check_stability_conditions(self):
        """Verifie les conditions de stabilite"""
        stable = True
        
        if self.k_rho <= 0:
            rospy.logwarn("STABILITE: k_rho doit etre > 0")
            stable = False
        
        if self.k_alpha <= self.k_rho:
            rospy.logwarn("STABILITE: k_alpha doit etre > k_rho")
            stable = False
        
        if self.k_beta >= 0:
            rospy.logwarn("STABILITE: k_beta doit etre < 0")
            stable = False
        
        if stable:
            rospy.loginfo("Conditions de stabilite verifiees OK")
    
    def odom_callback(self, msg):
        """Callback pour l'odometrie"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Conversion quaternion -> yaw
        o = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(o.x, o.y, o.z, o.w)
        self.current_theta = yaw
    
    def goal_callback(self, msg):
        """Callback pour la position cible"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # Conversion quaternion -> yaw
        o = msg.pose.orientation
        _, _, yaw = quaternion_to_euler(o.x, o.y, o.z, o.w)
        self.goal_theta = yaw
        
        self.goal_received = True
        self.goal_reached = False
        
        rospy.loginfo("Nouvelle cible: ({:.2f}, {:.2f}), theta*={:.1f} deg".format(
            self.goal_x, self.goal_y, math.degrees(self.goal_theta)))
    
    def enable_callback(self, msg):
        """Callback pour l'activation"""
        self.enabled = msg.data
        if self.enabled:
            rospy.loginfo("Controleur ACTIVE")
        else:
            rospy.loginfo("Controleur DESACTIVE")
            self.stop()
    
    def obstacle_callback(self, msg):
        """Callback obstacle_avoidance_node – bloque la commande si True"""
        if msg.data and not self.obstacle_blocked:
            rospy.logwarn("Controleur BLOQUE par détection obstacle")
        self.obstacle_blocked = msg.data

    def cartesian_to_polar(self):
        """Conversion en coordonnees polaires"""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        
        rho = math.sqrt(dx**2 + dy**2)
        
        angle_to_target = math.atan2(dy, dx)
        alpha = normalize_angle(angle_to_target - self.current_theta)
        beta = normalize_angle(self.goal_theta - angle_to_target)
        
        return rho, alpha, beta
    
    def compute_control(self, rho, alpha, beta):
        """Calcule les commandes v et gamma"""
        # Vitesse proportionnelle a la distance
        v = self.k_rho * rho
        
        # Entree virtuelle pour l'orientation
        w = self.k_alpha * alpha + self.k_beta * beta
        
        # Gestion de la marche arriere
        backward = False
        if abs(alpha) > math.pi / 2:
            v = -v
            backward = True
            if alpha > 0:
                alpha_adj = alpha - math.pi
            else:
                alpha_adj = alpha + math.pi
            w = self.k_alpha * alpha_adj + self.k_beta * beta
        
        # Calcul de l'angle de braquage
        if abs(v) > 1e-6:
            gamma = math.atan(w * self.L / v)
        else:
            gamma = 0.0
        
        # Saturations
        v = clamp(v, -self.v_max, self.v_max)
        gamma = clamp(gamma, -self.gamma_max, self.gamma_max)
        
        return v, gamma, backward
    
    def stop(self):
        """Arrete le robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def publish_state(self, state_str):
        """Publie l'etat du controleur"""
        msg = String()
        msg.data = state_str
        self.state_pub.publish(msg)
    
    def run(self):
        """Boucle principale"""
        rate = rospy.Rate(self.control_rate)
        
        while not rospy.is_shutdown():
            # Verification des conditions
            if not self.enabled:
                self.publish_state("DISABLED")
                rate.sleep()
                continue
            
            if not self.goal_received or self.goal_x is None:
                self.publish_state("WAITING_FOR_GOAL")
                rate.sleep()
                continue

            # Guard : si un obstacle est détecté, on s'arrête sans publier
            # de nouvelle commande – obstacle_avoidance_node gère le cmd_vel.
            if self.obstacle_blocked:
                self.publish_state("OBSTACLE_DETECTED")
                rate.sleep()
                continue
            
            # Calcul des coordonnees polaires
            rho, alpha, beta = self.cartesian_to_polar()
            
            # Publication des coordonnees polaires
            polar_msg = Vector3()
            polar_msg.x = rho
            polar_msg.y = alpha
            polar_msg.z = beta
            self.polar_pub.publish(polar_msg)
            
            # Publication de la distance
            dist_msg = Float64()
            dist_msg.data = rho
            self.distance_pub.publish(dist_msg)
            
            # Verification de l'arrivee
            angle_error = abs(normalize_angle(self.current_theta - self.goal_theta))
            
            if rho < self.position_threshold and angle_error < self.angle_threshold:
                if not self.goal_reached:
                    rospy.loginfo("CIBLE ATTEINTE!")
                    rospy.loginfo("  Position: ({:.3f}, {:.3f})".format(
                        self.current_x, self.current_y))
                    rospy.loginfo("  Orientation: {:.1f} deg".format(
                        math.degrees(self.current_theta)))
                    rospy.loginfo("  Erreur position: {:.3f} m".format(rho))
                    rospy.loginfo("  Erreur orientation: {:.1f} deg".format(
                        math.degrees(angle_error)))
                    self.goal_reached = True
                
                self.stop()
                self.publish_state("GOAL_REACHED")
                rate.sleep()
                continue
            
            # Phase finale: orientation seule
            if rho < self.position_threshold and angle_error >= self.angle_threshold:
                self.publish_state("FINAL_ROTATION")
                cmd = Twist()
                cmd.linear.x = 0.0
                angle_diff = normalize_angle(self.goal_theta - self.current_theta)
                cmd.angular.z = clamp(2.0 * angle_diff, -0.5, 0.5)
                self.cmd_vel_pub.publish(cmd)
                rate.sleep()
                continue
            
            # Calcul de la commande
            v, gamma, backward = self.compute_control(rho, alpha, beta)
            
            # Creation du message Twist
            cmd = Twist()
            cmd.linear.x = v
            
            # Conversion gamma -> angular.z
            if abs(v) > 1e-6:
                cmd.angular.z = (v / self.L) * math.tan(gamma)
            else:
                cmd.angular.z = 0.0
            
            self.cmd_vel_pub.publish(cmd)
            
            # Etat
            if backward:
                self.publish_state("MOVING_BACKWARD")
            else:
                self.publish_state("MOVING_FORWARD")
            
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = LimoController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
