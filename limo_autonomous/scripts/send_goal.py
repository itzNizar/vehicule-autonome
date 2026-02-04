#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Script utilitaire pour commander le robot Limo
Compatible Python 2.7 (ROS Melodic)

Usage:
    python send_goal.py <x> <y> <theta_deg>
    python send_goal.py --start
    python send_goal.py --stop

Auteur: Bureau d'etude ENSEM
Date: Fevrier 2026
"""

from __future__ import print_function, division
import sys
import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64


def euler_to_quaternion(roll, pitch, yaw):
    """Conversion angles d'Euler vers quaternion"""
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


def quaternion_to_euler(x, y, z, w):
    """Conversion quaternion vers angles d'Euler"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return 0, 0, yaw


class LimoCommander:
    """Classe pour commander le robot Limo"""
    
    def __init__(self):
        rospy.init_node('limo_commander', anonymous=True)
        
        # Publishers
        self.goal_pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)
        self.enable_pub = rospy.Publisher('/controller/enable', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Etat actuel
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.controller_state = "UNKNOWN"
        self.distance = 0.0
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self._odom_callback)
        rospy.Subscriber('/controller/state', String, self._state_callback)
        rospy.Subscriber('/controller/distance', Float64, self._distance_callback)
        
        # Attendre les connexions
        rospy.sleep(0.5)
    
    def _odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(o.x, o.y, o.z, o.w)
        self.current_theta = yaw
    
    def _state_callback(self, msg):
        self.controller_state = msg.data
    
    def _distance_callback(self, msg):
        self.distance = msg.data
    
    def send_goal(self, x, y, theta_deg):
        """Envoie une cible au controleur"""
        theta = math.radians(theta_deg)
        
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        qx, qy, qz, qw = euler_to_quaternion(0, 0, theta)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        
        self.goal_pub.publish(msg)
        print("Cible envoyee: ({:.2f}, {:.2f}), theta*={:.1f} deg".format(x, y, theta_deg))
    
    def start(self):
        """Active le controleur"""
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)
        print("Controleur ACTIVE")
    
    def stop(self):
        """Desactive le controleur"""
        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)
        
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        print("Controleur DESACTIVE - Robot arrete")
    
    def emergency_stop(self):
        """Arret d'urgence"""
        self.stop()
        cmd = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)
        print("ARRET D'URGENCE")
    
    def status(self):
        """Affiche l'etat actuel"""
        rospy.sleep(0.5)
        
        print("")
        print("=" * 50)
        print("ETAT DU ROBOT LIMO")
        print("=" * 50)
        print("Position:    ({:.3f}, {:.3f}) m".format(self.current_x, self.current_y))
        print("Orientation: {:.1f} deg".format(math.degrees(self.current_theta)))
        print("Etat:        {}".format(self.controller_state))
        print("Distance:    {:.3f} m".format(self.distance))
        print("=" * 50)
        print("")
    
    def wait_for_goal(self, timeout=60.0):
        """Attend que la cible soit atteinte"""
        print("En attente de l'arrivee...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            if self.controller_state == "GOAL_REACHED":
                print("Cible atteinte en {:.1f}s".format(elapsed))
                return True
            
            if elapsed > timeout:
                print("Timeout apres {}s".format(timeout))
                return False
            
            rate.sleep()
        
        return False


def print_usage():
    print("")
    print("Usage:")
    print("  python send_goal.py <x> <y> <theta_deg>  - Envoyer une cible")
    print("  python send_goal.py --start              - Activer le controleur")
    print("  python send_goal.py --stop               - Desactiver le controleur")
    print("  python send_goal.py --status             - Afficher l'etat")
    print("  python send_goal.py --emergency          - Arret d'urgence")
    print("")
    print("Exemples:")
    print("  python send_goal.py 1.0 0.0 0            - Aller a (1, 0) avec theta=0")
    print("  python send_goal.py 2.0 1.0 45           - Aller a (2, 1) avec theta=45 deg")
    print("")


def main():
    args = sys.argv[1:]
    
    if len(args) == 0:
        print_usage()
        return
    
    try:
        commander = LimoCommander()
        
        if args[0] == '--start':
            commander.start()
        
        elif args[0] == '--stop':
            commander.stop()
        
        elif args[0] == '--status':
            commander.status()
        
        elif args[0] == '--emergency':
            commander.emergency_stop()
        
        elif args[0] == '--wait':
            commander.wait_for_goal()
        
        elif len(args) >= 3:
            x = float(args[0])
            y = float(args[1])
            theta = float(args[2])
            commander.send_goal(x, y, theta)
            
            if len(args) > 3 and args[3] == '--wait':
                commander.wait_for_goal()
        
        else:
            print_usage()
            commander.status()
    
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nInterrompu")


if __name__ == '__main__':
    main()
