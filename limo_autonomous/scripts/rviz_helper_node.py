#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Noeud utilitaire RViz – Agilex Limo
Publié :
  /trajectory   (nav_msgs/Path)   – historique de la position du robot
  /goal_marker  (visualization_msgs/Marker) – étoile rouge sur la cible

Compatible Python 2.7 (ROS Melodic)
Auteur : Bureau d'etude ENSEM – Février 2026
"""

from __future__ import print_function, division
import rospy
import math

from nav_msgs.msg  import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header


def quaternion_to_euler_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RVizHelper(object):
    def __init__(self):
        rospy.init_node('rviz_helper', anonymous=False)

        # --- Données ---
        self.path_poses = []          # liste de PoseStamped
        self.goal_pose  = None        # PoseStamped dernière cible

        # --- Publishers ---
        self.path_pub   = rospy.Publisher('/trajectory',   Path,   queue_size=1)
        self.marker_pub = rospy.Publisher('/goal_marker',  Marker, queue_size=1)

        # --- Subscribers ---
        rospy.Subscriber('/odom',       Odometry,    self._odom_cb)
        rospy.Subscriber('/goal_pose',  PoseStamped, self._goal_cb)

        rospy.loginfo("rviz_helper : publié /trajectory et /goal_marker")

    # ── callbacks ─────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = 'odom'
        ps.pose = msg.pose.pose
        self.path_poses.append(ps)

        # Limiter la longueur du chemin en mémoire
        if len(self.path_poses) > 2000:
            self.path_poses = self.path_poses[-2000:]

    def _goal_cb(self, msg):
        self.goal_pose = msg

    # ── boucle ────────────────────────────────────────────────────────
    def run(self):
        rate = rospy.Rate(10)   # 10 Hz suffit pour la visualisation
        while not rospy.is_shutdown():

            # --- /trajectory ---
            if self.path_poses:
                path_msg         = Path()
                path_msg.header.stamp    = rospy.Time.now()
                path_msg.header.frame_id = 'odom'
                path_msg.poses   = self.path_poses
                self.path_pub.publish(path_msg)

            # --- /goal_marker (étoile rouge) ---
            if self.goal_pose is not None:
                m = Marker()
                m.header.frame_id = 'odom'
                m.header.stamp    = rospy.Time.now()
                m.ns              = 'goal'
                m.id              = 0
                m.type            = Marker.CYLINDER   # pilier rouge visible
                m.action          = Marker.ADD

                m.pose            = self.goal_pose.pose
                m.pose.position.z = 0.05              # légèrement au-dessus du sol

                m.scale.x = 0.08
                m.scale.y = 0.08
                m.scale.z = 0.30

                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 1.0

                self.marker_pub.publish(m)

            rate.sleep()


if __name__ == '__main__':
    try:
        RVizHelper().run()
    except rospy.ROSInterruptException:
        pass
