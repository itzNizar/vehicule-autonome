#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Interface de Supervision pour le robot Agilex Limo
GUI Tkinter pour commander et visualiser le robot
Compatible Python 2.7 (ROS Melodic)

Auteur: Bureau d'etude ENSEM
Date: Fevrier 2026

Fonctionnalites:
    - Visualisation position/orientation en temps reel
    - Envoi de commandes (goals)
    - Start/Stop du controleur
    - Affichage des coordonnees polaires
"""

from __future__ import print_function, division

import math
import threading

# Tkinter (compatible Python 2 et 3)
try:
    import Tkinter as tk
    import ttk
    from Tkinter import messagebox
except ImportError:
    import tkinter as tk
    from tkinter import ttk
    from tkinter import messagebox

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64


def quaternion_to_euler(x, y, z, w):
    """Conversion quaternion vers yaw"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def euler_to_quaternion(yaw):
    """Conversion yaw vers quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return 0.0, 0.0, sy, cy  # x, y, z, w


class TrajectoryCanvas(tk.Canvas):
    """
    Canvas pour visualiser la trajectoire du robot
    """
    
    def __init__(self, parent, width=400, height=400):
        tk.Canvas.__init__(self, parent, width=width, height=height, bg='white')
        
        self.width = width
        self.height = height
        
        # Donnees
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        
        self.trajectory = []
        self.max_points = 500
        
        # Echelle
        self.scale = 80  # pixels par metre
        self.offset_x = width // 2
        self.offset_y = height // 2
        
        self.draw_grid()
    
    def world_to_screen(self, x, y):
        """Convertit coordonnees monde -> ecran"""
        sx = int(self.offset_x + x * self.scale)
        sy = int(self.offset_y - y * self.scale)
        return sx, sy
    
    def draw_grid(self):
        """Dessine la grille de fond"""
        self.delete("grid")
        
        # Grille
        for i in range(-5, 6):
            sx, sy = self.world_to_screen(i, -5)
            ex, ey = self.world_to_screen(i, 5)
            color = '#CCCCCC' if i != 0 else '#888888'
            width = 1 if i != 0 else 2
            self.create_line(sx, sy, ex, ey, fill=color, width=width, tags="grid")
            
            sx, sy = self.world_to_screen(-5, i)
            ex, ey = self.world_to_screen(5, i)
            self.create_line(sx, sy, ex, ey, fill=color, width=width, tags="grid")
        
        # Labels
        ex, ey = self.world_to_screen(4.5, 0)
        self.create_text(ex, ey - 10, text="X", fill='gray', tags="grid")
        ex, ey = self.world_to_screen(0, 4.5)
        self.create_text(ex + 10, ey, text="Y", fill='gray', tags="grid")
    
    def update_robot(self, x, y, theta):
        """Met a jour la position du robot"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
        
        self.trajectory.append((x, y))
        if len(self.trajectory) > self.max_points:
            self.trajectory.pop(0)
        
        self.redraw()
    
    def update_goal(self, x, y, theta):
        """Met a jour la cible"""
        self.goal_x = x
        self.goal_y = y
        self.goal_theta = theta
        self.redraw()
    
    def clear_trajectory(self):
        """Efface la trajectoire"""
        self.trajectory = []
        self.redraw()
    
    def redraw(self):
        """Redessine tout"""
        self.delete("dynamic")
        
        # Trajectoire
        if len(self.trajectory) > 1:
            points = []
            for x, y in self.trajectory:
                sx, sy = self.world_to_screen(x, y)
                points.extend([sx, sy])
            if len(points) >= 4:
                self.create_line(points, fill='#6666FF', width=2, tags="dynamic")
        
        # Cible
        if self.goal_x is not None:
            gx, gy = self.world_to_screen(self.goal_x, self.goal_y)
            
            # Croix
            self.create_line(gx-10, gy, gx+10, gy, fill='red', width=2, tags="dynamic")
            self.create_line(gx, gy-10, gx, gy+10, fill='red', width=2, tags="dynamic")
            
            # Fleche orientation
            arrow_len = 20
            ax = gx + int(arrow_len * math.cos(self.goal_theta))
            ay = gy - int(arrow_len * math.sin(self.goal_theta))
            self.create_line(gx, gy, ax, ay, fill='red', width=2, arrow=tk.LAST, tags="dynamic")
        
        # Robot
        rx, ry = self.world_to_screen(self.robot_x, self.robot_y)
        
        # Corps (cercle)
        r = 12
        self.create_oval(rx-r, ry-r, rx+r, ry+r, fill='#66CC66', outline='#006600', width=2, tags="dynamic")
        
        # Fleche orientation
        arrow_len = 25
        ax = rx + int(arrow_len * math.cos(self.robot_theta))
        ay = ry - int(arrow_len * math.sin(self.robot_theta))
        self.create_line(rx, ry, ax, ay, fill='#004400', width=3, arrow=tk.LAST, tags="dynamic")


class SupervisorGUI:
    """
    Fenetre principale de supervision
    """
    
    def __init__(self):
        # Fenetre principale
        self.root = tk.Tk()
        self.root.title("LIMO Supervisor - Bureau d'etude ENSEM")
        self.root.geometry("900x600")
        
        # Variables
        self.current_x = tk.StringVar(value="0.000")
        self.current_y = tk.StringVar(value="0.000")
        self.current_theta = tk.StringVar(value="0.0")
        self.current_state = tk.StringVar(value="UNKNOWN")
        self.current_distance = tk.StringVar(value="0.000")
        
        self.polar_rho = tk.StringVar(value="0.000")
        self.polar_alpha = tk.StringVar(value="0.0")
        self.polar_beta = tk.StringVar(value="0.0")
        
        self.goal_x = tk.DoubleVar(value=1.0)
        self.goal_y = tk.DoubleVar(value=0.0)
        self.goal_theta = tk.DoubleVar(value=0.0)
        
        # Publishers ROS
        self.goal_pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)
        self.enable_pub = rospy.Publisher('/controller/enable', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Interface
        self.setup_ui()
        
        # Subscribers ROS (dans un thread separe)
        self.running = True
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/controller/state', String, self.state_callback)
        rospy.Subscriber('/controller/polar_coords', Vector3, self.polar_callback)
        rospy.Subscriber('/controller/distance', Float64, self.distance_callback)
    
    def setup_ui(self):
        """Configure l'interface"""
        # Frame principale
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Gauche: Canvas
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.canvas = TrajectoryCanvas(left_frame, 450, 450)
        self.canvas.pack(pady=10)
        
        clear_btn = ttk.Button(left_frame, text="Effacer trajectoire", 
                               command=self.canvas.clear_trajectory)
        clear_btn.pack()
        
        # Droite: Controles
        right_frame = ttk.Frame(main_frame, padding="10")
        right_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        # --- Etat du robot ---
        state_frame = ttk.LabelFrame(right_frame, text="Etat du Robot", padding="5")
        state_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(state_frame, text="Position X:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(state_frame, textvariable=self.current_x).grid(row=0, column=1, sticky=tk.E)
        ttk.Label(state_frame, text="m").grid(row=0, column=2, sticky=tk.W)
        
        ttk.Label(state_frame, text="Position Y:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(state_frame, textvariable=self.current_y).grid(row=1, column=1, sticky=tk.E)
        ttk.Label(state_frame, text="m").grid(row=1, column=2, sticky=tk.W)
        
        ttk.Label(state_frame, text="Orientation:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(state_frame, textvariable=self.current_theta).grid(row=2, column=1, sticky=tk.E)
        ttk.Label(state_frame, text="deg").grid(row=2, column=2, sticky=tk.W)
        
        ttk.Label(state_frame, text="Etat:").grid(row=3, column=0, sticky=tk.W)
        self.state_label = ttk.Label(state_frame, textvariable=self.current_state)
        self.state_label.grid(row=3, column=1, columnspan=2, sticky=tk.E)
        
        ttk.Label(state_frame, text="Distance:").grid(row=4, column=0, sticky=tk.W)
        ttk.Label(state_frame, textvariable=self.current_distance).grid(row=4, column=1, sticky=tk.E)
        ttk.Label(state_frame, text="m").grid(row=4, column=2, sticky=tk.W)
        
        # --- Coordonnees polaires ---
        polar_frame = ttk.LabelFrame(right_frame, text="Coordonnees Polaires", padding="5")
        polar_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(polar_frame, text="rho:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(polar_frame, textvariable=self.polar_rho).grid(row=0, column=1, sticky=tk.E)
        
        ttk.Label(polar_frame, text="alpha:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(polar_frame, textvariable=self.polar_alpha).grid(row=1, column=1, sticky=tk.E)
        
        ttk.Label(polar_frame, text="beta:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(polar_frame, textvariable=self.polar_beta).grid(row=2, column=1, sticky=tk.E)
        
        # --- Definir une cible ---
        goal_frame = ttk.LabelFrame(right_frame, text="Definir une Cible", padding="5")
        goal_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(goal_frame, text="X* (m):").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(goal_frame, textvariable=self.goal_x, width=10).grid(row=0, column=1)
        
        ttk.Label(goal_frame, text="Y* (m):").grid(row=1, column=0, sticky=tk.W)
        ttk.Entry(goal_frame, textvariable=self.goal_y, width=10).grid(row=1, column=1)
        
        ttk.Label(goal_frame, text="theta* (deg):").grid(row=2, column=0, sticky=tk.W)
        ttk.Entry(goal_frame, textvariable=self.goal_theta, width=10).grid(row=2, column=1)
        
        send_btn = ttk.Button(goal_frame, text="Envoyer Cible", command=self.send_goal)
        send_btn.grid(row=3, column=0, columnspan=2, pady=5)
        
        # --- Controles ---
        control_frame = ttk.LabelFrame(right_frame, text="Controles", padding="5")
        control_frame.pack(fill=tk.X, pady=5)
        
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(fill=tk.X)
        
        start_btn = ttk.Button(btn_frame, text="START", command=self.start_controller)
        start_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        stop_btn = ttk.Button(btn_frame, text="STOP", command=self.stop_controller)
        stop_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        emergency_btn = tk.Button(control_frame, text="ARRET D'URGENCE", 
                                  command=self.emergency_stop,
                                  bg='red', fg='white', font=('Arial', 10, 'bold'))
        emergency_btn.pack(fill=tk.X, pady=5)
        
        # --- Cibles predefinies ---
        presets_frame = ttk.LabelFrame(right_frame, text="Cibles Predefinies", padding="5")
        presets_frame.pack(fill=tk.X, pady=5)
        
        presets = [
            ("(1, 0, 0)", 1, 0, 0),
            ("(2, 1, 45)", 2, 1, 45),
            ("(0, 2, 90)", 0, 2, 90),
            ("Origine", 0, 0, 0),
        ]
        
        for i, (name, x, y, theta) in enumerate(presets):
            btn = ttk.Button(presets_frame, text=name,
                           command=lambda x=x, y=y, t=theta: self.set_preset(x, y, t))
            btn.grid(row=i//2, column=i%2, padx=2, pady=2, sticky=tk.EW)
    
    def ros_spin(self):
        """Thread ROS"""
        rate = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            rate.sleep()
    
    def odom_callback(self, msg):
        """Callback odometrie"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        theta = quaternion_to_euler(o.x, o.y, o.z, o.w)
        
        self.current_x.set("{:.3f}".format(x))
        self.current_y.set("{:.3f}".format(y))
        self.current_theta.set("{:.1f}".format(math.degrees(theta)))
        
        # Mettre a jour le canvas (thread-safe)
        self.root.after(0, lambda: self.canvas.update_robot(x, y, theta))
    
    def state_callback(self, msg):
        """Callback etat"""
        self.current_state.set(msg.data)
    
    def polar_callback(self, msg):
        """Callback coordonnees polaires"""
        self.polar_rho.set("{:.3f} m".format(msg.x))
        self.polar_alpha.set("{:.1f} deg".format(math.degrees(msg.y)))
        self.polar_beta.set("{:.1f} deg".format(math.degrees(msg.z)))
    
    def distance_callback(self, msg):
        """Callback distance"""
        self.current_distance.set("{:.3f}".format(msg.data))
    
    def send_goal(self):
        """Envoie la cible"""
        x = self.goal_x.get()
        y = self.goal_y.get()
        theta = math.radians(self.goal_theta.get())
        
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        qx, qy, qz, qw = euler_to_quaternion(theta)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        
        self.goal_pub.publish(msg)
        self.canvas.update_goal(x, y, theta)
        
        print("Cible envoyee: ({:.2f}, {:.2f}), theta={:.1f} deg".format(
            x, y, self.goal_theta.get()))
    
    def set_preset(self, x, y, theta):
        """Definit une cible predefinie"""
        self.goal_x.set(x)
        self.goal_y.set(y)
        self.goal_theta.set(theta)
        self.send_goal()
    
    def start_controller(self):
        """Active le controleur"""
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)
        print("Controleur ACTIVE")
    
    def stop_controller(self):
        """Desactive le controleur"""
        msg = Bool()
        msg.data = False
        self.enable_pub.publish(msg)
        print("Controleur DESACTIVE")
    
    def emergency_stop(self):
        """Arret d'urgence"""
        # Desactiver
        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)
        
        # Arreter
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        print("ARRET D'URGENCE!")
    
    def run(self):
        """Lance l'interface"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()
    
    def on_close(self):
        """Fermeture"""
        self.running = False
        self.root.destroy()


def main():
    # Initialisation ROS
    rospy.init_node('limo_supervisor_gui', anonymous=False)
    
    # Interface
    gui = SupervisorGUI()
    gui.run()


if __name__ == '__main__':
    main()
