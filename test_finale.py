#!/usr/bin/env python3
import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# =========================================================================================
# 1. SOLVEUR REEDS-SHEPP (Version que tu as validée : LSL, LSR, LRL)
# =========================================================================================

class Path:
    def __init__(self, lengths, ctypes):
        self.lengths = lengths        
        self.ctypes = ctypes          
        self.total_length = sum([abs(l) for l in lengths])
        self.type_name = "".join(ctypes)

class ReedsSheppPlanner:
    def __init__(self, r_min, step_size=0.05):
        self.r_min = float(r_min)
        self.step_size = step_size 

    def mod2pi(self, x):
        v = x % (2 * math.pi)
        return v

    def polar(self, x, y):
        r = math.sqrt(x**2 + y**2)
        theta = math.atan2(y, x)
        return r, theta

    def path_LSL(self, x, y, phi):
        u_pol, t_pol = self.polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
        if t_pol >= 0.0:
            u = u_pol 
            t = self.mod2pi(t_pol)
            v = self.mod2pi(phi - t)
            if v >= 0.0: return [t, u, v]
        return None

    def path_LRL(self, x, y, phi):
        u_pol, t_pol = self.polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
        if u_pol <= 4.0:
            u = -2.0 * math.asin(u_pol / 4.0)
            t = self.mod2pi(t_pol + u/2.0 + math.pi)
            v = self.mod2pi(phi - t + u)
            if t >= 0.0 and v >= 0.0: return [t, u, v]
        return None

    def try_all_paths(self, x, y, phi):
        paths = []
        p = self.path_LSL(x, y, phi)
        if p: paths.append(Path(p, ['L', 'S', 'L']))
        p = self.path_LSL(-x, y, -phi) 
        if p: paths.append(Path([-l for l in p], ['L', 'S', 'R']))
        p = self.path_LRL(x, y, phi)
        if p: paths.append(Path(p, ['L', 'R', 'L']))
        return paths

    def replace_LR(self, ctypes):
        new_types = []
        for t in ctypes:
            if t == 'L': new_types.append('R')
            elif t == 'R': new_types.append('L')
            else: new_types.append(t)
        return new_types

    def get_optimal_path(self, x, y, phi):
        paths = []
        paths += self.try_all_paths(x, y, phi)
        p_tf = self.try_all_paths(-x, y, -phi)
        for p in p_tf: p.lengths = [-l for l in p.lengths]
        paths += p_tf
        p_rf = self.try_all_paths(x, -y, -phi)
        for p in p_rf: p.ctypes = self.replace_LR(p.ctypes)
        paths += p_rf
        p_tr = self.try_all_paths(-x, -y, phi)
        for p in p_tr:
            p.lengths = [-l for l in p.lengths]
            p.ctypes = self.replace_LR(p.ctypes)
        paths += p_tr
        
        if not paths: return None
        return min(paths, key=lambda p: p.total_length)

    def reeds_shepp_path(self, q0, q1):
        dx = q1[0] - q0[0]; dy = q1[1] - q0[1]; dth = q1[2] - q0[2]
        c = math.cos(q0[2]); s = math.sin(q0[2])
        x = (c * dx + s * dy) / self.r_min
        y = (-s * dx + c * dy) / self.r_min
        
        best = self.get_optimal_path(x, y, dth)
        if best:
            best.lengths = [l * self.r_min for l in best.lengths]
            best.total_length *= self.r_min
            return best
        return None

    def discretize_path(self, path):
        if not path: return []
        waypoints = []
        x, y, yaw = 0.0, 0.0, 0.0
        waypoints.append({'x': x, 'y': y, 'yaw': yaw, 'gear': 1 if path.lengths[0]>=0 else -1})

        for l, t in zip(path.lengths, path.ctypes):
            gear = 1 if l >= 0 else -1
            dist = abs(l)
            step_count = int(dist / self.step_size) + 1
            curv = (1.0/self.r_min) if t == 'L' else ((-1.0/self.r_min) if t == 'R' else 0)
            
            for _ in range(step_count):
                x += gear * self.step_size * math.cos(yaw)
                y += gear * self.step_size * math.sin(yaw)
                yaw += gear * self.step_size * curv
                waypoints.append({'x': x, 'y': y, 'yaw': yaw, 'gear': gear})
            
            remain = dist - (step_count * self.step_size)
            if remain > 1e-4:
                x += gear * remain * math.cos(yaw)
                y += gear * remain * math.sin(yaw)
                yaw += gear * remain * curv
                waypoints[-1] = {'x': x, 'y': y, 'yaw': yaw, 'gear': gear}
        return waypoints

# =========================================================================================
# 2. CONTROLEUR ROBOT (AVEC LOGS DEBUG)
# =========================================================================================

class PIDController:
    def __init__(self, kp, ki, kd, max_out):
        self.kp, self.ki, self.kd, self.max_out = kp, ki, kd, max_out
        self.prev_error, self.integral, self.last_time = 0.0, 0.0, None

    def compute(self, error):
        current_time = time.time()
        if self.last_time is None: dt = 0.05
        else: dt = current_time - self.last_time
        self.integral += error * dt
        self.integral = max(min(self.integral, 2.0), -2.0) 
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        self.last_time = current_time
        return max(min(output, self.max_out), -self.max_out)

class LimoParkingVerbose:
    def __init__(self):
        rospy.init_node('limo_parking_debug', anonymous=False)
        rospy.on_shutdown(self.stop_robot)
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.R_MIN = 0.40
        self.v_cruise = 0.20
        self.pid_w = PIDController(kp=2.5, ki=0.01, kd=0.1, max_out=0.5)
        self.planner = ReedsSheppPlanner(self.R_MIN)

        # CONFIGURATION DE LA CIBLE
        self.xf, self.yf, self.tf = 1.0, 0.0, math.pi/2 
        
        self.xi, self.yi, self.ti = None, None, None
        self.path_waypoints = []
        self.current_idx = 0
        self.done = False
        self.debug_counter = 0 # Pour limiter les logs
        
        rospy.logwarn(">>> DEBUG MODE ACTIVÉ <<<")
        rospy.loginfo(f"Cible demandée : X={self.xf}, Y={self.yf}, Th={math.degrees(self.tf)}°")
        rospy.loginfo("En attente du message /odom...")

    def stop_robot(self):
        rospy.logwarn("!!! ARRET D'URGENCE / FIN !!!")
        self.pub.publish(Twist())

    def q_to_y(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def odom_callback(self, msg):
        if self.done: return
        
        # --- FIX NameError : Initialisation par défaut ---
        v_cmd = 0.0
        w_out = 0.0
        
        # 1. POSITION ACTUELLE
        cx, cy = msg.pose.pose.position.x, msg.pose.pose.position.y
        ct = self.q_to_y(msg.pose.pose.orientation)
        
        # 2. INITIALISATION (Calibration origine)
        if self.xi is None:
            self.xi, self.yi, self.ti = cx, cy, ct
            rospy.loginfo(f"--- INIT POSITION: X={cx:.2f}, Y={cy:.2f}, Th={math.degrees(ct):.1f}° ---")
            
            # Calcul Trajectoire Reeds-Shepp
            path = self.planner.reeds_shepp_path((0,0,0), (self.xf, self.yf, self.tf))
            
            if path:
                rospy.loginfo(f"CHEMIN TROUVÉ : {path.type_name}")
                self.path_waypoints = self.planner.discretize_path(path)
            else:
                rospy.logerr("ECHEC: Pas de chemin trouvé (Vérifier R_MIN ou Cible).")
                self.done = True
            return
        
        # 3. CONVERSION EN REPÈRE LOCAL
        dgx, dgy = cx - self.xi, cy - self.yi
        x = dgx * math.cos(-self.ti) - dgy * math.sin(-self.ti)
        y = dgx * math.sin(-self.ti) + dgy * math.cos(-self.ti)
        t = math.atan2(math.sin(ct - self.ti), math.cos(ct - self.ti))
        
        # 4. SUIVI DE TRAJECTOIRE ROBUSTE
        if not self.path_waypoints: return

        # --- FIX IndexError : Vérification AVANT lecture ---
        if self.current_idx >= len(self.path_waypoints):
            self.stop_robot()
            self.done = True
            rospy.loginfo(f"MISSION TERMINÉE (Fin de liste). Pos:({x:.2f},{y:.2f})")
            return
        # ---------------------------------------------------

        target = self.path_waypoints[self.current_idx]
        dist = math.sqrt((target['x'] - x)**2 + (target['y'] - y)**2)

        # LOGIQUE DE BASCULE (Pure Pursuit Simplifié)
        # Si on est plus proche du point SUIVANT que de l'actuel, on avance l'index
        if self.current_idx < len(self.path_waypoints) - 1:
            next_target = self.path_waypoints[self.current_idx + 1]
            dist_next = math.sqrt((next_target['x'] - x)**2 + (next_target['y'] - y)**2)
            if dist_next < dist:
                self.current_idx += 1
                return # On attend la prochaine boucle

        # Condition d'arrivée précise sur le dernier point
        if self.current_idx == len(self.path_waypoints) - 1 and dist < 0.15:
            self.stop_robot()
            self.done = True
            rospy.loginfo("ARRIVÉ (Cible atteinte) !")
            return

        # Passage de waypoint standard (Tolérance 20cm)
        if dist < 0.20:
            self.current_idx += 1
            return

        # 5. CALCUL COMMANDE
        angle_to_target = math.atan2(target['y'] - y, target['x'] - x)
        gear = target['gear']
        
        alpha = math.atan2(math.sin(angle_to_target - t), math.cos(angle_to_target - t))
        if gear == -1: alpha = math.atan2(math.sin(alpha - math.pi), math.cos(alpha - math.pi))
        
        beta = math.atan2(math.sin(target['yaw'] - t), math.cos(target['yaw'] - t))
        if gear == -1: beta = math.atan2(math.sin(beta - math.pi), math.cos(beta - math.pi))

        w_target = 2.0 * alpha + 0.8 * beta 
        w_out = self.pid_w.compute(w_target)
        v_cmd = gear * self.v_cruise
        
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_out
        self.pub.publish(cmd)
        
        # LOGS
        self.debug_counter += 1
        if self.debug_counter % 10 == 0:
             rospy.loginfo(f"IDX={self.current_idx} | Dist={dist:.2f} | Gear={gear} | CMD: v={v_cmd:.2f}, w={w_out:.2f}")

if __name__ == '__main__':
    try:
        LimoParkingVerbose()
        rospy.spin()
    except rospy.ROSInterruptException: pass