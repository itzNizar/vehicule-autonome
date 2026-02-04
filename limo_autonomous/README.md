# LIMO Autonomous – Bureau d'etude ENSEM

## Description

Package ROS Melodic (Python 2.7) pour le contrôle autonome du robot Agilex Limo
en mode Ackermann.  
Implémente une loi de commande en coordonnées polaires **avec détection et
évitement d'obstacles via le LiDAR**, visualisation RViz et traçage rqt_plot.

**Date :** Février 2026  
**Équipe :** Bureau d'etude ENSEM  
**Version :** 1.1.0

---

## Installation

```bash
cp -r limo_autonomous ~/agilex_ws/src/
chmod +x ~/agilex_ws/src/limo_autonomous/scripts/*.py
cd ~/agilex_ws && catkin_make
source devel/setup.bash
```

---

## Utilisation rapide

```bash
# Tout en un (driver + contrôleur + obstacles + RViz + rqt_plot)
roslaunch limo_autonomous autonomous.launch

# Sans driver (déjà lancé)
roslaunch limo_autonomous autonomous.launch use_driver:=false

# Sans RViz ni rqt_plot
roslaunch limo_autonomous autonomous.launch use_rviz:=false use_rqt:=false
```

### Envoyer une cible

```bash
cd ~/agilex_ws/src/limo_autonomous/scripts
python send_goal.py --start          # activer le contrôleur
python send_goal.py 1.0 0.0 0        # cible (1 m, 0 m, θ=0°)
python send_goal.py --stop           # arrêt
python send_goal.py --emergency      # arrêt d'urgence
```

---

## Architecture du système

```
/scan  ──►  obstacle_avoidance_node  ──► /obstacle_detected ──► controller_node ──► /cmd_vel
                    │                                                   ▲
                    ▼                                                   │
          /obstacle/min_dist_*                              /goal_pose  /odom
          /obstacle/status

/odom  ──►  rviz_helper_node  ──► /trajectory  ──► RViz
                    │
                    ▼
            /goal_marker     ──► RViz
```

### Noeuds

| Noeud | Script | Rôle |
|-------|--------|------|
| `limo_controller` | `controller_node.py` | Loi de commande polaire |
| `obstacle_avoidance` | `obstacle_avoidance_node.py` | Détection obstacles LiDAR |
| `rviz_helper` | `rviz_helper_node.py` | Publié trajectoire + goal marker |
| `supervisor_gui` | `supervisor_gui.py` | GUI Tkinter |

---

## Topics ROS

### Souscrits

| Topic | Type | Utilisé par |
|-------|------|-------------|
| `/odom` | nav_msgs/Odometry | controller, rviz_helper |
| `/scan` | sensor_msgs/LaserScan | obstacle_avoidance |
| `/goal_pose` | geometry_msgs/PoseStamped | controller, rviz_helper |
| `/controller/enable` | std_msgs/Bool | controller |
| `/obstacle_detected` | std_msgs/Bool | controller |

### Publié

| Topic | Type | Publié par |
|-------|------|------------|
| `/cmd_vel` | geometry_msgs/Twist | controller, obstacle_avoidance (arrêt) |
| `/controller/state` | std_msgs/String | controller |
| `/controller/distance` | std_msgs/Float64 | controller |
| `/controller/polar_coords` | geometry_msgs/Vector3 | controller |
| `/obstacle_detected` | std_msgs/Bool | obstacle_avoidance |
| `/obstacle/min_dist_left` | std_msgs/Float64 | obstacle_avoidance |
| `/obstacle/min_dist_center` | std_msgs/Float64 | obstacle_avoidance |
| `/obstacle/min_dist_right` | std_msgs/Float64 | obstacle_avoidance |
| `/obstacle/status` | std_msgs/String | obstacle_avoidance |
| `/trajectory` | nav_msgs/Path | rviz_helper |
| `/goal_marker` | visualization_msgs/Marker | rviz_helper |

---

## Paramètres

### Contrôleur

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `k_rho` | 1.0 | Gain distance |
| `k_alpha` | 3.0 | Gain angle vers cible |
| `k_beta` | -0.5 | Gain orientation finale |
| `v_max` | 0.5 | Vitesse max (m/s) |
| `wheelbase` | 0.2 | Empattement (m) |

### Évitement obstacles

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `stop_distance` | 0.3 | Distance d'arrêt (m) |
| `slow_distance` | 0.6 | Distance ralenti (m) |
| `fov_center_deg` | 30 | Demi-angle secteur centre (°) |

---

## Théorie – Loi de commande

```
v     = k_rho  × ρ
ω     = k_alpha × α  +  k_beta × β
γ     = arctan(ω × L / v)
```

Conditions de stabilité : **k_rho > 0**, **k_alpha > k_rho**, **k_beta < 0**.

---

## Structure du package

```
limo_autonomous/
├── scripts/
│   ├── controller_node.py            # Loi de commande (avec guard obstacle)
│   ├── obstacle_avoidance_node.py    # Détection obstacles LiDAR
│   ├── rviz_helper_node.py           # Publié /trajectory + /goal_marker
│   ├── send_goal.py                  # CLI utilitaire
│   ├── simulation_standalone.py      # Simulation Python pure
│   └── supervisor_gui.py             # GUI Tkinter
├── launch/
│   ├── autonomous.launch             # Lancement complet
│   ├── controller_only.launch        # Contrôleur + obstacles (driver séparé)
│   ├── supervisor.launch             # GUI + RViz + rqt
│   └── rqt_plot.launch               # 3 fenêtres rqt_plot
├── rviz/
│   └── limo_full.rviz                # Configuration RViz complète
├── config/
│   └── controller_params.yaml        # Paramètres
├── CMakeLists.txt
├── package.xml
└── README.md
```
