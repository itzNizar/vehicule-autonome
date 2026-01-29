from pylimo import limo
import time
import math

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def saturate(value, min_val, max_val):
    return max(min(value, max_val), min_val)

robot = limo.LIMO()
robot.EnableCommand()
time.sleep(0.5)

L = 0.2
k_rho = 0.5
k_alpha = 3.0
k_beta = -2.0
v_max = 0.3
gamma_max = math.radians(25)

x_star = 1.0
y_star = 0.0
theta_star = 0.0

rho_threshold = 0.05
theta_threshold = math.radians(5)

X = 0.0
Y = 0.0
Theta = 0.0
T = 0.1
iteration = 0

INIT_YAW = robot.GetIMUYawData()

try:
    while True:
        iteration += 1
        
        yaw_current = robot.GetIMUYawData()
        yaw_deg = yaw_current - INIT_YAW
        Theta = wrap_to_pi(math.radians(yaw_deg))

        v_measured = robot.GetLinearVelocity()
        X = X + T * v_measured * math.cos(Theta)
        Y = Y + T * v_measured * math.sin(Theta)

        dx = x_star - X
        dy = y_star - Y
        Rho = math.sqrt(dx**2 + dy**2)

        angle_to_goal = math.atan2(dy, dx)
        Alpha = wrap_to_pi(angle_to_goal - Theta)
        Beta = wrap_to_pi(theta_star - angle_to_goal)

        if abs(Alpha) > math.pi/2:
            direction = -1
            Alpha = wrap_to_pi(Alpha - math.copysign(math.pi, Alpha))
            Beta = wrap_to_pi(Beta - math.copysign(math.pi, Beta))
        else:
            direction = 1

        theta_error = abs(wrap_to_pi(Theta - theta_star))
        
        if iteration % 5 == 0:
            print(f"Pos:({X:.2f},{Y:.2f}) Theta:{math.degrees(Theta):.1f}° Rho:{Rho:.2f}")

        if Rho < rho_threshold and theta_error < theta_threshold:
            print("CIBLE ATTEINTE !")
            robot.SetMotionCommand(linear_vel=0, steering_angle=0)
            break

        V = direction * k_rho * Rho
        w = k_alpha * Alpha + k_beta * Beta

        V = saturate(V, -v_max, v_max)
        
        if abs(V) > 1e-6:
            Gamma = math.atan(w * L / abs(V))
        else:
            Gamma = 0.0
            
        Gamma = saturate(Gamma, -gamma_max, gamma_max)

        robot.SetMotionCommand(linear_vel=V, steering_angle=Gamma)
        time.sleep(T)

except KeyboardInterrupt:
    robot.SetMotionCommand(linear_vel=0, steering_angle=0)