import numpy as np
import matplotlib.pyplot as plt
import math

theta1_deg = 40.0  
theta2_deg = 30.0  

L1 = 2.0   
L2 = 19.0  
coxa = 0.0  

theta1 = math.radians(theta1_deg)
theta2 = math.radians(theta2_deg)

x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

print("=== FORWARD KINEMATICS ===")
print(f"Theta1 = {theta1_deg}°")
print(f"Theta2 = {theta2_deg}°")
print(f"End-Effector (x, y) = ({x:.4f}, {y:.4f})\n")

# HOMOGENOUS TRANSFORMATION MATRICES
def rot_z(theta):
    return np.array([
        [math.cos(theta), -math.sin(theta), 0],
        [math.sin(theta),  math.cos(theta), 0],
        [0, 0, 1]
    ])

def trans_x(a):
    return np.array([
        [1, 0, a],
        [0, 1, 0],
        [0, 0, 1]
    ])

T0_1 = rot_z(theta1) @ trans_x(L1)
T1_2 = rot_z(theta2) @ trans_x(L2)
T0_2 = T0_1 @ T1_2

print("=== HOMOGENEOUS TRANSFORMATION MATRICES ===")
np.set_printoptions(precision=4, suppress=True)
print("T0_1 =\n", T0_1)
print("\nT1_2 =\n", T1_2)
print("\nT0_2 =\n", T0_2, "\n")

# Inverse Kinematik
def inverse_kinematics(x, y, L1, L2):
    r2 = x**2 + y**2
    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)

    if cos_theta2 < -1.0 or cos_theta2 > 1.0:
        return []  # Jika Tidak ada solusi

    solutions = []
    for sign in [+1, -1]:  # dua kemungkinan: elbow up dan elbow down
        sin_theta2 = sign * math.sqrt(1 - cos_theta2**2)
        theta2 = math.atan2(sin_theta2, cos_theta2)
        k1 = L1 + L2 * math.cos(theta2)
        k2 = L2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        solutions.append((math.degrees(theta1), math.degrees(theta2)))

    return solutions

ik_solutions = inverse_kinematics(x, y, L1, L2)
print("=== INVERSE KINEMATICS ===")
if ik_solutions:
    for i, (t1, t2) in enumerate(ik_solutions, 1):
        print(f"Solusi {i}: theta1 = {t1:.4f}°, theta2 = {t2:.4f}°")
else:
    print("Tidak ada solusi IK (posisi di luar jangkauan)\n")

# Visualisasi Matplotlib
x0, y0 = 0, 0
x1 = L1 * math.cos(theta1)
y1 = L1 * math.sin(theta1)
x2 = x
y2 = y

plt.figure(figsize=(6, 6))
plt.plot([x0, x1, x2], [y0, y1, y2], '-o', linewidth=2)
plt.text(x0, y0, "Coxa (0,0)")
plt.text(x1, y1, f"Joint1 ({x1:.2f}, {y1:.2f})")
plt.text(x2, y2, f"End ({x2:.2f}, {y2:.2f})")

plt.title("2-DOF Planar Leg - Forward Kinematics")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.grid(True)
plt.axis('equal')
plt.show()
