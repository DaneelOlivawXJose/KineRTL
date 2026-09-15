import roboticstoolbox as rtb
import numpy as np

# 1. Definir el modelo del robot (el mismo que usa tu VHDL en robot_config_pkg.vhd)
robot = rtb.DHRobot([
    rtb.RevoluteDH(d=0.61, a=0.04, alpha=1.5708),
    rtb.RevoluteDH(d=0.00, a=0.51, alpha=0.0),
    rtb.RevoluteDH(d=0.00, a=0.05, alpha=-1.5708),
    rtb.RevoluteDH(d=0.59, a=0.00, alpha=1.5708),
    rtb.RevoluteDH(d=0.00, a=0.00, alpha=-1.5708),
    rtb.RevoluteDH(d=0.26, a=0.00, alpha=0.0)
], name="RobotFPGA")

# 2. Los valores en Q8.24 que sacó tu VHDL en la imagen (theta_out)
vhdl_q_raw = [25855467, 3144786, -1696234, 16510257, 7564915, 23297561]

# Convertir de Q8.24 a radianes dividiendo entre 2^24 (16777216.0)
vhdl_q_rad = [val / 16777216.0 for val in vhdl_q_raw]

print("Ángulos devueltos por la FPGA (en radianes):")
print(np.array(vhdl_q_rad))

# 3. Calcular la Cinemática Directa (DK) en Python usando los ángulos de la FPGA
T_resultante = robot.fkine(vhdl_q_rad)

print("\nMatriz Homogénea generada por el VHDL:")
print(T_resultante)

input("\nPulsa Enter para salir...")