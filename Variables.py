import numpy as np
import Variables_global as vbg
import kinematics_v2 as kin

# ============================================================
# VARIABILI CINEMATICHE E DI POSIZIONAMENTO
# ============================================================

'''
Le pose sono di base delle liste di 6 valori [x, y, z, rx, ry, rz] in mm e gradi,
oppure le pose dei giunti sono liste di 6 valori in gradi [j1, j2, j3, j4, j5, j6]

I punti invece sono array numpy di 3 valori [x, y, z] in mm
anche in coordinate sfereiche [r, alpha, beta] con r in mm e alpha, beta in gradi
'''

CAMERA_POSE_EE = [32.83, 22.8 , 90.85, 0.0, 0.0, -90.0]  # Posa della fotocamera rispetto all'End Effector
MARKER_POSE_EE = [0.0, 0.22, 140, 0.0, 0.0, -90.0]       # Posa del marcatore rispetto all'End Effector

BASE_MARKER_POSE = [x, y, z, 0, 0, 0]
H_marker = kin.compute_homogeneous_transform(BASE_MARKER_POSE)


# Da qua in poi vanno cambiate trasformale prima in relative al marker nella posizione iniziale
# poi definisci la nuova posizone del marker ruotato
# le ritrasformi in globale -> devi aggiungere 180 alla rz

# legacy defaults
#DEFAULT_POSITIONING_JOINTS = [-132 , 0, -133, -48 , -42, 180]
#DEFAULT_POSITIONING_POSE = [34, 400, 320, 90, 0, -90]

#LOOK_DOWN_POSITION = [34,440,700,180,0,-90] -> usa il corrispondente j per univocità
# DEFAULT POSITION FOR START AND END
LOOK_DOWN_POSITION_J_INIZIO = [-115.16134643554688,
 22.920495986938477,
 -106.5320816040039,
 -6.388057708740234,
 -90.00030517578125,
 244.83934020996094 - 360]

def local_start_marker(local_pose):
    """Calcola la posa del marker in coordinate locali rispetto alla posizione iniziale del casco."""
    # Applica la trasformazione inversa del marker iniziale per ottenere la posa relativa
    H_local = kin.compute_homogeneous_transform(local_pose)
    H_relative = np.linalg.inv(H_marker) @ H_local
    return kin.extract_pose_from_homogeneous(H_relative)


def global_final_marker(local_pose):
    """Calcola la posa finale del marker in coordinate globali a partire da una posa locale."""
    # Applica la trasformazione del marker iniziale per ottenere la posa globale
    H_relative = kin.compute_homogeneous_transform(local_pose)
    H_global = H_marker @ H_relative
    return kin.extract_pose_from_homogeneous(H_global)


HELMET_CENTER_GLOBAL = global_final_marker(local_start_marker(vbg.HELMET_CENTER_GLOBAL))


VISIERA_0 = global_final_marker(local_start_marker(vbg.VISIERA_0))




# ............................