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
H_marker = kin.create_homogeneous_matrix(BASE_MARKER_POSE)



H_marker_inv = kin.create_homogeneous_matrix(BASE_MARKER_POSE, Inverse=True)
H_marker = kin.create_homogeneous_matrix(BASE_MARKER_POSE)

def local_start_marker(global_pose):
    p_local = kin.homogeneous_trasform(H_marker_inv, global_pose[:3]) #transform delle x,y,z
    return p_local.tolist() + global_pose[3:] #return con le 3 posizioni sovrascritte convertite in lista + le rotaz originali del p.to

def global_final_marker(local_pose):
    p_global = kin.homogeneous_trasform(H_marker, local_pose[:3])
    return p_global.tolist() + local_pose[3:]


# ============================================================
# POSIZIONI DI DEFAULT E JOINT
# ============================================================

# legacy defaults
#DEFAULT_POSITIONING_JOINTS = [-132 , 0, -133, -48 , -42, 180]
#DEFAULT_POSITIONING_POSE = [34, 400, 320, 90, 0, -90]

#LOOK_DOWN_POSITION = [34,440,700,180,0,-90] -> usa il corrispondente j per univocità
LOOK_DOWN_POSITION_J_INIZIO = [-115.16134643554688,
 22.920495986938477,
 -106.5320816040039,
 -6.388057708740234,
 -90.00030517578125,
 244.83934020996094 - 360]


# ============================================================
# CENTRO DEL CASCO
# ============================================================

HELMET_CENTER_GLOBAL = global_final_marker(local_start_marker(vbg.HELMET_CENTER_GLOBAL))


# ============================================================
# SEQUENZA APERTURA VISIERA
# ============================================================

VISIERA_0 = global_final_marker(local_start_marker(vbg.VISIERA_0))
VISIERA_1 = global_final_marker(local_start_marker(vbg.VISIERA_1))
VISIERA_2 = global_final_marker(local_start_marker(vbg.VISIERA_2))
VISIERA_3 = global_final_marker(local_start_marker(vbg.VISIERA_3))
VISIERA_4 = global_final_marker(local_start_marker(vbg.VISIERA_4))
VISIERA_5 = global_final_marker(local_start_marker(vbg.VISIERA_5))


# ============================================================
# ALLONTANAMENTO
# ============================================================

ALLONTANAMENTO       = global_final_marker(local_start_marker(vbg.ALLONTANAMENTO))
ALLONTANAMENTO_JOINT = vbg.ALLONTANAMENTO_JOINT  # joint — non trasformato


# ============================================================
# SOLE (chiusura visiera da sole)
# ============================================================

SOLE_1 = global_final_marker(local_start_marker(vbg.SOLE_1))
SOLE_2 = global_final_marker(local_start_marker(vbg.SOLE_2))
SOLE_3 = global_final_marker(local_start_marker(vbg.SOLE_3))
SOLE_4 = global_final_marker(local_start_marker(vbg.SOLE_4))


# ============================================================
# LUNA (pressione pulsante)
# ============================================================

LUNA_1 = global_final_marker(local_start_marker(vbg.LUNA_1))
LUNA_2 = global_final_marker(local_start_marker(vbg.LUNA_2))
LUNA_3 = global_final_marker(local_start_marker(vbg.LUNA_3))
LUNA_4 = LUNA_2  
LUNA_5 = global_final_marker(local_start_marker(vbg.LUNA_5))
LUNA_6 = global_final_marker(local_start_marker(vbg.LUNA_6))


# ============================================================
# SEQUENZA CHIUSURA VISIERA
# ============================================================

VISIERA_10_J = vbg.VISIERA_10_J  
VISIERA_11   = global_final_marker(local_start_marker(vbg.VISIERA_11))
VISIERA_12   = global_final_marker(local_start_marker(vbg.VISIERA_12))
VISIERA_13   = global_final_marker(local_start_marker(vbg.VISIERA_13))
VISIERA_14   = global_final_marker(local_start_marker(vbg.VISIERA_14))

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









# ............................