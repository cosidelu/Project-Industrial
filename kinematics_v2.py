import numpy as np

# Costanti globali per la configurazione del sistema
# ci rcordaimo che le pose stanno in liste perché il controller ragiona in liste
# i punti invece vanno direttamente in np arrays


def create_rot_matrix_zyx(r):
        """
        genera la matrice di rotazione a partire dalle rotazioni intrinseche ordinate zyx
        """
        rx, ry, rz = np.radians([r[0], r[1], r[2]])

        # Matrici di rotazione elementari
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx),  np.cos(rx)]
        ])

        R_y = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])

        R_z = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz),  np.cos(rz), 0],
            [0, 0, 1]
        ])
        
        # Rotazione totale (Convenzione Z-Y-X) 
        # Per andare da globale a locale l'ordine è ZYX -> ma noi stiamo andando da locale a globale -> XYZ
        # P_G = R_z @ R_y @ R_x @ P_L + T
        R = R_z @ R_y @ R_x
        return R

def rot_matrix_to_angles_zyx(R):
    """
    Estrae gli angoli di Cardano (rx, ry, rz) in gradi da una matrice di rotazione R.
    L'ordine atteso è lo stesso di create_rot_matrix_zyx (ZYX intrinseco o XYZ estrinseco).
    """
    # Calcoliamo la radice per verificare la presenza di singolarità (Gimbal Lock)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        # In caso di Gimbal Lock (ry = +/- 90 gradi)
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0

    r = np.array(np.degrees([rx, ry, rz]))

    return r

def create_homogeneous_matrix(pose, Inverse=False):
    """
    Genera una matrice di trasformazione omogenea 4x4 a partire da un vettore posa.
    NB: posa è una lista di python, non un vettore numpy

     SIGNIFICATO FISICO E MATEMATICO DELLA MATRICE H (4x4)
    
             [ ux  vx  wx | Tx ]
         H = [ uy  vy  wy | Ty ]
             [ uz  vz  wz | Tz ]
             [ -----------|----]
             [  0   0   0 |  1 ]
    
     1. SIGNIFICATO DEI VETTORI COLONNA:
        - Colonne 1, 2, 3: Rappresentano i versori (vettori unitari) degli assi
          locali x, y, z proiettati lungo gli assi del sistema globale.
        - Colonna 4 (T): Rappresenta il vettore posizione dell'origine del
          sistema di riferimento locale rispetto all'origine globale.
    
     2. FUNZIONE DELLA MOLTIPLICAZIONE (TRASFORMAZIONE DI COORDINATE):
        La moltiplicazione H @ P_L trasforma le coordinate di un punto P
        espresse nel sistema locale (P_L) nelle sue coordinate nel sistema
        globale (P_G) secondo la relazione:
    
        P_G = H @ P_L
    
        Analiticamente, l'operazione esegue la rotazione del punto tramite
        la sottomatrice 3x3 e la successiva traslazione tramite il vettore T.
    
    :param pose: Vettore [x, y, z, rx, ry, rz] con angoli espressi in gradi.
                 La convenzione di rotazione è Cardano Z -> Y -> X.
    :return: Matrice numpy 4x4.
    """
    R_total = create_rot_matrix_zyx(pose[3:])
    t = np.array(pose[:3])

    if Inverse:
        R_total = R_total.T  # Trasposta per l'inversione della rotazione
        t = -R_total @ t  # Inversione della traslazione
    
    # Inizializzazione della matrice identità 4x4
    H = np.eye(4)
    
    # Inserimento della sottomatrice di rotazione 3x3
    H[0:3, 0:3] = R_total
    
    # Inserimento del vettore di traslazione 3x1
    H[0:3, 3] = t
    
    return H

def homogeneous_trasform(H,point):
    # funziona sia se il punto è una lista che un array
    p_hom = np.array([point[0], point[1], point[2], 1.0])
    p_trans_hom = H @ p_hom

    return p_trans_hom[:-1]

'''
Per posizionare la telecamera nello spazio utilizziamo un nuovo set di coordinate sferiche:
- r: distanza dal centro del casco
- alpha: prima rotazione che decide quale "dorsale fare" (rispetto all'y globale) #prima era rispetto allo z ma creava un gibal lock nell'apice, scomodo
- beta: seconda rotazione rispetto all'asse x, traccia la "dorsale del casco"


Alcuni punti esempio:
- beta = 0, qualsiasi alpha : retro del casco -> è una posizone di lock ma è anche inaccessibile quindi bone
- alpha = 0, beta = 90: sommità del casco
- alpha > 0 : lato sx del casco
- alpha < 0 : lato dx del casco
- beta = 180, qualsiasi alpha : fronte del casco -> è un'altra posizione di gimble ma again bone perché è inaccessibile
'''


def to_helmet_coordinates(spherical_coords, helmet_center):
    """
    Converte coordinate sferiche (r, alpha, beta) in coordinate cartesiane 
    e angoli di orientamento (rx, ry, rz) in gradi secondo la convenzione 
    intrinseca Rz -> Ry' -> Rx''. L'asse z punta verso il centro del casco.
    
    Convenzione:
    - alpha: rotazione attorno all'asse y globale (0 sul piano ZY)
    - beta: elevazione rispeto al piano XY (0 sul retro, 90 sommità, 180 fronte)
    """
    r, alpha_deg, beta_deg = spherical_coords[0], spherical_coords[1], spherical_coords[2]
    
    alpha = np.radians(alpha_deg)
    beta = np.radians(beta_deg)
    
    # Trasformazione cinematica diretta delle coordinate spaziali
    x = r * np.sin(beta) * np.sin(alpha) + helmet_center[0]
    y = r * np.cos(beta)                 + helmet_center[1]
    z = r * np.sin(beta) * np.cos(alpha) + helmet_center[2]

    point = np.array([x, y, z])
    
    # Assegnazione diretta degli angoli di Eulero basata sulla terna Z-Y'-X''
    rx = beta_deg + 90.0
    ry = alpha_deg
    rz = 0.0
    
    # Normalizzazione degli angoli nell'intervallo standard [-180, 180] gradi
    rx = (rx + 180) % 360 - 180
    ry = (ry + 180) % 360 - 180
    
    rotations = np.array([rx, ry, rz])
    return point, rotations

def to_helmet_angles(defect_pos, helmet_center):
    """
    Calcola gli angoli sferici alpha e beta per un punto sul casco relativo al centro,
    invertendo la cinematica basata sul sistema a due assi y-x'.
    
    :param defect_pos: [x, y, z] posizione del difetto o target nello spazio.
    :param helmet_center: [x, y, z] centro del casco.
    :return: np.array([r, alpha_deg, beta_deg])
    """
    vector = np.array(defect_pos) - np.array(helmet_center)
    r = np.linalg.norm(vector)
    
    if r == 0:
        return np.array([0.0, 0.0, 0.0])
        
    dy = vector[1]
    
    # Inversione cinematica per l'angolo beta (basato sulla proiezione lungo y)
    # Ritorna valori nell'intervallo [0, 180] gradi, coerente con la cinematica del casco
    beta_rad = np.arccos(np.clip(dy / r, -1.0, 1.0))
    
    # Calcolo della proiezione sul piano ortogonale all'asse y
    r_xz = np.sqrt(vector[0]**2 + vector[2]**2)
    
    if r_xz > 0:
        # x = r_sin(beta)*sin(alpha) e z = r_sin(beta)*cos(alpha)
        sin_alpha = vector[0] / r_xz
        cos_alpha = vector[2] / r_xz
        alpha_rad = np.arctan2(sin_alpha, cos_alpha)
    else:
        # Singolarità ai poli (retro o fronte puro), alpha non influisce sulla posizione
        alpha_rad = 0.0
    
    alpha_deg = np.degrees(alpha_rad)
    beta_deg = np.degrees(beta_rad)
    
    return np.array([r, alpha_deg, beta_deg])

def compute_ee_pose_for_tool_target(p_obj, r_obj, tool_pose_ee):
    """
    Data la posa target per l'ORIGINE del tool (p_obj, r_obj),
    calcola la posa dell'End Effector necessaria affinché il tool
    raggiunga esattamente quella posa.

    Questa funzione generalizza il calcolo per gestire offset di traslazione e rotazione
    del tool rispetto all'end-effector, usando matrici di trasformazione omogenea.

    L'equazione risolta è: H_ee = H_target @ (H_tool_ee)^-1

    Input:
    - p_obj: np.array([x, y, z]) posizione target del frame del tool nel globale [mm]
    - r_obj: np.array([rx, ry, rz]) rotazioni target del frame del tool in gradi.
    - tool_pose_ee: lista [x, y, z, rx, ry, rz] dell'offset del tool rispetto all'EE.

    Output:
    - lista Python [x, y, z, rx, ry, rz] della posa richiesta per l'EE.
    """
    # 1. Matrice omogenea dell'obiettivo (dove vogliamo che si trovi il tool)
    H_obj = create_homogeneous_matrix(p_obj.tolist() + r_obj.tolist())
    
    # 2. Matrice omogenea inversa dell'offset del tool rispetto all'EE
    H_tool_ee_inv = create_homogeneous_matrix(tool_pose_ee, Inverse=True)
    
    # 3. La posa dell'EE è data da: H_ee_global = H_obj * (H_tool_ee)^-1
    H_ee_global = H_obj @ H_tool_ee_inv
    
    # 4. Estrazione della traslazione e riconversione della matrice di rotazione in angoli
    t_ee = H_ee_global[0:3, 3]
    R_ee = H_ee_global[0:3, 0:3]
    r_ee = rot_matrix_to_angles_zyx(R_ee)
    
    return t_ee.tolist() + r_ee.tolist()

# =====================================================================
# BLOCCO DI VALIDAZIONE UNITARIA (UNIT TEST)
# =====================================================================
if __name__ == "__main__":
    print("ciao")
    
    helmet_center = [0, 0, 0]
    point = [1,1, 1]

    sph = [1, 45,90]

    angles = to_helmet_angles(point, helmet_center)
    pos, rot = to_helmet_coordinates(sph, helmet_center)
    print(pos)
    
