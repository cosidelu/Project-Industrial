import kinematics_v2 as kin
from robot_control import RobotController

from Variables import HELMET_CENTER_GLOBAL

import cv2
import numpy as np
import time

def variable_helmet_radius(alpha, beta, r_apex=380.0, r_side=300.0, r_back=300.0, r_front=450.0, r_min=300.0):
    """
    Raggio di lavoro variabile in funzione degli angoli sferici (alpha, beta).

    MOTIVAZIONE GEOMETRICA
    Il casco non è una sfera centrata su HELMET_CENTER_GLOBAL. Per raggiungere i
    difetti bassi (sotto il centro, dove alpha sforerebbe i +/-90) si abbassa il
    centro del casco; così facendo però l'apice reale del casco si avvicina al
    centro e, a raggio costante, la camera all'apice rischia la collisione.
    Si compensa allontanando la camera in alto (apice) e tenendola più vicina
    ai lati e sul retro: una superficie tipo ellissoide/paraboloide invece di una sfera.

    MODELLO (forma quadratica liscia)
        r(a, b) = r_apex
                  - (r_apex - r_side) * sin^2(alpha)   # riduzione verso i lati
                  - (r_apex - r_back) * cos^2(beta)    # riduzione verso retro
                  + (r_front - r_apex) * sin^2(beta-90) # aumento verso il fronte

    PUNTI DI ANCORAGGIO (con i default richiesti):
        - alpha=0,    beta=90  -> r_apex = 400   (apice del casco)
        - alpha=+-90, beta=90  -> r_side = 300   (lati)
        - alpha=0,    beta=0   -> r_back = 300   (retro)
        - alpha=0,    beta=180 -> r_front = 500  (fronte)

    NOTE
    - sin^2(alpha): 0 ad alpha=0, 1 ad alpha=+-90 -> sposta dal valore apice a quello laterale.
    - cos^2(beta):  0 a beta=90, 1 a beta=0   -> sposta dal valore apice a quello retro.
    - sin^2(beta-90): 0 a beta=90, 1 a beta=180 -> sposta dal valore apice a quello frontale.
    - Le due riduzioni si sommano: negli "angoli" (alpha alto E beta lontano da 90)
      il raggio può scendere parecchio, perciò viene applicato un clamp a r_min.

    :param alpha: angolo azimutale [gradi].
    :param beta: angolo di elevazione [gradi] (0 retro, 90 apice, 180 fronte).
    :return: raggio in mm.
    """
    a = np.radians(alpha)
    b = np.radians(beta)

    if beta <= 90.0:
        r_beta = r_apex - (r_apex - r_back) * np.cos(b) ** 2
        r = r_beta - (r_beta - r_side) * np.sin(a) ** 2
    else:
        r_alpha = r_apex - (r_apex - r_side) * np.sin(a) ** 2
        r_front_beta = r_apex + (r_front - r_apex) * np.sin(np.radians(beta - 90.0)) ** 2
        w = np.clip((beta - 90.0) / 90.0, 0.0, 1.0)
        r = (1.0 - w) * r_alpha + w * r_front_beta

    return max(r, r_min)


def make_radius_fn(radius):
    """
    Normalizza il parametro 'radius' in una funzione radius(alpha, beta).

    Accetta sia uno scalare (raggio costante: comportamento legacy) sia una
    funzione gia' dipendente dagli angoli (es. variable_helmet_radius), cosi'
    move_circle_spherical resta compatibile con tutte le chiamate esistenti.
    """
    if callable(radius):
        return radius
    return lambda alpha, beta: float(radius)


def angles_unsafe(alpha, beta):
    """
    Verifica se gli angoli sferici (alpha, beta) si trovano al di fuori dei limiti 
    di sicurezza o all'interno di zone di collisione (dietro, davanti, o laterale eccessivo).
    
    Convenzione corrente:
    - alpha: intervallo [-90, 90] gradi.
    - beta: intervallo [0, 180] gradi (0 = retro, 90 = sommità, 180 = fronte).
    
    :return: True se la configurazione è pericolosa/non ammessa, False altrimenti.
    """
    # 1. Verifica dei limiti geometrici del dominio della convenzione
    if not (0.0 <= beta <= 180.0):
        return True

    # 2. Controllo limite laterale: alpha deve essere compreso entro +/- 89 gradi
    if alpha > 89 or alpha < -89.0:
        return True

    # 3. Controllo zona posteriore (retro): più larga che alta
    # A alpha = 0 la soglia è 20, a alpha = +/-90 la soglia sale a 45
    beta_soglia_retro = 20 + 25.0 * (alpha / 90.0)**2
    if beta < beta_soglia_retro:
        return True

    # 4. Controllo zona anteriore (fronte): più stretta che alta
    # A alpha = 0 la soglia massima ammessa è 110; il termine in alpha è moltiplicato per zero,
    # quindi il valore rimane costante per tutti gli angoli laterali.
    beta_soglia_fronte = 110.0 + 0 * (alpha / 90.0)**2
    if beta > beta_soglia_fronte:
        return True

    return False

def is_trajectory_unsafe(start_alpha, start_beta, end_alpha, end_beta, steps=100):
    """
    Verifica la sicurezza dell'intera traiettoria sferica tramite discretizzazione.
    Restituisce False se tutti i punti intermedi sono sicuri, True altrimenti.
    """
    alphas = np.linspace(start_alpha, end_alpha, num=steps)
    betas = np.linspace(start_beta, end_beta, num=steps)
    
    for a, b in zip(alphas, betas):
        if angles_unsafe(a, b):
            return True
            
    return False

def move_circle_spherical(controller, end_sph_coord, radius, tool_pose_ee, helmet_center=HELMET_CENTER_GLOBAL, speed=300):
    """
    Esegue un movimento circolare da una posizione corrente a una posizione finale
    definita da angoli sferici (alpha, beta) attorno al casco.
    La cinematica è sicura dai gimbal lock poiché i poli (beta=0, beta=180) 
    sono esclusi dalle limitazioni di sicurezza.

    'radius' può essere uno scalare (raggio costante) oppure una funzione
    radius(alpha, beta) -> mm (es. variable_helmet_radius): in quest'ultimo caso
    il raggio viene ricalcolato per ogni punto della traiettoria.
    """

    # Normalizza il raggio in una funzione degli angoli (scalare -> costante)
    radius_fn = make_radius_fn(radius)

    def actually_move(start_a, start_b, end_a, end_b, force_ptp=False):
        """
        Esegue fisicamente il movimento. Utilizza move_circle calcolando il midpoint,
        oppure ottimizza con PTP per spostamenti molto piccoli o forzati.
        Il raggio viene valutato puntualmente tramite radius_fn(alpha, beta).
        """
        d_alpha = end_a - start_a
        d_beta = end_b - start_b

        r_end_val = radius_fn(end_a, end_b)
        p_end, r_end = kin.to_helmet_coordinates([r_end_val, end_a, end_b], helmet_center)
        ee_end = kin.compute_ee_pose_for_tool_target(p_end, r_end, tool_pose_ee=tool_pose_ee)

        if force_ptp or (d_alpha**2 + d_beta**2 < 5**2):
            if force_ptp:
                print(f"  [FORCE PTP] Esecuzione forzata verso (alpha={end_a:.1f}°, beta={end_b:.1f}°).")
            else:
                print(f"  [SHORT PATH] Distanza angolare < 5°. Esecuzione ottimizzata PTP verso (alpha={end_a:.1f}°, beta={end_b:.1f}°).")
            controller.move_ptp(ee_end, speed=speed)
        else:
            mid_a = np.mean([start_a, end_a])
            mid_b = np.mean([start_b, end_b])
            r_mid_val = radius_fn(mid_a, mid_b)
            p_mid, r_mid = kin.to_helmet_coordinates([r_mid_val, mid_a, mid_b], helmet_center)
            ee_mid = kin.compute_ee_pose_for_tool_target(p_mid, r_mid, tool_pose_ee=tool_pose_ee)
            
            print(f"  [CIRCLE] Movimento sferico: ({start_a:.1f}°, {start_b:.1f}°) -> ({end_a:.1f}°, {end_b:.1f}°)")
            controller.move_circle(ee_mid, ee_end, speed=speed)

    # --- 1. Calcolo coordinate attuali ---
    ee_pose = controller.robot.tcp_coord
    H_ee_to_glob = kin.create_homogeneous_matrix(ee_pose)
    tool_position_ee = tool_pose_ee[:3]
    
    tool_position_global = kin.homogeneous_trasform(H_ee_to_glob, tool_position_ee)
    start_angles = kin.to_helmet_angles(tool_position_global, helmet_center)

    start_radius = start_angles[0]
    start_alpha, start_beta = start_angles[1], start_angles[2]

    # Raggio target nella posizione angolare attuale (coerente con radius variabile)
    target_start_radius = radius_fn(start_alpha, start_beta)

    if abs(start_radius - target_start_radius) > 20 and False:
        if input(f"  [WARNING] Raggio attuale {start_radius:.1f} mm differisce significativamente dal raggio target {target_start_radius:.1f} mm. \n Premere Invio per continuare comunque, o nope per annullare...").lower() == "nope":
            print("  Movimento annullato dall'utente.")
            return False
    
    end_alpha, end_beta = end_sph_coord[1], end_sph_coord[2]

    # --- 2. Controllo Sicurezza Destinazione ---
    if angles_unsafe(end_alpha, end_beta):
        print(f"  [SKIP] Destinazione (alpha={end_alpha:.1f}°, beta={end_beta:.1f}°) fuori limiti sicurezza.")
        return False

    # --- 3. Controllo Sicurezza Traiettoria ---
    # Se il segmento taglia una zona pericolosa, deviamo passando per l'apice (0, 90) che è sempre sicuro.
    if is_trajectory_unsafe(start_alpha, start_beta, end_alpha, end_beta):
        print(f"  [SAFETY] Traiettoria non sicura. Deviazione tramite l'apice del casco.")
        apex_alpha, apex_beta = 0.0, 90.0
        actually_move(start_alpha, start_beta, apex_alpha, apex_beta)
        actually_move(apex_alpha, apex_beta, end_alpha, end_beta)
        return True

    # --- 4. Suddivisione Archi Ampi (Prevenzione Errori Controller) ---
    # Dato che alpha è limitato a +/- 90, l'arco massimo teorico è 180°.
    # Un solo split a metà garantisce che il robot debba gestire archi <= 90°.
    if abs(end_alpha - start_alpha) > 90 or abs(end_beta - start_beta) > 90:
        print("  [SPLIT] Traiettoria sferica ampia. Suddivisione in due segmenti.")
        mid_a = np.mean([start_alpha, end_alpha])
        mid_b = np.mean([start_beta, end_beta])
        actually_move(start_alpha, start_beta, mid_a, mid_b)
        actually_move(mid_a, mid_b, end_alpha, end_beta)
        return True

    # --- 5. Esecuzione Traiettoria Diretta ---
    actually_move(start_alpha, start_beta, end_alpha, end_beta)
    return True
