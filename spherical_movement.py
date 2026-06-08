import kinematics_v2 as kin
from robot_control import RobotController

from Variables import HELMET_CENTER_GLOBAL

import cv2
import numpy as np
import time


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

    # 2. Controllo limite laterale: alpha deve essere compreso entro +/- 120 gradi
    if alpha > 105.0 or alpha < -95.0:
        return True

    # 3. Controllo zona posteriore (retro): più larga che alta
    # A alpha = 0 la soglia è 20, a alpha = 90 la soglia sale a 30
    beta_soglia_retro = 15.0 + 25.0 * abs((alpha / 90.0)**3)
    if beta < beta_soglia_retro:
        return True

    # 4. Controllo zona anteriore (fronte): più stretta che alta
    # A alpha = 0 la soglia massima ammessa è 110, a alpha = 90 sale a 140
    beta_soglia_fronte = 130.0 + 0 * (alpha / 90.0)**2
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
    """

    def actually_move(start_a, start_b, end_a, end_b, force_ptp=False):
        """
        Esegue fisicamente il movimento. Utilizza move_circle calcolando il midpoint,
        oppure ottimizza con PTP per spostamenti molto piccoli o forzati.
        """
        d_alpha = end_a - start_a
        d_beta = end_b - start_b

        p_end, r_end = kin.to_helmet_coordinates([radius, end_a, end_b], helmet_center)
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
            p_mid, r_mid = kin.to_helmet_coordinates([radius, mid_a, mid_b], helmet_center)
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

    if abs(start_radius - radius) > 20 and False:
        if input(f"  [WARNING] Raggio attuale {start_radius:.1f} mm differisce significativamente dal raggio target {radius:.1f} mm. \n Premere Invio per continuare comunque, o nope per annullare...").lower() == "nope":
            print("  Movimento annullato dall'utente.")
            return False
    
    start_alpha, start_beta = start_angles[1], start_angles[2]
    end_alpha, end_beta = end_sph_coord[1], end_sph_coord[2]

    # --- 2. Controllo Sicurezza Destinazione ---
    if angles_unsafe(end_alpha, end_beta):
        print(f"  [SKIP] Destinazione (alpha={end_alpha:.1f}°, beta={end_beta:.1f}°) fuori limiti sicurezza.")
        return False

    # --- 3. Controllo Sicurezza Traiettoria ---
    # Se il segmento taglia una zona pericolosa, deviamo passano per l'apice (0, 90) che è sempre sicuro.
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



if __name__ == "__main__":

    import kinematics_v2 as kin
    from robot_control import RobotController
    import Variables as vb

    IP_LAB = "192.168.19.22"
    IP_SIM = "127.0.0.1"
    controller = RobotController(ip_address=IP_SIM , default_position_j=vb.LOOK_DOWN_POSITION_J_INIZIO)
    controller.disconnect()
    controller.connect()

    controller.default_positioning()

    # Definizione della posizione di partenza nominale (Apice del casco)
    START_SPH = [300, 0, 90]

    # Generazione della lista dei casi critici (Stress Test) per la nuova convenzione
    esph_test = [
        {
            "coord": [300, 0, 120], 
            "desc": "Violazione frontale centrale: beta=120 supera la soglia limite di 110 ad alpha=0."
        },
        {
            "coord": [300, 90, 145], 
            "desc": "Violazione frontale laterale: beta=145 supera la tolleranza allargata di 140 ad alpha=90."
        },
        {
            "coord": [300, 0, 15], 
            "desc": "Violazione posteriore centrale: beta=15 interseca l'ingombro della base (limite 20 ad alpha=0)."
        },
        {
            "coord": [300, 100, 90], 
            "desc": "Violazione del limite laterale assoluto: alpha=100 eccede il dominio operativo di +/- 90 gradi."
        },
        {
            "coord": [300, 0, -5], 
            "desc": "Violazione geometrica assoluta: l'angolo di elevazione beta è inferiore a 0."
        },
        {
            "coord": [300, 90, 90], 
            "desc": "Posizionamento laterale destro (alpha=90, beta=90) preparatorio per i test di spostamento."
        },
        {
            "coord": [300, -90, 90], 
            "desc": "Ampio spostamento angolare (180 gradi su alpha): attivazione della suddivisione in due sottomovimenti sferici."
        },
        {
            "coord": [300, -90, 130], 
            "desc": "Posizionamento frontale-laterale sinistro (beta=130): zona sicura preparatoria per il test di deviazione."
        },
        {
            "coord": [300, 90, 130], 
            "desc": "Test traiettoria insicura: il movimento diretto verso il lato opposto attraversa il viso al centro (midpoint alpha=0, beta=130 non ammesso). Attesa deviazione via apice."
        },
        {
            "coord": [300, 0, 90], 
            "desc": "Ritorno all'apice: destinazione sicura e azzeramento posizionale."
        }
    ]

    # Inizializzazione del robot all'apice prima di iniziare i test
    p_obj, r_obj = kin.to_helmet_coordinates(START_SPH, vb.HELMET_CENTER_GLOBAL)
    ee_pose = kin.compute_ee_pose_for_tool_target(p_obj, r_obj, vb.CAMERA_POSE_EE)
    controller.move_ptp(ee_pose)
    print("Robot posizionato all'apice (alpha=0, beta=90). Inizio sequenza di test.\n")
    print("-" * 60)

    # Esecuzione iterativa dello stress test
    for case in esph_test:
        esph = case["coord"]
        desc = case["desc"]
        
        print(f"Test in esecuzione: {desc}")
        print(f"Target: alpha={esph[1]}°, beta={esph[2]}°")
        
        success = move_circle_spherical(
            controller=controller,
            radius=esph[0],
            end_sph_coord=esph,
            tool_pose_ee=vb.CAMERA_POSE_EE,
            helmet_center=vb.HELMET_CENTER_GLOBAL
        )
        
        if not success:
            print("Esito: RIFIUTATO (Comportamento atteso per coordinate non sicure).")
        else:
            print("Esito: COMPLETATO.")
            
        print("-" * 60)
