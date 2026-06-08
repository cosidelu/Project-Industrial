import kinematics_v2 as kin
from robot_control import RobotController
from defects_id_wrapper import (take_defects_global,    # wrapper to take photos
                                duplicate_filter)       # filter to remove duplicates
from camera_scripts_v2 import  (draw_multiple_debug,    # function to draw debug info on the image
                                init_zed,               # function to initialize the ZED camera
                                WINDOW_NAME_RGB, WINDOW_NAME_MASK)

from Variables import HELMET_CENTER_GLOBAL, CAMERA_POSE_EE, MARKER_POSE_EE

import cv2
import numpy as np
import time

# =====================================================
# LOCAL VARIABLES
# =====================================================

INSPECTION_RADIUS = 400
HUB_ANGLES = (0, 90)

# Sequenza di waypoint per l'ispezione.
# Ogni elemento è una tupla (alpha_deg, beta_deg, is_hub).
INSPECTION_ANGLES = [
    (0,    90,  True),        # hub iniziale — PTP
    (0,    90,  False),       # scatto 1: dall'alto
    (0,    30,  False),       # scatto 2: metà dietro
    (90,   30,  False),       # scatto 3: sinistra
    (-90,  30,  False),       # scatto 4: destra
    (0,    90,  True),        # hub finale
]

# Velocità di avvicinamento al casco per la marcatura [mm/s]
MARKING_SPEED = 50

# Velocità di movimento circolare di ispezione [mm/s]
INSPECTION_SPEED = 300

# Distanza minima garantita dal centro del casco durante l'approccio al marker [mm]
# Serve ad evitare collisioni nella fase di avvicinamento al difetto
MIN_APPROACH_RADIUS = 400

# Raggio di posizionamento per la fase di raffinamento 3D [mm].
# Il robot si porta a questa distanza dal CENTRO del casco, nella direzione radiale
# esatta del difetto (stesso alpha, stesso beta dell'ispezione globale).
# Tenuto a 400 mm per sicurezza: abbastanza vicino da inquadrare il difetto al centro
# dell'immagine con buona risoluzione, abbastanza lontano da non rischiare collisioni.
# Può essere ridotto se il casco lo permette.
CLOSE_INSPECTION_RADIUS = 400

# Numero di foto ravvicinate da scattare per ciascun difetto nella fase di raffinamento
N_CLOSE_SHOTS = 10

# Soglia per rimuovere i falsi duplicati tra le foto dell'ispezione globale [mm]
DUPLICATE_THRESHOLD = 25.0

# Soglia di associazione per il raffinamento: distanza massima accettabile tra la posizione
# stimata del difetto (pos3d_global) e la posizione rilevata in ciascuno scatto ravvicinato
# per considerarlo valido ai fini del raffinamento.

ASSOCIATION_THRESHOLD = 30.0 # mm

# =====================================================
# Spherical motion functions
# =====================================================

INSPECTION_RADIUS = 300

# =====================================================
# INSPECTION
# =====================================================

def inspection_phase(controller, 
                     zed, runtime, image_zed, point_cloud,
                     helmet_center, 
                     inspection_angles, radius):
    """
    Esegue la fase di ispezione globale:
    1. PTP al primo punto (hub).
    2. `move_circle_spherical` per muoversi tra i punti di scatto, usando override se necessario.
    3. Ad ogni waypoint di scatto, acquisisce e processa i difetti.
    4. Filtra i duplicati e restituisce la lista univoca.

    Input:
    - controller: istanza di RobotController
    - zed, runtime, image_zed, point_cloud: oggetti ZED
    - helmet_center: centro del casco nel frame globale [mm]
    - inspection_angles: lista di tuple (alpha, beta, is_hub)
    - radius: raggio di ispezione [mm]

    Output:
    - lista univoca di oggetti defect con pos3d_global valorizzato.
    """
    H_cam_to_ee = kin.create_homogeneous_matrix(CAMERA_POSE_EE)
    all_defects = []

    n_waypoints = len(inspection_angles)
    n_shots = sum(1 for ang in inspection_angles if not ang[2]) # Conta solo i non-hub
    print(f"Inizio ispezione globale: {n_waypoints} waypoint totali, {n_shots} scatti previsti.")

    shot_count = 0
    for i, (alpha_deg, beta_deg, is_hub) in enumerate(inspection_angles):
        if is_hub:
            tag = "HUB (transito, nessuna foto)"
        else:
            shot_count += 1
            tag = f"SCATTO {shot_count}/{n_shots} — alpha={alpha_deg}°, beta={beta_deg}°"
        
        print(f"  [{i+1}/{n_waypoints}] {tag}")

        end_sph = np.array([radius, alpha_deg, beta_deg])

        if i == 0:
            p_first, r_first = kin.to_helmet_coordinates(end_sph, helmet_center)
            ee_first_pose = kin.compute_ee_pose_for_tool_target(p_first, r_first, tool_pose_ee=CAMERA_POSE_EE)
            print(f"    PTP verso il primo punto di ispezione (hub)...")
            controller.move_ptp(ee_first_pose, speed=INSPECTION_SPEED)
        else:
            print(f"    Movimento sferico verso il punto successivo...")
            move_circle_spherical(controller, end_sph, radius, CAMERA_POSE_EE, helmet_center, speed=INSPECTION_SPEED)

        if is_hub:
            continue

        time.sleep(1) 

        # ---- Acquisizione e calcolo coordinate globali ----
        ee_pose_live = controller.robot.tcp_coord
        H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
        H_cam_to_global = H_ee_to_global @ H_cam_to_ee

        defect_list, bgr_image = take_defects_global(runtime, zed, image_zed, point_cloud,
                                                     H_cam_to_global=H_cam_to_global)

        # ---- Rendering debug ----
        debug_img, mask_bgr = draw_multiple_debug(bgr_image, defect_list, show_global=True)
        cv2.imshow(WINDOW_NAME_MASK, mask_bgr)
        cv2.imshow(WINDOW_NAME_RGB, debug_img)
        cv2.waitKey(100)

        all_defects.extend(defect_list)
        print(f"    Difetti rilevati in questo scatto: {len(defect_list)}")

    print(f"\nIspezione globale completata. Difetti totali pre-filtraggio: {len(all_defects)}")
    unique_defects = duplicate_filter(all_defects, distance_threshold=DUPLICATE_THRESHOLD)
    print(f"Difetti univoci dopo filtraggio: {len(unique_defects)}")
    return unique_defects

def safe_transit_via_hub(controller, helmet_center, tool_pose_ee=CAMERA_POSE_EE, radius=INSPECTION_RADIUS,
                         safety_radius=400.0, speed=INSPECTION_SPEED):
    """
    Porta il robot al punto hub sicuro (sopra il casco) tramite un movimento sferico.
    Da chiamare prima di ogni movimento verso un difetto per garantire una
    traiettoria sicura.

    Input:
    - controller: istanza di RobotController
    - helmet_center: centro del casco nel globale [mm]
    - tool_pose_ee: posa del tool da utilizzare (es. CAMERA_POSE_EE o MARKER_POSE_EE)
    - radius: raggio della sfera su cui posizionare l'hub [mm]
    - safety_radius: raggio della sfera di sicurezza da non attraversare [mm]
    - speed: velocità del movimento [mm/s]
    """
    hub_alpha, hub_beta = HUB_ANGLES
    hub_sph = np.array([radius, hub_alpha, hub_beta])

    current_tcp = np.array(controller.robot.tcp_coord[:3])
    dist_from_helmet = np.linalg.norm(current_tcp - helmet_center)

    if dist_from_helmet < safety_radius:
        print(f"  [SAFETY] Posizione attuale a {dist_from_helmet:.0f}mm dal centro casco "
              f"(< {safety_radius:.0f}mm). Transito forzato per l'hub prima di procedere.")
    else:
        print(f"  [TRANSIT] Transito per hub prima del prossimo difetto "
              f"(distanza attuale dal casco: {dist_from_helmet:.0f}mm).")

    move_circle_spherical(controller, hub_sph, radius, tool_pose_ee, helmet_center, speed=speed)

# =====================================================
# MARKING
# =====================================================

def refine_defect_position(controller, zed, runtime, image_zed, point_cloud,
                           defect_obj, helmet_center,
                           close_radius=CLOSE_INSPECTION_RADIUS,
                           n_shots=N_CLOSE_SHOTS):
    """ Raffina la posizione del difetto avvicinando la camera. """
    if defect_obj.pos3d_global is None:
        raise ValueError("Defect object must have pos3d_global defined for refinement.")

    H_cam_to_ee = kin.create_homogeneous_matrix(CAMERA_POSE_EE)

    spher_coords = kin.to_helmet_angles(defect_obj.pos3d_global, helmet_center)
    _, alpha_deg, beta_deg = spher_coords[0], spher_coords[1], spher_coords[2]

    print(f"    Posizionamento per raffinamento: r={close_radius}mm, alpha={alpha_deg:.1f}°, beta={beta_deg:.1f}°")
    end_sph = np.array([close_radius, alpha_deg, beta_deg])
    move_circle_spherical(controller, end_sph, close_radius, CAMERA_POSE_EE, helmet_center, speed=INSPECTION_SPEED)

    time.sleep(1)

    ee_pose_live = controller.robot.tcp_coord
    H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
    H_cam_to_global = H_ee_to_global @ H_cam_to_ee

    refined_positions = []
    for shot_idx in range(n_shots):
        defect_list_shot, bgr_image = take_defects_global(
            runtime, zed, image_zed, point_cloud, H_cam_to_global
        )

        best_match = None
        best_dist = float("inf")

        for d in defect_list_shot:
            if d.pos3d_global is None: continue
            dist = np.linalg.norm(d.pos3d_global - defect_obj.pos3d_global)
            if dist < best_dist:
                best_dist = dist
                best_match = d

        if best_match is not None and best_dist < ASSOCIATION_THRESHOLD:
            refined_positions.append(best_match.pos3d_global)
            found_str = f"trovato (dist={best_dist:.1f}mm)"
        else:
            found_str = "non trovato"

        debug_img, mask_bgr = draw_multiple_debug(bgr_image, defect_list_shot, show_global=True)
        cv2.imshow(WINDOW_NAME_MASK, mask_bgr)
        cv2.imshow(WINDOW_NAME_RGB, debug_img)
        cv2.waitKey(50)

        print(f"      Scatto {shot_idx + 1}/{n_shots}: {found_str}")

    if not refined_positions:
        print(f"    [ATTENZIONE] Nessun scatto valido per questo difetto.")
        return False

    mean_refined = np.mean(np.array(refined_positions), axis=0)
    old_pos = defect_obj.pos3d_global.copy()
    defect_obj.pos3d_global = mean_refined

    delta = np.linalg.norm(mean_refined - old_pos)
    print(f"    Raffinamento completato ({len(refined_positions)}/{n_shots} scatti validi).")
    print(f"    Posizione: {np.round(old_pos, 1)} → {np.round(mean_refined, 1)} mm (delta={delta:.1f} mm)")

    return True


def mark_defect(controller, defect_obj, helmet_center,
                min_approach_radius=MIN_APPROACH_RADIUS,
                marking_speed=MARKING_SPEED):
    """ Esegue il movimento finale verso il casco per marcare il difetto. """
    if defect_obj.pos3d_global is None:
        print("  [SKIP] Difetto senza coordinate globali, salto marcatura.")
        return

    pos = defect_obj.pos3d_global
    print(f"  Avvio marcatura in {np.round(pos, 1)} mm")

    spher_coords = kin.to_helmet_angles(pos, helmet_center)
    r_def, alpha_deg, beta_deg = spher_coords[0], spher_coords[1], spher_coords[2]

    p_obj, r_obj = kin.to_helmet_coordinates([r_def, alpha_deg, beta_deg], helmet_center)
    ee_marking_pose = kin.compute_ee_pose_for_tool_target(p_obj, r_obj, tool_pose_ee=MARKER_POSE_EE)

    r_approach = max(r_def + 50.0, float(min_approach_radius))
    
    print(f"    Approccio a r={r_approach:.0f}mm...")
    approach_sph = np.array([r_approach, alpha_deg, beta_deg])
    move_circle_spherical(controller, approach_sph, r_approach, MARKER_POSE_EE, helmet_center, speed=marking_speed)

    print(f"    Avanzamento al difetto (marcatura)...")
    controller.move_line(ee_marking_pose, speed=marking_speed // 2)

    p_approach_obj, _ = kin.to_helmet_coordinates([r_approach, alpha_deg, beta_deg], helmet_center)
    ee_approach_pose = kin.compute_ee_pose_for_tool_target(p_approach_obj, r_obj, tool_pose_ee=MARKER_POSE_EE)

    print(f"    Arretramento al punto di approccio...")
    controller.move_line(ee_approach_pose, speed=marking_speed)


def refine_and_mark_phase(controller, zed, runtime, image_zed, point_cloud, unique_defects, helmet_center):
    """ Esegue l'intero workflow: viaggia al difetto, affina le misure, timbra se valido. """
    print(f"\nInizio ciclo di Raffinamento e Marcatura per {len(unique_defects)} difetti.")
    confirmed_marked = 0

    for i, d in enumerate(unique_defects):
        print(f"\n--- Elaborazione Difetto {i+1}/{len(unique_defects)} ---")
        print(f"  Posizione stimata globale: {np.round(d.pos3d_global, 1)} mm")
        
        try:
            safe_transit_via_hub(controller, helmet_center, tool_pose_ee=CAMERA_POSE_EE)
            success = refine_defect_position(controller, zed, runtime, image_zed, point_cloud, d, helmet_center)

            if success:
                safe_transit_via_hub(controller, helmet_center, tool_pose_ee=MARKER_POSE_EE)
                mark_defect(controller, d, helmet_center)
                confirmed_marked += 1
            else:
                print(f"  [SCARTATO] Difetto ignorato per mancata conferma durante il refining.")
        except Exception as e:
            print(f"  [ERRORE] Errore sul difetto {i+1}: {e}")

    print(f"\nOperazione conclusa. Difetti marcati con successo: {confirmed_marked}/{len(unique_defects)}")



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

    # Generazione della lista dei casi critici (Stress Test)
    esph_test = [
        {
            "coord": [300, 180, 45], 
            "desc": "Violazione frontale diretta: zona viso con beta < 60."
        },
        {
            "coord": [300, 150, 60], 
            "desc": "Condizione di bordo limite: sul margine esatto della zona frontale di sicurezza."
        },
        {
            "coord": [300, 10, 15], 
            "desc": "Violazione posteriore/bassa: collisione con la base (alpha^2 + beta^2 < 400)."
        },
        {
            "coord": [300, -179, 85], 
            "desc": "Ampio spostamento angolare (> 80 gradi) con alpha negativo: test suddivisione while loop."
        },
        {
            "coord": [300, 179, 85], 
            "desc": "Ampio spostamento angolare (> 80 gradi) con alpha positivo: test suddivisione while loop simmetrico."
        },
        {
            "coord": [300, 0, -5], 
            "desc": "Violazione del limite inferiore assoluto di beta."
        },
        {
            "coord": [300, 0, 90], 
            "desc": "Destinazione coincidente con la partenza (Gimbal lock bypass)."
        },
        {
            "coord": [300, 120, 15], 
            "desc": "Posizione sicura preparatoria per il test di traiettoria."
        },
        {
            "coord": [300, -120, 15], 
            "desc": "Test di traiettoria insicura: transito attraverso alpha=0 e beta=15 (zona base vietata)."
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
