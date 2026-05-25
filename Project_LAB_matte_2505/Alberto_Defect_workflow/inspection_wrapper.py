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

# Raggio di ispezione generale (scansione globale del casco) [mm]
INSPECTION_RADIUS = 400

# Angoli sferici per l'ispezione del casco.
# Il robot parte dall'hub (sopra il casco) con un PTP, poi percorre i 5 punti
# di scatto uno dopo l'altro con move_circle (senza mai tornare all'hub).
# Al termine ritorna all'hub con un ultimo move_circle.
# Ogni elemento è una tripla (alpha_deg, beta_deg, is_hub).
# I waypoint hub (is_hub=True) NON producono acquisizioni.
HUB_ANGLES = (0, 90)

# Sequenza di waypoint per l'ispezione.
# Ogni elemento è una tupla (alpha_deg, beta_deg, is_hub, mid_override).
# - mid_override: tupla (alpha, beta) del mid-point da usare per il move_circle
#   verso QUESTO waypoint. Se None, il mid viene calcolato come media angolare
#   con il waypoint precedente.
#   Va usato esplicitamente quando la media angolare porta il robot
#   su una traiettoria indesiderata (es. transizioni di 180° in alpha,
#   dove la media è ambigua e il robot potrebbe passare dalla parte sbagliata).
INSPECTION_ANGLES = [
    (0,    90,  True,  None),        # hub iniziale — PTP, mid ignorato
    (0,    90,  False, None),        # scatto 1: dall'alto — media (0,90) OK
    (0,    45,  False, None),        # scatto 2: fronte — media (0,67) OK, scende dolcemente
    (90,   45,  False, None),        # scatto 3: destra — media (45,45) OK
    (-90,  45,  False, (0, 85)),     # scatto 4: sinistra — OVERRIDE: beta=85 forza il
                                     #   robot a salire quasi fino al polo prima di scendere
                                     #   a sinistra, evitando il semicerchio orizzontale
                                     #   davanti al casco che la media (0,45) produrrebbe.
    (180,  45,  False, None),        # scatto 5: dietro — media (45,45) OK
    (0,    90,  True,  None),        # hub finale — media (90,67) OK, risale a destra
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
# INSPECTION
# =====================================================

def inspection_phase(controller, 
                     zed, runtime, image_zed, point_cloud,
                     helmet_center, 
                     inspection_angles, radius):
    """
    Esegue la fase di ispezione globale con percorso circolare continuo:

    1. Costruisce i waypoint sferici con build_inspection_waypoints.
    2. PTP dall'hub iniziale (sopra il casco).
    3. move_circle diretto da un punto di scatto al successivo, senza tornare
       all'hub tra uno scatto e l'altro. Il mid-point sferico garantisce che
       l'arco resti sulla sfera di ispezione senza avvicinarsi al casco.
    4. Ad ogni waypoint di scatto:
       - Acquisisce immagine e point cloud (take_defects)
       - Legge tcp_coord in real-time per costruire H_cam_to_global
       - Calcola pos3d_global per ogni difetto rilevato
       - Accumula nella lista globale
    5. move_circle finale verso l'hub di ritorno.
    6. Filtra duplicati e restituisce la lista univoca.

    Input:
    - controller: istanza di RobotController (connesso, già in default_positioning)
    - zed, runtime, image_zed, point_cloud: oggetti ZED da init_zed()
    - helmet_center: np.array([x, y, z]) centro del casco nel globale [mm]
    - inspection_angles: lista di tuple (alpha_deg, beta_deg, is_hub)
    - radius: raggio di ispezione [mm]

    Output:
    - lista univoca di oggetti defect con pos3d_global valorizzato (duplicati rimossi)
    """
    H_cam_to_ee = kin.create_homogeneous_matrix(CAMERA_POSE_EE)
    all_defects = []

    waypoints = build_inspection_waypoints(inspection_angles, helmet_center, radius)
    n_shots = sum(1 for wp in waypoints if not wp["is_hub"])

    print(f"Inizio ispezione globale: {len(waypoints)} waypoint totali, {n_shots} scatti previsti.")

    # PTP dall'hub iniziale: traiettoria sicura dalla posizione di rest.
    # Tutti i waypoint successivi si raggiungono con move_circle diretto,
    # senza mai tornare all'hub intermedio.
    print(f"  [0/{len(waypoints)}] PTP dalla posizione di rest all'hub iniziale...")
    controller.move_ptp(waypoints[0]["end"], speed=INSPECTION_SPEED)

    shot_count = 0
    for i, wp in enumerate(waypoints):
        is_hub = wp["is_hub"]
        alpha_deg, beta_deg, _, __ = inspection_angles[i]

        if is_hub:
            tag = "HUB (transito, nessuna foto)"
        else:
            shot_count += 1
            tag = f"SCATTO {shot_count}/{n_shots} — alpha={alpha_deg}°, beta={beta_deg}°"

        print(f"  [{i+1}/{len(waypoints)}] {tag}")

        # Tutti i waypoint successivi al primo si raggiungono con arco circolare
        # passando per il mid-point pre-calcolato. La camera rimane sempre
        # orientata verso il centro del casco durante il transito.
        if i > 0:
            controller.move_circle(wp["mid"], wp["end"], speed=INSPECTION_SPEED)

        # I waypoint hub sono solo di transito: nessuna acquisizione
        if is_hub:
            continue

        #waits a sec to stabilize the camera before taking picture
        time.sleep(1) 

        # ---- Fase 4: acqusizione difetti con coordinate globali con posa EE letta in real-time ----
        # tcp_coord viene letto DOPO il completamento del move_circle (bloccante),
        # quindi rappresenta la posizione reale del robot al momento dello scatto.
        ee_pose_live = controller.robot.tcp_coord
        H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
        H_cam_to_global = H_ee_to_global @ H_cam_to_ee

        defect_list, bgr_image = take_defects_global(runtime, zed, image_zed, point_cloud,
                                                     H_cam_to_global= H_cam_to_global)


        # ---- Rendering debug ----
        debug_img, mask_bgr = draw_multiple_debug(bgr_image, defect_list, show_global=True)
        cv2.imshow(WINDOW_NAME_MASK, mask_bgr)
        cv2.imshow(WINDOW_NAME_RGB, debug_img)
        cv2.waitKey(100) #waits 100 seconds before proceeding to the next

        all_defects.extend(defect_list)
        print(f"    Difetti rilevati in questo scatto: {len(defect_list)}")

    print(f"\nIspezione globale completata. Difetti totali pre-filtraggio: {len(all_defects)}")

    unique_defects = duplicate_filter(all_defects, distance_threshold=DUPLICATE_THRESHOLD)
    print(f"Difetti univoci dopo filtraggio: {len(unique_defects)}")

    return unique_defects

def safe_transit_via_hub(controller, helmet_center, radius=INSPECTION_RADIUS,
                         safety_radius=400.0, speed=INSPECTION_SPEED):
    """
    Verifica se la posizione attuale dell'EE è troppo vicina alla sfera di sicurezza
    attorno al casco (raggio safety_radius dal centro del casco). Se sì, oppure in
    ogni caso come transito inter-difetto, porta il robot al punto hub sicuro
    (direttamente sopra il casco, HUB_ANGLES) prima di procedere alla destinazione
    successiva. Da chiamare PRIMA di ogni move_ptp che manda il robot verso un difetto.

    Il punto hub è garantito essere a distanza `radius` dal centro del casco,
    quindi fuori dalla sfera di sicurezza a patto che radius >= safety_radius.

    Input:
    - controller: istanza di RobotController
    - helmet_center: np.array([x, y, z]) centro del casco nel globale [mm]
    - radius: raggio della sfera su cui posizionare l'hub [mm]
    - safety_radius: raggio della sfera di sicurezza da non attraversare [mm]
    - speed: velocità del movimento verso l'hub [mm/s]
    """
    # Calcola la posa EE per il punto hub sopra il casco
    hub_alpha, hub_beta = HUB_ANGLES
    p_hub, r_hub = kin.to_helmet_coordinates([radius, hub_alpha, hub_beta], helmet_center)
    ee_hub_pose = kin.compute_ee_pose_for_tool_target(p_hub, r_hub, tool_pose_ee=CAMERA_POSE_EE)

    # Legge la posizione TCP corrente e controlla la distanza dal centro del casco
    current_tcp = np.array(controller.robot.tcp_coord[:3])
    dist_from_helmet = np.linalg.norm(current_tcp - helmet_center)

    if dist_from_helmet < safety_radius:
        print(f"  [SAFETY] Posizione attuale a {dist_from_helmet:.0f}mm dal centro casco "
              f"(< {safety_radius:.0f}mm). Transito forzato per l'hub prima di procedere.")
    else:
        print(f"  [TRANSIT] Transito per hub prima del prossimo difetto "
              f"(distanza attuale dal casco: {dist_from_helmet:.0f}mm).")

    controller.move_ptp(ee_hub_pose, speed=speed)

def build_inspection_waypoints(inspection_angles, helmet_center, radius):
    """
    Genera la sequenza di waypoint (pose EE) per l'ispezione circolare.

    Per ogni waypoint calcola:
    - La posa EE finale ('end'): camera posizionata sul punto sferico e orientata
      perpendicolarmente verso il centro del casco.
    - Il mid-point ('mid'): usato da move_circle per definire l'arco geometrico.
      Di default è la media angolare sferica con il waypoint precedente, che
      mantiene il robot sulla sfera di raggio `radius`.
      Se la tupla contiene un mid_override (alpha, beta), quel punto viene usato
      al posto della media — necessario per transizioni dove la media angolare
      porta il robot su una traiettoria indesiderata.
    - Il flag 'is_hub'.

    CASO CRITICO — transizione destra (alpha=+90) -> sinistra (alpha=-90):
    I due punti distano 180° in alpha. La media aritmetica dà alpha=0 (fronte),
    quindi il robot percorrerebbe un semicerchio orizzontale davanti al casco
    invece di passare sopra. Il mid_override con beta alto (es. 85°) forza
    il robot a salire verso il polo superiore prima di scendere a sinistra.

    Input:
    - inspection_angles: lista di tuple (alpha_deg, beta_deg, is_hub, mid_override)
                         mid_override è (alpha, beta) oppure None
    - helmet_center: np.array([x, y, z]) centro del casco nel frame globale [mm]
    - radius: raggio della sfera di ispezione [mm]

    Output:
    - lista di dizionari con chiavi:
        'end'    : lista [x,y,z,rx,ry,rz] — posa EE del waypoint
        'mid'    : lista [x,y,z,rx,ry,rz] — posa EE del mid-point per move_circle
        'is_hub' : bool
    """
    waypoints = []
    for i, entry in enumerate(inspection_angles):
        alpha_deg, beta_deg, is_hub, mid_override = entry

        # Posa EE del waypoint corrente
        p_obj, r_obj = kin.to_helmet_coordinates(
            [radius, alpha_deg, beta_deg], helmet_center
        )
        ee_end = kin.compute_ee_pose_for_tool_target(p_obj, r_obj, tool_pose_ee = CAMERA_POSE_EE)

        # Mid-point: override esplicito oppure media angolare con il precedente
        if mid_override is not None:
            # Override esplicito: il punto sferico è già definito nella lista.
            mid_alpha, mid_beta = mid_override
        else:
            # Media angolare sferica: mantiene il robot sulla sfera di ispezione.
            prev_alpha = inspection_angles[i - 1][0]
            prev_beta  = inspection_angles[i - 1][1]
            mid_alpha  = (prev_alpha + alpha_deg) / 2.0
            mid_beta   = (prev_beta  + beta_deg)  / 2.0

        p_mid, r_mid = kin.to_helmet_coordinates(
            [radius, mid_alpha, mid_beta], helmet_center
        )
        ee_mid = kin.compute_ee_pose_for_tool_target(p_mid, r_mid, tool_pose_ee=CAMERA_POSE_EE)

        waypoints.append({"end": ee_end, "mid": ee_mid, "is_hub": is_hub})

    return waypoints

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

    # 1. Trova gli angoli della direzione radiale del difetto
    spher_coords = kin.to_helmet_angles(defect_obj.pos3d_global, helmet_center)
    r_def, alpha_deg, beta_deg = spher_coords[0], spher_coords[1], spher_coords[2]

    # 2. Posa d'osservazione frontale
    p_close, r_close = kin.to_helmet_coordinates([close_radius, alpha_deg, beta_deg], helmet_center)
    ee_close_pose = kin.compute_ee_pose_for_tool_target(p_close, r_close, tool_pose_ee=CAMERA_POSE_EE)

    print(f"    Posizionamento per raffinamento: r={close_radius}mm, alpha={alpha_deg:.1f}°, beta={beta_deg:.1f}°")
    controller.move_ptp(ee_close_pose, speed=INSPECTION_SPEED)

    # Attendi la stabilizzazione fisica del braccio per non avere foto mosse
    time.sleep(1)

    # 3. Aggiorna la matrice per l'allineamento dopo aver mosso il robot
    ee_pose_live = controller.robot.tcp_coord
    H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
    H_cam_to_global = H_ee_to_global @ H_cam_to_ee

    refined_positions = []
    association_threshold = ASSOCIATION_THRESHOLD

    # 4. Multi-scatto per raffinare
    for shot_idx in range(n_shots):
        defect_list_shot, bgr_image = take_defects_global(
            runtime, zed, image_zed, point_cloud, H_cam_to_global
        )

        best_match = None
        best_dist = float("inf")

        for d in defect_list_shot:
            if d.pos3d_global is None:
                continue
            dist = np.linalg.norm(d.pos3d_global - defect_obj.pos3d_global)
            if dist < best_dist:
                best_dist = dist
                best_match = d

        if best_match is not None and best_dist < association_threshold:
            refined_positions.append(best_match.pos3d_global)
            found_str = f"trovato (dist={best_dist:.1f}mm)"
        else:
            found_str = "non trovato"

        debug_img, mask_bgr = draw_multiple_debug(bgr_image, defect_list_shot, show_global=True)
        cv2.imshow(WINDOW_NAME_MASK, mask_bgr)
        cv2.imshow(WINDOW_NAME_RGB, debug_img)
        cv2.waitKey(50)

        print(f"      Scatto {shot_idx + 1}/{n_shots}: {found_str}")

    if len(refined_positions) == 0:
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
    """ Esegue il movimento finale verso il casco per marcare il difetto con il tool Marker. """
    if defect_obj.pos3d_global is None:
        print("  [SKIP] Difetto senza coordinate globali, salto marcatura.")
        return

    pos = defect_obj.pos3d_global
    print(f"  Avvio marcatura in {np.round(pos, 1)} mm")

    spher_coords = kin.to_helmet_angles(pos, helmet_center)
    r_def, alpha_deg, beta_deg = spher_coords[0], spher_coords[1], spher_coords[2]

    # Posa target (Marker allineato) e Posa Approccio
    p_obj, r_obj = kin.to_helmet_coordinates([r_def, alpha_deg, beta_deg], helmet_center)
    ee_marking_pose = kin.compute_ee_pose_for_tool_target(p_obj, r_obj, tool_pose_ee=MARKER_POSE_EE)

    r_approach = max(r_def + 50.0, float(min_approach_radius))
    p_approach_obj, _ = kin.to_helmet_coordinates([r_approach, alpha_deg, beta_deg], helmet_center)
    ee_approach_pose = kin.compute_ee_pose_for_tool_target(p_approach_obj, r_obj, tool_pose_ee=MARKER_POSE_EE)

    print(f"    Approccio a r={r_approach:.0f}mm → {np.round(ee_approach_pose[:3], 1)} mm")
    controller.move_ptp(ee_approach_pose, speed=marking_speed)

    print(f"    Avanzamento al difetto (marcatura)...")
    controller.move_line(ee_marking_pose, speed=marking_speed // 2)

    print(f"    Arretramento al punto di approccio...")
    controller.move_line(ee_approach_pose, speed=marking_speed)


def refine_and_mark_phase(controller, zed, runtime, image_zed, point_cloud, unique_defects, helmet_center):
    """ Esegue l'intero workflow continuo: viaggia al difetto, affina le misure, timbra se valido. """
    print(f"\nInizio ciclo continuo di Raffinamento e Marcatura per {len(unique_defects)} difetti.")
    confirmed_marked = 0

    for i, d in enumerate(unique_defects):
        print(f"\n--- Elaborazione Difetto {i+1}/{len(unique_defects)} ---")
        print(f"  Posizione stimata globale: {np.round(d.pos3d_global, 1)} mm")
        
        try:
            safe_transit_via_hub(controller, helmet_center)
            success = refine_defect_position(controller, zed, runtime, image_zed, point_cloud, d, helmet_center)

            if success:
                safe_transit_via_hub(controller, helmet_center) # Riordina i giunti e i tool offset prima di marcare
                mark_defect(controller, d, helmet_center)
                confirmed_marked += 1
            else:
                print(f"  [SCARTATO] Difetto ignorato per mancata conferma durante il refining.")
        except Exception as e:
            print(f"  [ERRORE] Errore sul difetto {i+1}: {e}")

    print(f"\nOperazione conclusa. Difetti marcati con successo: {confirmed_marked}/{len(unique_defects)}")


if __name__ == "__main__":
    IP_LAB = "192.168.19.22"
    controller = RobotController(ip_address=IP_LAB , default_position_j=vb.LOOK_DOWN_POSITION_J_INIZIO)
    controller.disconnect()
    controller.connect()

    # initialize the ZED camera
    zed, runtime, image_zed, point_cloud = init_zed()
    
    inspection_phase(controller, 
                     zed, runtime, image_zed, point_cloud,
                     helmet_center, 
                     inspection_angles, radius):