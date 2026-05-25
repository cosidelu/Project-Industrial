import kinematics_v2 as kin
from camera_scripts_v2 import defect, take_defects, draw_multiple_debug, init_zed, WINDOW_NAME_RGB, WINDOW_NAME_MASK
from robot_control import RobotController
import cv2
import numpy as np
from Variables import (
    HELMET_CENTER_GLOBAL,
    CAMERA_POSE_EE,
    MARKER_POSE_EE,
    LOOK_DOWN_POSITION_J_INIZIO,
)


# ============================================================
# PARAMETRI DI ISPEZIONE
# ============================================================

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

# Soglia di distanza euclidea per considerare due difetti come duplicati [mm]
DUPLICATE_THRESHOLD_MM = 10

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


# ============================================================
# FASE 4: CALCOLO COORDINATE GLOBALI
# ============================================================

def compute_global_coordinates(defect_list, H_cam_to_global):
    """
    Aggiorna pos3d_global per ciascun difetto applicando la trasformazione
    omogenea dal frame camera al frame globale.

    Input:
    - defect_list: lista di oggetti defect con pos3d_camera già valorizzato
                   (prodotti da take_defects in camera_scripts_v2)
    - H_cam_to_global: matrice omogenea 4x4, ottenuta come:
                       H_ee_to_global @ H_cam_to_ee
                       dove H_ee_to_global = kin.create_homogeneous_matrix(tcp_coord)
                       e   H_cam_to_ee    = kin.create_homogeneous_matrix(CAMERA_POSE_EE)

    Modifica in-place: scrive defect.pos3d_global per ogni difetto.
    Non restituisce nulla.

    Usa: kin.homogeneous_trasform (da kinematics_v2)
    """
    for d in defect_list:
        if d.pos3d_camera is not None:
            d.pos3d_global = kin.homogeneous_trasform(H_cam_to_global, d.pos3d_camera)


# ============================================================
# FASE 6: ESPLORAZIONE MULTISCATTO E FILTRAGGIO DUPLICATI
# ============================================================

def compute_ee_pose_for_camera_target(p_obj, r_obj):
    """
    Data la posa target per l'ORIGINE della camera (p_obj, r_obj),
    calcola la posa dell'End Effector necessaria affinché la camera
    raggiunga esattamente quella posa.

    La camera deve trovarsi in p_obj con orientamento r_obj, che viene
    calcolato da helmet_coordinates in modo che l'asse ottico sia
    perpendicolare al casco e puntante verso il suo centro.

    Equazione inversa della cinematica diretta:
        P_obj = R_ee @ P_camera_ee + t_ee
        => t_ee = P_obj - R_ee @ P_camera_ee

    Input:
    - p_obj: np.array([x, y, z]) posizione target del frame camera nel globale [mm]
    - r_obj: np.array([rx, ry, rz]) rotazioni EE in gradi
             (output di kin.helmet_coordinates, garantisce perpendicolarità al casco)

    Output:
    - lista Python [x, y, z, rx, ry, rz] da passare a controller.move_*

    Usa:
    - kin.create_rot_matrix_zyx(r_obj) da kinematics_v2
    - CAMERA_POSE_EE[:3] da Variables (offset fisico camera-EE)

    NOTA: r_obj deve essere un np.array (come restituito da helmet_coordinates),
    non una lista Python, affinché .tolist() funzioni correttamente.
    """
    R_ee = kin.create_rot_matrix_zyx(r_obj)
    t_ee = p_obj - (R_ee @ np.array(CAMERA_POSE_EE[:3]))
    return t_ee.tolist() + r_obj.tolist()


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
        p_obj, r_obj = kin.helmet_coordinates(
            [radius, alpha_deg, beta_deg], helmet_center
        )
        ee_end = compute_ee_pose_for_camera_target(p_obj, r_obj)

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

        p_mid, r_mid = kin.helmet_coordinates(
            [radius, mid_alpha, mid_beta], helmet_center
        )
        ee_mid = compute_ee_pose_for_camera_target(p_mid, r_mid)

        waypoints.append({"end": ee_end, "mid": ee_mid, "is_hub": is_hub})

    return waypoints


def filter_duplicate_defects(all_defects, threshold_mm=DUPLICATE_THRESHOLD_MM):
    """
    Filtra i difetti duplicati confrontando le coordinate globali pos3d_global.
    Mantiene il primo difetto rilevato tra quelli che cadono entro la soglia
    di distanza euclidea (proviene dallo scatto con prospettiva migliore).

    Input:
    - all_defects: lista completa di oggetti defect con pos3d_global valorizzato
    - threshold_mm: soglia di distanza euclidea in mm

    Output:
    - lista filtrata senza duplicati. I difetti con pos3d_global None vengono scartati.

    Usa: np.linalg.norm
    """
    unique_defects = []

    for candidate in all_defects:
        if candidate.pos3d_global is None:
            continue

        is_duplicate = False
        for unique in unique_defects:
            dist = np.linalg.norm(candidate.pos3d_global - unique.pos3d_global)
            if dist < threshold_mm:
                is_duplicate = True
                break

        if not is_duplicate:
            unique_defects.append(candidate)

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
    p_hub, r_hub = kin.helmet_coordinates([radius, hub_alpha, hub_beta], helmet_center)
    ee_hub_pose = compute_ee_pose_for_camera_target(p_hub, r_hub)

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


def inspection_phase(controller, zed, runtime, image_zed, point_cloud,
                     helmet_center, inspection_angles, radius):
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

        # ---- Acquisizione e rilevamento ----
        defect_list, bgr_image = take_defects(runtime, zed, image_zed, point_cloud)

        # ---- Fase 4: coordinate globali con posa EE letta in real-time ----
        # tcp_coord viene letto DOPO il completamento del move_circle (bloccante),
        # quindi rappresenta la posizione reale del robot al momento dello scatto.
        ee_pose_live = controller.robot.tcp_coord
        H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
        H_cam_to_global = H_ee_to_global @ H_cam_to_ee

        compute_global_coordinates(defect_list, H_cam_to_global)

        # ---- Rendering debug ----
        draw_multiple_debug(bgr_image, defect_list, show_global=True)
        cv2.waitKey(100)

        all_defects.extend(defect_list)
        print(f"    Difetti rilevati in questo scatto: {len(defect_list)}")

    print(f"\nIspezione globale completata. Difetti totali pre-filtraggio: {len(all_defects)}")

    unique_defects = filter_duplicate_defects(all_defects, DUPLICATE_THRESHOLD_MM)
    print(f"Difetti univoci dopo filtraggio: {len(unique_defects)}")

    return unique_defects


# ============================================================
# FASE 6b: RAFFINAMENTO POSIZIONE DIFETTI (10 foto ravvicinate)
# ============================================================

def refine_defect_position(controller, zed, runtime, image_zed, point_cloud,
                           defect_obj, helmet_center,
                           close_radius=CLOSE_INSPECTION_RADIUS,
                           n_shots=N_CLOSE_SHOTS):
    """
    Raffina la posizione di un singolo difetto già localizzato grossolanamente
    dalla fase di ispezione globale.

    PERCHÉ QUESTO STEP HA SENSO rispetto all'ispezione globale:
    L'ispezione globale scatta da 5 angolazioni fisse (0°, 45° fronte/dx/sx/dietro):
    un difetto sul lato destro viene visto di sbieco dalla camera frontale e da
    quella posteriore. Qui invece il robot si porta NELLA DIREZIONE RADIALE ESATTA
    del difetto (alpha e beta derivati dalla sua pos3d_global), così la camera
    guarda il difetto perpendicolarmente, centrandolo nell'immagine. La stima
    depth del point cloud ZED è molto più precisa quando il punto è frontale
    e centrato rispetto a quando è ai bordi o visto di sbieco.

    Procedura:
    1. Calcola alpha e beta del difetto con find_defect_angles.
    2. Porta il robot a close_radius dal centro del casco, nella direzione
       radiale esatta del difetto: la camera è perpendicolare al casco e il
       difetto è centrato nell'immagine. Il robot rimane a distanza di sicurezza
       dal casco (default 400 mm) per evitare collisioni.
    3. Legge tcp_coord reale (robot fermo dopo il move) e costruisce H_cam_to_global.
    4. Scatta N_CLOSE_SHOTS fotogrammi consecutivi senza muovere il robot.
       Per ogni scatto: cerca il difetto più vicino alla stima attuale (entro
       2x DUPLICATE_THRESHOLD_MM) e ne raccoglie il pos3d_global.
    5. Media di tutte le stime valide → aggiorna defect_obj.pos3d_global.

    Input:
    - controller: istanza di RobotController
    - zed, runtime, image_zed, point_cloud: oggetti ZED da init_zed()
    - defect_obj: oggetto defect con pos3d_global già valorizzato grossolanamente
    - helmet_center: np.array([x, y, z]) centro del casco nel globale [mm]
    - close_radius: distanza dal centro del casco [mm] (default CLOSE_INSPECTION_RADIUS).
                    Mantenuto a 400 mm per sicurezza; riducibile se il casco lo permette.
    - n_shots: numero di acquisizioni (default N_CLOSE_SHOTS = 10)

    Output:
    - True se il raffinamento ha prodotto almeno una stima valida (pos3d_global aggiornato)
    - False se nessuno scatto ha rilevato il difetto (pos3d_global invariato)

    Usa:
    - kin.find_defect_angles() da kinematics_v2
    - kin.helmet_coordinates() da kinematics_v2
    - compute_ee_pose_for_camera_target() locale
    - controller.move_ptp() da robot_control
    - take_defects() da camera_scripts_v2
    - controller.robot.tcp_coord da tm_libraries
    - kin.create_homogeneous_matrix() da kinematics_v2
    - compute_global_coordinates() locale
    - CAMERA_POSE_EE da Variables
    """
    if defect_obj.pos3d_global is None:
        print("  [SKIP raffinamento] difetto senza pos3d_global.")
        return False

    H_cam_to_ee = kin.create_homogeneous_matrix(CAMERA_POSE_EE)

    # 1. Angoli sferici della direzione del difetto rispetto al centro del casco.
    #    r_def non viene usato come raggio di posizionamento: usiamo close_radius
    #    per mantenere la distanza di sicurezza dal casco.
    _, alpha_deg, beta_deg = kin.find_defect_angles(
        defect_obj.pos3d_global, helmet_center
    )

    # 2. Posa a close_radius nella direzione radiale esatta del difetto.
    #    helmet_coordinates con [close_radius, alpha_deg, beta_deg] garantisce:
    #    - la camera sia a close_radius dal centro del casco (sicurezza)
    #    - l'asse ottico punti verso helmet_center (r_obj = [beta+90, 0, -alpha])
    #    - il difetto, che è sulla stessa retta radiale, risulti centrato
    #      nell'immagine e frontale alla camera → stima depth più precisa.
    p_close, r_close = kin.helmet_coordinates(
        [close_radius, alpha_deg, beta_deg], helmet_center
    )
    ee_close_pose = compute_ee_pose_for_camera_target(p_close, r_close)

    print(f"    Posizionamento per raffinamento: r={close_radius}mm dal centro casco, "
          f"alpha={alpha_deg:.1f}°, beta={beta_deg:.1f}° (direzione del difetto)")
    controller.move_ptp(ee_close_pose, speed=INSPECTION_SPEED)

    # 3. Lettura posa EE reale dopo il posizionamento (move_ptp è bloccante).
    #    tcp_coord riflette la posizione effettiva del robot. Usare la posa
    #    reale garantisce che H_cam_to_global sia coerente con lo scatto.
    ee_pose_live = controller.robot.tcp_coord
    H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
    H_cam_to_global = H_ee_to_global @ H_cam_to_ee

    # 4. N_CLOSE_SHOTS acquisizioni consecutive senza muovere il robot.
    #    Il robot è fermo → H_cam_to_global resta valida per tutti gli scatti.
    #    La media su più frame riduce il rumore temporale del point cloud ZED.
    refined_positions = []

    for shot_idx in range(n_shots):
        defect_list_shot, bgr_image = take_defects(
            runtime, zed, image_zed, point_cloud
        )

        compute_global_coordinates(defect_list_shot, H_cam_to_global)

        # Cerca il difetto più vicino alla stima attuale tra quelli rilevati
        best_match = None
        best_dist  = float("inf")

        for d in defect_list_shot:
            if d.pos3d_global is None:
                continue
            dist = np.linalg.norm(d.pos3d_global - defect_obj.pos3d_global)
            if dist < best_dist:
                best_dist  = dist
                best_match = d

        # Accetta lo scatto solo se il difetto trovato è entro
        # 2x DUPLICATE_THRESHOLD_MM dalla stima attuale
        association_threshold = DUPLICATE_THRESHOLD_MM * 2
        if best_match is not None and best_dist < association_threshold:
            refined_positions.append(best_match.pos3d_global)
            found_str = f"trovato (dist={best_dist:.1f}mm)"
        else:
            found_str = "non trovato"

        draw_multiple_debug(bgr_image, defect_list_shot, show_global=True)
        cv2.waitKey(50)

        print(f"      Scatto {shot_idx + 1}/{n_shots}: {found_str}")

    # 5. Media delle stime valide → aggiorna pos3d_global.
    if len(refined_positions) == 0:
        print(f"    [ATTENZIONE] Nessun scatto valido per questo difetto.")
        return False

    mean_refined = np.mean(np.array(refined_positions), axis=0)
    old_pos = defect_obj.pos3d_global.copy()
    defect_obj.pos3d_global = mean_refined

    delta = np.linalg.norm(mean_refined - old_pos)
    print(f"    Raffinamento completato ({len(refined_positions)}/{n_shots} scatti validi).")
    print(f"    Posizione: {np.round(old_pos, 1)} → {np.round(mean_refined, 1)} mm  "
          f"(delta={delta:.1f} mm)")

    return True


def refinement_phase(controller, zed, runtime, image_zed, point_cloud,
                     unique_defects, helmet_center):
    """
    Per ogni difetto univoco trovato nella fase di ispezione globale, porta il
    robot a CLOSE_INSPECTION_RADIUS dal centro del casco nella direzione radiale
    esatta del difetto, e scatta N_CLOSE_SHOTS foto per mediare la posizione 3D
    con maggiore precisione rispetto alla stima grossolana della fase 6.

    Dopo questa fase ogni difetto confermato ha pos3d_global aggiornato con la
    media di N_CLOSE_SHOTS stime ravvicinate da distanza ravvicinata, che
    sostituisce la stima grossolana della fase 6 e guida la marcatura.

    I difetti per cui il raffinamento fallisce (nessun match trovato in nessuno
    degli N_CLOSE_SHOTS scatti) vengono scartati e non inviati alla marcatura.

    Input:
    - controller: istanza di RobotController
    - zed, runtime, image_zed, point_cloud: oggetti ZED da init_zed()
    - unique_defects: lista di oggetti defect con pos3d_global grossolano
    - helmet_center: np.array([x, y, z]) centro del casco [mm]

    Output:
    - lista di oggetti defect con pos3d_global raffinato (solo quelli confermati)

    Usa: refine_defect_position() locale
    """
    print(f"\nInizio raffinamento: {len(unique_defects)} difetti da raffinare "
          f"({N_CLOSE_SHOTS} foto ravvicinate ciascuno a {CLOSE_INSPECTION_RADIUS}mm dal casco).")

    confirmed_defects = []

    for i, d in enumerate(unique_defects):
        print(f"\n  Difetto {i+1}/{len(unique_defects)} — "
              f"posizione attuale: {np.round(d.pos3d_global, 1)} mm")
        try:
            # Transito sicuro per l'hub prima di avvicinarsi a ciascun difetto.
            # Garantisce che il robot non attraversi mai la sfera di sicurezza
            # di raggio 400mm attorno al centro del casco durante il trasferimento
            # tra un difetto e il successivo.
            safe_transit_via_hub(controller, helmet_center)

            success = refine_defect_position(
                controller, zed, runtime, image_zed, point_cloud,
                d, helmet_center
            )
            if success:
                confirmed_defects.append(d)
            else:
                print(f"  [SCARTATO] Difetto non confermato dal raffinamento ravvicinato.")
        except TimeoutError as e:
            print(f"  [ERRORE] Timeout durante il raffinamento: {e}")
        except Exception as e:
            print(f"  [ERRORE] Errore imprevisto: {e}")

    print(f"\nRaffinamento completato. "
          f"Difetti confermati: {len(confirmed_defects)}/{len(unique_defects)}")

    return confirmed_defects


# ============================================================
# FASE 7: POSIZIONAMENTO E MARCATURA
# ============================================================

def compute_ee_pose_for_marker_target(p_obj, r_obj):
    """
    Calcola la posa dell'EE necessaria affinché la PUNTA DEL MARKER
    raggiunga esattamente il punto target p_obj con orientamento r_obj.

    Equazione inversa:
        t_ee = P_obj - R_ee @ P_marker_ee

    r_obj proviene da kin.helmet_coordinates → il marker risulta
    perpendicolare alla superficie del casco e allineato con il difetto.

    Input:
    - p_obj: np.array([x, y, z]) posizione del difetto nel globale [mm]
    - r_obj: np.array([rx, ry, rz]) rotazioni EE in gradi

    Output:
    - lista [x, y, z, rx, ry, rz]

    Usa:
    - kin.create_rot_matrix_zyx(r_obj) da kinematics_v2
    - MARKER_POSE_EE[:3] da Variables
    """
    R_ee = kin.create_rot_matrix_zyx(r_obj)
    t_ee = p_obj - (R_ee @ np.array(MARKER_POSE_EE[:3]))
    return t_ee.tolist() + r_obj.tolist()


def mark_defect(controller, defect_obj, helmet_center,
                min_approach_radius=MIN_APPROACH_RADIUS,
                marking_speed=MARKING_SPEED):
    """
    Esegue la sequenza di marcatura per un singolo difetto:

    1. find_defect_angles → ricava (r_def, alpha, beta) del difetto.
    2. helmet_coordinates → posa cartesiana con asse marker perpendicolare al casco.
    3. compute_ee_pose_for_marker_target → posa EE compensata per offset marker.
    4. Calcola r_approach = max(r_def + 50, MIN_APPROACH_RADIUS):
       il punto di approccio sta sulla stessa retta radiale del difetto ma a
       distanza maggiore. L'EE non entra mai nella sfera di MIN_APPROACH_RADIUS
       prima del tratto lineare finale controllato.
    5. move_ptp al punto di approccio.
    6. move_line fino al difetto (marcatura, velocità dimezzata).
    7. move_line di ritorno al punto di approccio.

    Input:
    - controller: istanza di RobotController
    - defect_obj: oggetto defect con pos3d_global raffinato dalla fase 6b
    - helmet_center: np.array([x, y, z]) centro del casco [mm]
    - min_approach_radius: distanza minima garantita dal centro [mm]
    - marking_speed: velocità di avvicinamento [mm/s]

    Usa:
    - kin.find_defect_angles() da kinematics_v2
    - kin.helmet_coordinates() da kinematics_v2
    - compute_ee_pose_for_marker_target() locale
    - controller.move_ptp(), controller.move_line() da robot_control
    """
    if defect_obj.pos3d_global is None:
        print("  [SKIP] Difetto senza coordinate globali, salto.")
        return

    pos = defect_obj.pos3d_global
    print(f"  Marcatura difetto in {np.round(pos, 1)} mm")

    # 1. Angoli sferici del difetto rispetto al centro del casco
    r_def, alpha_deg, beta_deg = kin.find_defect_angles(pos, helmet_center)

    # 2. Posa target: helmet_coordinates restituisce r_obj = [beta+90, 0, -alpha],
    #    che orienta l'asse Z dell'EE (e quindi del marker) verso helmet_center.
    #    Il marker è quindi perpendicolare al casco e allineato con il difetto.
    p_obj, r_obj = kin.helmet_coordinates([r_def, alpha_deg, beta_deg], helmet_center)
    ee_marking_pose = compute_ee_pose_for_marker_target(p_obj, r_obj)

    # 3. Punto di approccio: stessa direzione radiale, raggio maggiore.
    r_approach = max(r_def + 50.0, float(min_approach_radius))
    p_approach_obj, _ = kin.helmet_coordinates(
        [r_approach, alpha_deg, beta_deg], helmet_center
    )
    ee_approach_pose = compute_ee_pose_for_marker_target(p_approach_obj, r_obj)

    # 4. PTP al punto di approccio (fuori dalla zona di rischio)
    print(f"    Approccio a r={r_approach:.0f}mm → {np.round(ee_approach_pose[:3], 1)} mm")
    controller.move_ptp(ee_approach_pose, speed=marking_speed)

    # 5. Avanzamento lineare fino al difetto (marcatura effettiva)
    print(f"    Avanzamento al difetto (marcatura)...")
    controller.move_line(ee_marking_pose, speed=marking_speed // 2)

    # 6. Arretramento lineare al punto di approccio
    print(f"    Arretramento al punto di approccio...")
    controller.move_line(ee_approach_pose, speed=marking_speed)

    print(f"    Marcatura completata.")


def marking_phase(controller, confirmed_defects, helmet_center):
    """
    Esegue la fase di marcatura per tutti i difetti confermati dal raffinamento.
    Gestisce le eccezioni per singolo difetto per non bloccare il ciclo sugli altri.

    Input:
    - controller: istanza di RobotController
    - confirmed_defects: lista di oggetti defect con pos3d_global raffinato
    - helmet_center: np.array([x, y, z]) centro del casco [mm]

    Usa: mark_defect() locale
    """
    print(f"\nInizio marcatura: {len(confirmed_defects)} difetti confermati da marcare.")

    for i, d in enumerate(confirmed_defects):
        print(f"\n  Difetto {i+1}/{len(confirmed_defects)}")
        try:
            # Transito sicuro per l'hub prima di marcare ciascun difetto.
            # Garantisce traiettoria libera da collisioni con il casco
            # durante il trasferimento tra una marcatura e la successiva.
            safe_transit_via_hub(controller, helmet_center)

            mark_defect(controller, d, helmet_center)
        except TimeoutError as e:
            print(f"    [ERRORE] Timeout durante la marcatura: {e}")
        except Exception as e:
            print(f"    [ERRORE] Errore imprevisto: {e}")

    print("\nFase di marcatura completata.")


# ============================================================
# MAIN
# ============================================================

def main():
    # ---------- Fase 1: Inizializzazione hardware ----------
    print("=== INIZIALIZZAZIONE HARDWARE ===")

    controller = RobotController()
    controller.connect()
    print("Robot connesso.")

    zed, runtime, image_zed, point_cloud = init_zed()
    print("ZED inizializzata.")

    cv2.namedWindow(WINDOW_NAME_RGB)
    cv2.namedWindow(WINDOW_NAME_MASK)

    # Porta il robot alla posizione di rest (LOOK_DOWN_POSITION_J_INIZIO),
    # definita in robot_control.default_positioning().
    # Da qui partirà il PTP verso il primo hub dell'ispezione.
    controller.default_positioning()
    print("Robot in posizione di rest (LOOK_DOWN_POSITION_J_INIZIO).")

    try:
        # ---------- Fase 6: Ispezione globale multiscatto ----------
        print("\n=== FASE 6: ISPEZIONE GLOBALE ===")
        unique_defects = inspection_phase(
            controller=controller,
            zed=zed,
            runtime=runtime,
            image_zed=image_zed,
            point_cloud=point_cloud,
            helmet_center=HELMET_CENTER_GLOBAL,
            inspection_angles=INSPECTION_ANGLES,
            radius=INSPECTION_RADIUS,
        )

        print(f"\nDifetti trovati (stima grossolana): {len(unique_defects)}")
        for i, d in enumerate(unique_defects):
            print(f"  Difetto {i+1}: {np.round(d.pos3d_global, 1)} mm, area={d.area:.0f} px")

        if len(unique_defects) == 0:
            print("Nessun difetto rilevato. Terminazione.")
            return

        # ---------- Fase 6b: Raffinamento posizione (10 foto ravvicinate) ----------
        print("\n=== FASE 6b: RAFFINAMENTO POSIZIONE DIFETTI ===")
        confirmed_defects = refinement_phase(
            controller=controller,
            zed=zed,
            runtime=runtime,
            image_zed=image_zed,
            point_cloud=point_cloud,
            unique_defects=unique_defects,
            helmet_center=HELMET_CENTER_GLOBAL,
        )

        print(f"\nDifetti confermati dopo raffinamento: {len(confirmed_defects)}")
        for i, d in enumerate(confirmed_defects):
            print(f"  Difetto {i+1}: {np.round(d.pos3d_global, 1)} mm")

        if len(confirmed_defects) == 0:
            print("Nessun difetto confermato dal raffinamento. Terminazione.")
            return

        # Ritorno alla posizione di rest prima della conferma operatore
        controller.default_positioning()

        input("\nPremi INVIO per procedere alla marcatura dei difetti...")

        # Ritorno alla posizione di rest prima della marcatura
        controller.default_positioning()

        # ---------- Fase 7: Marcatura ----------
        print("\n=== FASE 7: MARCATURA DIFETTI ===")
        marking_phase(
            controller=controller,
            confirmed_defects=confirmed_defects,
            helmet_center=HELMET_CENTER_GLOBAL,
        )

    finally:
        # ---------- Fase 8: Terminazione ----------
        print("\n=== TERMINAZIONE ===")
        controller.default_positioning()
        controller.disconnect()
        zed.close()
        cv2.destroyAllWindows()
        print("Sistema terminato correttamente.")


if __name__ == "__main__":
    main()