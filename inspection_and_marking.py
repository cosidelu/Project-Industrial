import cv2
import numpy as np
import time

import kinematics_v2 as kin
import Variables as vb
from Variables import HELMET_CENTER_GLOBAL, CAMERA_POSE_EE, MARKER_POSE_EE

from robot_control import RobotController
from spherical_movement import move_circle_spherical

from defects_id_wrapper import (
    take_defects_global,
    duplicate_filter
)

from camera_scripts_v2 import (
    draw_multiple_debug,
    init_zed,
    WINDOW_NAME_RGB,
    WINDOW_NAME_MASK
)


# =====================================================
# PARAMETRI GENERALI
# =====================================================

# Se False: usa la detection classica del difetto verde.
# Se True: usa la nuova detection generica tramite maschera inversa
# dei colori attesi del casco.
GENERIC_DETECTION = False

# IP del robot reale in laboratorio.
# Per simulazione/test locale puoi mettere "127.0.0.1".
IP_ROBOT = "192.168.1.3"

# Raggio della sfera di ispezione attorno al casco [mm].
INSPECTION_RADIUS = 400

# Punto hub sicuro sopra il casco.
# Usato come transito tra ispezione, raffinamento e marcatura.
HUB_ANGLES = (0, 90)

# Waypoint di ispezione.
# Ogni tupla è:
# (alpha_deg, beta_deg, is_hub)
#
# is_hub=True  -> punto di transito, nessuna foto
# is_hub=False -> punto di scatto
INSPECTION_ANGLES = [
    (0,    90,  True),     # hub iniziale
    (0,    90,  False),    # scatto 1: dall'alto
    (0,    45,  False),    # scatto 2
    (87,   60,  False),    # scatto 3
    (-87,  60,  False),    # scatto 4
    (0,    90,  True),     # hub finale
]

# Velocità movimento durante ispezione [mm/s].
INSPECTION_SPEED = 400

# Velocità movimento durante marcatura [mm/s].
MARKING_SPEED = 50

# Raggio usato per il raffinamento.
# Il robot va nella direzione del difetto, ma resta a questa distanza
# dal centro casco per sicurezza.
CLOSE_INSPECTION_RADIUS = 400

# Numero di foto ravvicinate per raffinare ogni difetto.
N_CLOSE_SHOTS = 10

# Soglia per rimuovere duplicati tra più scatti [mm].
DUPLICATE_THRESHOLD = 25.0

# Soglia per associare un difetto rilevato nel raffinamento
# al difetto originale [mm].
ASSOCIATION_THRESHOLD = 30.0

# Distanza minima dal centro casco per il punto di approccio del marker [mm].
MIN_APPROACH_RADIUS = 250


# =====================================================
# PARAMETRI FILTRI / TUNING
# =====================================================

# -------------------------------
# Tuning ispezione globale
# -------------------------------

# In ispezione globale il difetto può non essere perfettamente al centro.
# None = considera tutta l'immagine.
GLOBAL_ATTENTION_RADIUS = None

# Filtro cilindrico in frame camera.
# Tiene solo punti entro questo raggio laterale [mm].
GLOBAL_CYLINDER_RADIUS = 220.0

# Range di profondità lungo Z camera [mm].
# Tiene solo punti non troppo vicini e non troppo lontani.
GLOBAL_HEIGHT_RANGE = (150, 700)


# -------------------------------
# Tuning raffinamento
# -------------------------------

# Nel raffinamento il difetto dovrebbe essere centrato nell'immagine.
# Quindi restringiamo la ricerca al centro.
REFINE_ATTENTION_RADIUS = 250

# Cilindro più stretto per il raffinamento [mm].
REFINE_CYLINDER_RADIUS = 150.0

# Range di profondità più stretto per il raffinamento [mm].
REFINE_HEIGHT_RANGE = (50, 300)


# -------------------------------
# Filtro globale
# -------------------------------

# Filtro rispetto alla posizione globale del casco.
# Per ora meglio False: accendilo solo quando HELMET_CENTER_GLOBAL è affidabile.
USE_GLOBAL_POSITION_FILTER = False


# =====================================================
# FASE 1 - ISPEZIONE GLOBALE
# =====================================================
  
def point_and_shoot(controller,
                                 zed,
                                 runtime,
                                 image_zed,
                                 point_cloud,
                                 test_sph,
                                 helmet_center=HELMET_CENTER_GLOBAL):
    move_circle_spherical(
        controller=controller,
        end_sph_coord=test_sph,
        radius=test_sph[0],
        tool_pose_ee=vb.CAMERA_POSE_EE,
        helmet_center=helmet_center,
        speed=INSPECTION_SPEED
    )

    time.sleep(1)

    H_cam_to_ee = kin.create_homogeneous_matrix(vb.CAMERA_POSE_EE)
    ee_pose_live = controller.robot.tcp_coord
    H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
    H_cam_to_global = H_ee_to_global @ H_cam_to_ee

    defect_list, bgr_image = take_defects_global(
        runtime,
        zed,
        image_zed,
        point_cloud,
        H_cam_to_global=H_cam_to_global,
        attention_radius=GLOBAL_ATTENTION_RADIUS,
        cylindrical_filter=True,
        radius=GLOBAL_CYLINDER_RADIUS,
        height_range=GLOBAL_HEIGHT_RANGE,
        position_filtering=USE_GLOBAL_POSITION_FILTER,
        generic_detection=GENERIC_DETECTION
    )

    return defect_list, bgr_image


# =====================================================
# FASE 2 - RAFFINAMENTO POSIZIONE DIFETTO
# =====================================================

def refine_defect_position(controller,
                           zed,
                           runtime,
                           image_zed,
                           point_cloud,
                           defect_obj,
                           helmet_center = HELMET_CENTER_GLOBAL,
                           close_radius=CLOSE_INSPECTION_RADIUS,
                           n_shots=N_CLOSE_SHOTS,
                           generic_detection=False):
    """
    Raffina la posizione 3D di un singolo difetto.

    Procedura:
    1. Prende defect_obj.pos3d_global.
    2. Calcola alpha e beta del difetto rispetto al centro casco.
    3. Muove la camera a close_radius nella stessa direzione.
    4. Scatta n_shots foto.
    5. Per ogni foto trova il difetto più vicino alla posizione precedente.
    6. Media le posizioni valide.
    7. Aggiorna defect_obj.pos3d_global.
    """

    if defect_obj.pos3d_global is None:
        print("  [SKIP] Difetto senza pos3d_global.")
        return False

    H_cam_to_ee = kin.create_homogeneous_matrix(CAMERA_POSE_EE)

    # Coordinate sferiche della posizione stimata del difetto.
    r_def, alpha_deg, beta_deg = kin.to_helmet_angles(
        defect_obj.pos3d_global,
        helmet_center
    )

    print(
        f"  Raffinamento difetto:"
        f" r_stimato={r_def:.1f}mm,"
        f" alpha={alpha_deg:.1f}°,"
        f" beta={beta_deg:.1f}°"
    )

    # Punto di osservazione ravvicinato:
    # stesso alpha/beta del difetto, ma raggio fissato a close_radius.
    close_sph = np.array([close_radius, alpha_deg, beta_deg])

    print(f"  Movimento camera a r={close_radius}mm nella direzione del difetto.")

    move_circle_spherical(
        controller=controller,
        end_sph_coord=close_sph,
        radius=close_radius,
        tool_pose_ee=CAMERA_POSE_EE,
        helmet_center=helmet_center,
        speed=INSPECTION_SPEED
    )

    time.sleep(1)

    # Posa reale dopo il movimento.
    ee_pose_live = controller.robot.tcp_coord
    H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
    H_cam_to_global = H_ee_to_global @ H_cam_to_ee

    refined_positions = []

    for shot_idx in range(n_shots):

        # Acquisizione + detection con parametri più stretti.
        defect_list_shot, bgr_image = take_defects_global(
            runtime,
            zed,
            image_zed,
            point_cloud,
            H_cam_to_global=H_cam_to_global,

            # Nel raffinamento il difetto dovrebbe essere centrato.
            attention_radius=REFINE_ATTENTION_RADIUS,

            # Filtro cilindrico più stretto.
            cylindrical_filter=True,
            radius=REFINE_CYLINDER_RADIUS,
            height_range=REFINE_HEIGHT_RANGE,

            # Per ora stesso filtro globale della fase principale.
            position_filtering=USE_GLOBAL_POSITION_FILTER,

            generic_detection=generic_detection
        )

        best_match = None
        best_dist = float("inf")

        # Cerco il difetto rilevato più vicino alla posizione stimata.
        for d in defect_list_shot:
            if d.pos3d_global is None:
                continue

            dist = np.linalg.norm(d.pos3d_global - defect_obj.pos3d_global)

            if dist < best_dist:
                best_dist = dist
                best_match = d

        # Accetto il match solo se è abbastanza vicino.
        if best_match is not None and best_dist < ASSOCIATION_THRESHOLD:
            refined_positions.append(best_match.pos3d_global)
            found_str = f"trovato, distanza={best_dist:.1f}mm"
        else:
            found_str = "non trovato"

        debug_img, mask_bgr = draw_multiple_debug(
            bgr_image,
            defect_list_shot,
            show_global=True
        )

        cv2.imshow(WINDOW_NAME_MASK, mask_bgr)
        cv2.imshow(WINDOW_NAME_RGB, debug_img)
        cv2.waitKey(50)

        print(f"    Scatto {shot_idx + 1}/{n_shots}: {found_str}")

        # Stampa valori utili per tuning.
        for idx, d in enumerate(defect_list_shot):
            if d.pos3d_camera is not None:
                radial_camera = np.sqrt(d.pos3d_camera[0] ** 2 + d.pos3d_camera[1] ** 2)
                print(f"      Difetto rilevato {idx + 1}:")
                print(f"        centroid: {d.centroid}")
                print(f"        pos3d_camera: {np.round(d.pos3d_camera, 1)}")
                print(f"        Z camera: {d.pos3d_camera[2]:.1f} mm")
                print(f"        radial camera: {radial_camera:.1f} mm")
                print(f"        pos3d_global: {np.round(d.pos3d_global, 1)}")

    if len(refined_positions) == 0:
        print("  [ATTENZIONE] Nessuno scatto valido durante il raffinamento.")
        return False

    old_pos = defect_obj.pos3d_global.copy()
    mean_refined = np.mean(np.array(refined_positions), axis=0)

    defect_obj.pos3d_global = mean_refined

    delta = np.linalg.norm(mean_refined - old_pos)

    print("  Raffinamento completato.")
    print(f"  Posizione vecchia: {np.round(old_pos, 1)} mm")
    print(f"  Posizione nuova:   {np.round(mean_refined, 1)} mm")
    print(f"  Delta: {delta:.1f} mm")

    return True


# =====================================================
# FASE 3 - MARCATURA SINGOLO DIFETTO
# =====================================================

def mark_defect(controller,
                defect_obj,
                helmet_center,
                min_approach_radius=MIN_APPROACH_RADIUS,
                marking_speed=MARKING_SPEED):
    """
    Marca un singolo difetto con il pennarello.

    Procedura:
    1. Prende defect_obj.pos3d_global.
    2. Calcola coordinate sferiche del difetto rispetto al centro casco.
    3. Calcola la posa target del marker sul difetto.
    4. Calcola un punto di approccio più lontano, sulla stessa direzione radiale.
    5. Salva la posa corrente come punto di ritorno post-marking.
    6. Va al punto di approccio con move_ptp.
    7. Avanza linearmente al difetto con move_line (tocco).
    8. Arretra linearmente al punto di approccio con move_line.
    9. Torna con move_ptp alla posa salvata al punto 5.
    """

    if defect_obj.pos3d_global is None:
        print("  [SKIP] Difetto senza coordinate globali.")
        return

    pos = defect_obj.pos3d_global

    print(f"  Avvio marcatura difetto in {np.round(pos, 1)} mm")

    # Coordinate sferiche del difetto.
    r_def, alpha_deg, beta_deg = kin.to_helmet_angles(
        pos,
        helmet_center
    )

    # Posa target del marker sul difetto.
    p_obj, r_obj = kin.to_helmet_coordinates(
        [r_def, alpha_deg, beta_deg],
        helmet_center
    )

    ee_marking_pose = kin.compute_ee_pose_for_tool_target(
        p_obj,
        r_obj,
        tool_pose_ee=MARKER_POSE_EE
    )

    # Punto di approccio:
    # stessa direzione del difetto, ma più lontano dal centro casco.
    r_approach = max(r_def + 50.0, float(min_approach_radius))

    p_approach_obj, _ = kin.to_helmet_coordinates(
        [r_approach, alpha_deg, beta_deg],
        helmet_center
    )

    ee_approach_pose = kin.compute_ee_pose_for_tool_target(
        p_approach_obj,
        r_obj,
        tool_pose_ee=MARKER_POSE_EE
    )

    print(f"    Punto di approccio a r={r_approach:.1f}mm.")
    print(f"    Posa approccio EE: {np.round(ee_approach_pose, 1)}")

    # Salva la posa attuale come punto di ritorno post-marking,
    # PRIMA di qualsiasi movimento verso il difetto.
    #pre_marking_pose = list(controller.robot.tcp_coord)

    # Movimento al punto di approccio.
    if not move_circle_spherical(
        controller=controller,
        end_sph_coord=[r_approach, alpha_deg, beta_deg],
        radius=250,
        tool_pose_ee=MARKER_POSE_EE,
        helmet_center=helmet_center,
        speed=marking_speed
    ):
        print("  [SKIP] Impossibile raggiungere il punto di approccio in sicurezza.")
        return False
    
    pre_marking_pose = controller.robot.tcp_coord

    # Movimento lineare lento fino al difetto (tocco).
    print("    Avanzamento lineare al difetto.")
    controller.move_line(
        ee_marking_pose,
        speed=marking_speed // 2
    )

    # Arretramento lineare al punto di approccio.
    print("    Arretramento lineare.")
    controller.move_line(
        pre_marking_pose,
        speed=marking_speed
    )

    # PTP di ritorno alla posa precedente al marking.
    #print("    Ritorno PTP alla posa pre-marking.")
    #controller.move_ptp(
    #    pre_marking_pose,
    #   speed=marking_speed
    #)

    print("    Marcatura completata.")
