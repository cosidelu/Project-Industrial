import kinematics_v2 as kin
from camera_scripts_v2 import take_defects_local, draw_multiple_debug, init_zed, WINDOW_NAME_RGB, WINDOW_NAME_MASK
import cv2
from Variables import HELMET_CENTER_GLOBAL, CAMERA_POSE_EE

import numpy as np

# =====================================================
# MAIN WRAPPER FUNCTION
# =====================================================

def take_defects_global(runtime, zed, image_zed, point_cloud, H_cam_to_global, 
                        attention_radius=None, 
                        cylindrical_filter=False, radius=150.0, height_range=(0, 300),
                        position_filtering=False,
                        generic_detection=False):
    
    '''
    Funzione wrapper che esegue l'intero flusso di calcolo per ottenere la lista dei difetti con coordinate globali.
    1. Acquisisce i difetti locali e l'immagine BGR dalla ZED.
    2. Calcola le coordinate globali per ciascun difetto utilizzando la matrice omogenea pre-calcolata.
    3. Applica eventuali filtri sui difetti (cylindrical_filter e position_filtering) se richiesti.

    input:
    - runtime, zed, image_zed, point_cloud: parametri necessari per l'acquisizione dei difetti locali dalla ZED.
    - H_cam_to_global: matrice omogenea pre-calcolata per la trasformazione delle coordinate dalla telecamera al frame globale.
    - attention_radius [pixel]: parametro opzionale per limitare l'attenzione ad un cerchio di pixel attorno al centro dell'immagine (passato a take_defects_local).
    - cylindrical_filter: booleano che indica se applicare un filtro cilindrico basato sulle coordinate della telecamera.
    - radius [mm]: raggio del cilindro per il filtro cilindrico.
    - height_range [mm]: intervallo di altezza per il filtro cilindrico.
    - position_filtering: booleano che indica se applicare un filtro sferico basato sulla distanza globale dei difetti da un centro di riferimento (es. centro del casco).

    output:
    - defect_list: lista di oggetti difetto con coordinate globali aggiornate e filtrati secondo i criteri specificati.
    - bgr_image: immagine BGR acquisita dalla ZED, utile per il debug e il rendering dei layer informativi.
    '''

    defect_list, bgr_image = take_defects_local(
        runtime,
        zed,
        image_zed,
        point_cloud,
        attention_radius=attention_radius,
        generic_detection=generic_detection
    )

    compute_global_coordinates(defect_list, H_cam_to_global)

    compute_spherical_coordinates(defect_list, helmet_center=HELMET_CENTER_GLOBAL)

    if cylindrical_filter:
        defect_list = cam_cylinder_filter(defect_list, radius=radius, height_range=height_range)

    if position_filtering:
        defect_list = glob_position_filter(defect_list)

    return defect_list, bgr_image


def duplicate_filter(defect_list, distance_threshold=25.0):
    """
    Filtra i difetti duplicati basandosi sulla distanza euclidea tra le loro posizioni globali.
    Restituisce una nuova lista di difetti unici.
    """
    unique_defects = defect_list.copy()
    i = 0
    j = 0

    while i < len(unique_defects):
        d_original = unique_defects[i]
        j = i + 1  # Inizia a confrontare dal successivo
        while j < len(unique_defects):
            d_confronto = unique_defects[j]
            dist = np.linalg.norm(d_original.pos3d_global - d_confronto.pos3d_global) # euclidean distance in mm
            if dist < distance_threshold:
                # Se i difetti sono troppo vicini, considerali duplicati e rimuovi il secondo
                unique_defects.pop(j)
            else:
                j += 1 # Solo se non rimuoviamo, incrementiamo j per confrontare il prossimo difetto
        i += 1

    return unique_defects

# =====================================================
# AUXILIARY FUNCTIONS
# =====================================================

def compute_global_coordinates(defect_list, H_cam_to_global):
    """
    Aggiorna l'attributo pos3d_global per ciascun difetto nella lista fornita.
    Utilizza la matrice omogenea pre-calcolata per trasformare i punti dal frame fotocamera al frame globale.
    """
    for d in defect_list:
        if d.pos3d_camera is not None:
            # 4. Applicazione della trasformazione spaziale (valori già in mm dalla ZED)
            d.pos3d_global = kin.homogeneous_trasform(H_cam_to_global, d.pos3d_camera)

def compute_spherical_coordinates(defect_list, helmet_center=HELMET_CENTER_GLOBAL):
    """
    Calcola le coordinate sferiche (r, alpha, beta) per ciascun difetto rispetto al centro del casco.
    """
    for d in defect_list:
        if d.pos3d_camera is not None:
            d.sph_coord = kin.to_helmet_angles(d.pos3d_global, helmet_center)


def cam_cylinder_filter(defect_list, radius=150.0, height_range=(0, 300)):
    """
    Filtra i difetti basandosi su una regione cilindrica attorno alla telecamera.
    Restituisce una nuova lista di difetti che si trovano all'interno del cilindro.
    """
    filtered_defects = []
    for d in defect_list:
        if d.pos3d_camera is not None:
            # Calcola la distanza dalla telecamera (solo nella direzione orizzontale)
            dist_horizontal = np.sqrt(d.pos3d_camera[0]**2 + d.pos3d_camera[1]**2)
            # Verifica se il difetto è all'interno del cilindro
            if dist_horizontal <= radius and height_range[0] <= d.pos3d_camera[2] <= height_range[1]:
                filtered_defects.append(d)
    return filtered_defects


def glob_position_filter(defect_list, range = [200, 300], center = HELMET_CENTER_GLOBAL):
    """
    Filtra i difetti basandosi sulla distanza euclidea tra la loro posizione globale e un centro di riferimento (es. centro del casco).
    Restituisce una nuova lista di difetti che si trovano entro un certo intervallo di distanza dal centro.
    """
    filtered_defects = []
    for d in defect_list:
        if d.pos3d_global is not None:
            dist = np.linalg.norm(d.pos3d_global - center) # euclidean distance in mm
            if range[0] <= dist <= range[1]:
                filtered_defects.append(d)
    return filtered_defects


# =====================================================
# MAIN EXECUTION
# =====================================================


if __name__ == "__main__":
    # initialize the ZED camera
    zed, runtime, image_zed, point_cloud = init_zed()

    # initialize the total defects list
    all_defects = []

    print("Inizio acquisizione dati. Premere SPAZIO per continuare, premere ESC per terminare.")

    # Creiamo preventivamente le finestre in modo che cv2.waitKey() possa registrare gli input da subito
    cv2.namedWindow(WINDOW_NAME_RGB)
    cv2.namedWindow(WINDOW_NAME_MASK)

    # Costruzione della matrice H_cam_to_global fuori dal ciclo definendo le due pose
    # tramite la funzione kin.create_homogeneous_matrix.
    # 1. Posa dell'End Effector (Robot): fittizia, traslata a 40 cm (400 mm) di altezza.
    #ee_pose_global = [0.0, 0.0, 400.0, 0.0, 0.0, 0.0]
    # 2. Offset telecamera: solo orientamento (Z_cam su X_glob, Y_cam giù, X_cam a dx).
    #camera_offset_ee = [0.0, 0.0, 0.0, -90.0, 0.0, -90.0]

    # in questo modo le coordinate globali sono calcolate rispetto al trackpad del computer per il testing

    ee_pose_global = [300, 500, 300, 0, 90, 0]
    camera_pose_ee = CAMERA_POSE_EE

    H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_global)
    H_cam_to_ee = kin.create_homogeneous_matrix(camera_pose_ee)
    H_cam_to_global = H_ee_to_global @ H_cam_to_ee

    try:
        while True:
            # Attesa input utente PRIMA di effettuare la foto e l'elaborazione
            while True:
                key = cv2.waitKey(0) & 0xFF
                if key == 27 or key == 32:  # ESC (27) o SPAZIO (32)
                    break
                    
            if key == 27:  # Se è ESC, interrompi l'acquisizione ed esci
                break
                
            # Esecuzione dell'intero flusso di calcolo
            defect_list, bgr_image = take_defects_global(runtime, zed, image_zed, point_cloud, H_cam_to_global,
                                                         attention_radius=None,
                                                         cylindrical_filter=False,
                                                         position_filtering=False)

            # Rendering del layer informativo con coordinate globali
            debug_img, mask_bgr = draw_multiple_debug(bgr_image, defect_list, show_global=True)
            #cv2.imshow(WINDOW_NAME_MASK, mask_bgr)
            cv2.imshow(WINDOW_NAME_RGB, debug_img)

            all_defects.extend(defect_list)
                
    finally:
        # Deallocazione sicura della memoria hardware
        zed.close()
        cv2.destroyAllWindows()
        print("Terminazione corretta dei processi periferici.")

        print(f"Totale difetti rilevati: {len(all_defects)}")

        # Filtra i difetti duplicati
        unique_defects = duplicate_filter(all_defects)
        print(f"Totale difetti unici: {len(unique_defects)}")

        # Plotta finale dei difetti unici con coordinate globali
        index = 1
        for d in unique_defects:
            cv2.imshow(f"Defect {index}", d.img())
            index += 1

        print("Premere un tasto qualsiasi per chiudere le finestre dei difetti.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
