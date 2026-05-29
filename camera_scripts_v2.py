import cv2
import numpy as np
import pyzed.sl as sl

# ============================================================
# PARAMETERS TO TUNE
# ============================================================

# Initial HSV range for green detection
LOWER_GREEN = np.array([35, 80, 80])
UPPER_GREEN = np.array([85, 255, 255])

# Minimum blob area to accept a green region as valid
MIN_GREEN_AREA = 150
MIN_ANOMALY_AREA = 150

# Nomi delle finestre di OpenCV per il rendering
WINDOW_NAME_RGB = "Analisi RGB Multipla"
WINDOW_NAME_MASK = "Maschera di Rilevamento Cumulativa"

# ============================================================
#                 DEFECT CLASS
# ============================================================

class defect():
    def __init__(self, centroid = np.array([0, 0]), mask = None, bgr_img = None):
        self.centroid = centroid    # np.array([cx, cy]) in pixels
        self.mask = mask            # binary mask of the defect in the image, same size as the input image
        self.bgr_img = bgr_img      # original BGR image
        self.points3d = None          # np.array of all 3D points corresponding to the defect mask, to be calculated from the point cloud and the mask
        self.pos3d_camera = None           # np.array([x, y, z]) in millimeters, to be calculated from the point cloud and the mask
        self.pos3d_global = None           # np.array([x, y, z]) in millimeters, to be calculated by transforming the camera coordinates into the global robot coordinates
        self.sph_coord = None              # np.array([r, alpha, beta]) in millimeters and degrees, to be calculated by converting the global Cartesian coordinates into spherical coordinates

    def img(self):
        """
        Returns a BGR image with the defect mask and centroid overlaid for visualization.
        """
        
        debug_img = self.bgr_img.copy()
    
        mask_bgr = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)
        debug_img = cv2.bitwise_or(debug_img, mask_bgr)

        if self.centroid is not None:
            cx, cy = int(self.centroid[0]), int(self.centroid[1])
            cv2.circle(debug_img, (cx, cy), 6, (0, 0, 255), -1)

        # we plot also the coordinates in the corner of the image
        if self.pos3d_global is not None:
            X, Y, Z = self.pos3d_global
            testo = f"G: ({X:.2f}, {Y:.2f}, {Z:.2f})mm"
            cv2.putText(debug_img, testo, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
        return debug_img
    
    def say_hi(self, Name=None):
        """Stampa valori utili per debug/tuning del singolo difetto."""

        if Name is not None:
            print(f"      {Name} rilevato:")
        else:
            print("      Difetto rilevato:")

        if self.centroid is not None:
            print(f"        centroid: {self.centroid}")

        if self.pos3d_camera is not None:
            radial_camera = np.sqrt(self.pos3d_camera[0] ** 2 + self.pos3d_camera[1] ** 2)
            print(f"        pos3d_camera: {np.round(self.pos3d_camera, 1)}")
            print(f"        Z camera: {self.pos3d_camera[2]:.1f} mm")

        if self.pos3d_global is not None:
            print(f"        pos3d_global: {np.round(self.pos3d_global, 1)}")

        if self.sph_coord is not None:
            print(f"        sph_coord [r, alpha, beta]: {np.round(self.sph_coord, 1)}")

# ============================================================
# CAMERA / ZED UTILITY FUNCTIONS
# They provide the correct syntax to initialize and read data
# from the ZED camera.
# ============================================================

def init_zed():
    """
    This function initializes the ZED camera with standard settings
    for this lab and returns all the main objects needed later.

    Returned objects:
    - zed: the camera object
    - runtime: runtime parameters used during grabbing
    - image_zed: container for the RGB image
    - point_cloud: container for the 3D point cloud
    """
    zed = sl.Camera()

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.MILLIMETER

    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED opening error: {status}")

    runtime = sl.RuntimeParameters()

    image_zed = sl.Mat()
    point_cloud = sl.Mat()

    return zed, runtime, image_zed, point_cloud


def zed_mat_to_bgr(image_zed):
    """

    This function converts the ZED image format into a standard OpenCV
    BGR image so that it can be processed with cv2 functions.

    Input:
    - image_zed: ZED image container

    Output:
    - image in OpenCV BGR format
    """
    image = image_zed.get_data()
    if image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


# ============================================================
#                 2D DEFECT DETECTION
# ============================================================

def find_all_green_masks_and_centroids(bgr_image, 
                                       LOWER_GREEN = LOWER_GREEN, 
                                       UPPER_GREEN = UPPER_GREEN, 
                                       MIN_GREEN_AREA = MIN_GREEN_AREA,
                                       attention_radius = None):
    """
    Segmenta l'immagine RGB per identificare regioni di colore verde.
    Restituisce una maschera globale e una lista di dizionari, dove ciascun dizionario 
    contiene i dati e una maschera binaria isolata per il singolo difetto.

    Input:
    - bgr_image: Immagine standard in formato OpenCV (BGR).
    - LOWER_GREEN: Limite inferiore dello spazio colore HSV.
    - UPPER_GREEN: Limite superiore dello spazio colore HSV.
    - MIN_GREEN_AREA: Soglia di area minima per filtrare il rumore.
    - attention_radius: Raggio di attenzione per filtrare i difetti in base alla distanza dal centro.
    Output:
    - difetti_estratti: Lista di dizionari con chiavi 'centroid', 'contour', 'area', 'mask_isolata', 'bgr_img'.
    """
 
    # Conversione dello spazio colore da BGR ad HSV
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    # Binarizzazione tramite sogliatura HSV
    mask_globale = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

    if attention_radius is not None:
        # Calcolo del centro dell'immagine
        height, width = mask_globale.shape
        center_x, center_y = width // 2, height // 2

        # Creazione di una maschera circolare centrata sull'immagine
        circle_mask = np.zeros_like(mask_globale)
        cv2.circle(circle_mask, (center_x, center_y), attention_radius, 255, thickness=-1)
        # Applicazione della maschera circolare alla maschera globale
        mask_globale = cv2.bitwise_and(mask_globale, circle_mask)

        cv2.circle(bgr_image, (center_x, center_y), attention_radius, (255, 0, 0), 2)

    # Filtraggio morfologico spaziale per la riduzione del rumore
    kernel = np.ones((5, 5), np.uint8)
    mask_globale = cv2.morphologyEx(mask_globale, cv2.MORPH_OPEN, kernel)
    mask_globale = cv2.morphologyEx(mask_globale, cv2.MORPH_CLOSE, kernel)

    # Estrazione dei domini topologici connessi (contorni)
    contours, _ = cv2.findContours(mask_globale, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Struttura dati per accumulare i risultati
    difetti_estratti = []

    if not contours:
        return difetti_estratti

    # Analisi iterativa di ciascun dominio identificato
    for contour in contours:
        area = cv2.contourArea(contour)

        # Criterio di validazione dimensionale
        if area >= MIN_GREEN_AREA:
            
            # Calcolo dei momenti spaziali per l'individuazione del centroide
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Allocazione di una matrice binaria vuota (tutti i pixel a 0) 
                # avente le medesime dimensioni spaziali della maschera originaria
                mask_isolata = np.zeros_like(mask_globale)
                
                # Tracciamento riempito (cv2.FILLED) del singolo dominio poligonale
                cv2.drawContours(mask_isolata, [contour], -1, 255, thickness=cv2.FILLED)
                
                difetti_estratti.append(defect(centroid=np.array([cx, cy]), mask=mask_isolata, bgr_img=bgr_image))

    return difetti_estratti

def find_all_generic_anomaly_masks_and_centroids(bgr_image, 
                                                 MIN_AREA=MIN_ANOMALY_AREA,
                                                 attention_radius=None):
    """
    Segmenta l'immagine RGB per identificare regioni di colore anomalo.
    Il metodo crea una maschera contenente i colori attesi del casco
    (nero, bianco, grigio e rosso), poi ne calcola l'inverso.
    Restituisce una lista di oggetti defect, dove ciascun oggetto contiene
    i dati e una maschera binaria isolata per il singolo difetto.

    Input:
    - bgr_image: Immagine standard in formato OpenCV (BGR).
    - MIN_AREA: Soglia di area minima per filtrare il rumore.
    - attention_radius: Raggio di attenzione per filtrare i difetti in base alla distanza dal centro.
    Output:
    - difetti_estratti: Lista di oggetti defect con centroid, mask e bgr_img.
    """

    # Conversione dello spazio colore da BGR ad HSV
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    # Binarizzazione tramite sogliatura HSV dei colori attesi del casco

    # Nero: bassa luminosità
    mask_black = cv2.inRange(
        hsv,
        np.array([0, 0, 0]),
        np.array([179, 255, 60])
    )

    # Bianco: bassa saturazione e alta luminosità
    mask_white = cv2.inRange(
        hsv,
        np.array([0, 0, 180]),
        np.array([179, 70, 255])
    )

    # Grigio: bassa saturazione e luminosità intermedia
    mask_gray = cv2.inRange(
        hsv,
        np.array([0, 0, 60]),
        np.array([179, 70, 200])
    )

    # Rosso classico: in HSV il rosso si trova sia vicino a 0° sia vicino a 180°
    mask_red_1 = cv2.inRange(
        hsv,
        np.array([0, 80, 50]),
        np.array([12, 255, 255])
    )

    mask_red_2 = cv2.inRange(
        hsv,
        np.array([170, 80, 50]),
        np.array([179, 255, 255])
    )

    # Creazione della maschera globale contenente tutti i colori attesi del casco
    mask_globale = np.zeros(hsv.shape[:2], dtype=np.uint8)
    mask_globale = cv2.bitwise_or(mask_globale, mask_black)
    mask_globale = cv2.bitwise_or(mask_globale, mask_white)
    mask_globale = cv2.bitwise_or(mask_globale, mask_gray)
    mask_globale = cv2.bitwise_or(mask_globale, mask_red_1)
    mask_globale = cv2.bitwise_or(mask_globale, mask_red_2)

    # Inversione della maschera:
    # tutto ciò che NON appartiene ai colori attesi viene considerato possibile difetto
    mask_globale = cv2.bitwise_not(mask_globale)

    if attention_radius is not None:
        # Calcolo del centro dell'immagine
        height, width = mask_globale.shape
        center_x, center_y = width // 2, height // 2

        # Creazione di una maschera circolare centrata sull'immagine
        circle_mask = np.zeros_like(mask_globale)
        cv2.circle(circle_mask, (center_x, center_y), attention_radius, 255, thickness=-1)

        # Applicazione della maschera circolare alla maschera globale
        mask_globale = cv2.bitwise_and(mask_globale, circle_mask)

        cv2.circle(bgr_image, (center_x, center_y), attention_radius, (255, 0, 0), 2)

    # Filtraggio morfologico spaziale per la riduzione del rumore
    kernel = np.ones((5, 5), np.uint8)
    mask_globale = cv2.morphologyEx(mask_globale, cv2.MORPH_OPEN, kernel)
    mask_globale = cv2.morphologyEx(mask_globale, cv2.MORPH_CLOSE, kernel)

    # Estrazione dei domini topologici connessi (contorni)
    contours, _ = cv2.findContours(mask_globale, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Struttura dati per accumulare i risultati
    difetti_estratti = []

    if not contours:
        return difetti_estratti

    # Analisi iterativa di ciascun dominio identificato
    for contour in contours:
        area = cv2.contourArea(contour)

        # Criterio di validazione dimensionale
        if area >= MIN_AREA:

            # Calcolo dei momenti spaziali per l'individuazione del centroide
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Allocazione di una matrice binaria vuota (tutti i pixel a 0)
                # avente le medesime dimensioni spaziali della maschera originaria
                mask_isolata = np.zeros_like(mask_globale)

                # Tracciamento riempito (cv2.FILLED) del singolo dominio poligonale
                cv2.drawContours(mask_isolata, [contour], -1, 255, thickness=cv2.FILLED)

                difetti_estratti.append(defect(centroid=np.array([cx, cy]), mask=mask_isolata, bgr_img=bgr_image))

    return difetti_estratti

# ============================================================
# PART 2 - 3D POSITION ESTIMATION
# ============================================================

def extract_3d_points_from_mask(mask, point_cloud):
    """

    Goal:
    Convert all green pixels into valid 3D points.

    Important note:
    Do NOT use only one pixel if you want a stable 3D estimate, but collect many 
    3D points and average them.

    1. Get all pixel coordinates from the binary mask
    2. For each pixel, read the corresponding 3D point from the ZED point cloud
    3. Reject invalid 3D points
    4. Return the list of valid 3D points

    Input:
    - mask: binary mask of green pixels
    - point_cloud: ZED point cloud container

    Output:
    - Nx3 numpy array of valid 3D points
    """

    # Extract all pixel coordinates where the mask is white
    ys, xs = np.where(mask > 0)

    points_3d = []

    # For each green pixel, query the corresponding 3D point
    for x, y in zip(xs, ys):
        err, point = point_cloud.get_value(int(x), int(y))

        if err == sl.ERROR_CODE.SUCCESS:
            X, Y, Z, RGBA = point

            # Keep only finite and valid 3D points
            if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
                if Z > 0:
                    points_3d.append([X, Y, Z])

    if len(points_3d) == 0:
        return np.empty((0, 3), dtype=np.float32)

    return np.array(points_3d, dtype=np.float32)


def compute_mean_3d_point(points_3d):
    """
    Goal:
    Compute one stable 3D estimate from many valid 3D points.

    Input:
    - points_3d: Nx3 array

    Output:
    - mean_point: [X, Y, Z] or None if no valid points exist
    """

    if len(points_3d) == 0:
        return None

    mean_point = np.mean(points_3d, axis=0)
    return mean_point


# ============================================================
# DEBUG / VISUALIZATION FUNCTIONS
# ============================================================

def draw_multiple_debug(bgr_image, defect_list, show_global=False):
    """
    Sovrappone le informazioni visive di molteplici difetti sull'immagine sorgente.
    """
    debug_img = bgr_image.copy()
    
    # Allocazione di una matrice nulla per accumulare le maschere binarie
    mask_globale = np.zeros(bgr_image.shape[:2], dtype=np.uint8)

    for defect in defect_list:
        # 1. Unione topologica delle maschere isolate
        if defect.mask is not None:
            mask_globale = cv2.bitwise_or(mask_globale, defect.mask)

        # 2. Annotazione dei centroidi e delle stime tridimensionali
        if defect.centroid is not None:
            cx, cy = int(defect.centroid[0]), int(defect.centroid[1])
            
            # Disegno del centroide spaziale
            cv2.circle(debug_img, (cx, cy), 6, (0, 0, 255), -1)
            
            testo = ""
            if show_global and defect.pos3d_global is not None:
                X, Y, Z = defect.pos3d_global
                testo = f"G: ({X:.2f}, {Y:.2f}, {Z:.2f})mm"
            elif defect.pos3d_camera is not None:
                X, Y, Z = defect.pos3d_camera
                testo = f"C: ({X:.2f}, {Y:.2f}, {Z:.2f})mm"
                
            if testo:
                # Apposizione del vettore 3D in prossimità del centroide
                cv2.putText(debug_img, testo, (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Conversione della maschera globale ad un formato compatibile con la visualizzazione standard
    mask_bgr = cv2.cvtColor(mask_globale, cv2.COLOR_GRAY2BGR)
    return debug_img, mask_bgr

# ============================================================
# defect wrapper main program
# ============================================================

def take_defects_local(runtime, zed, image_zed, point_cloud, attention_radius=None, generic_detection=False):
    '''
    Funzione raccoglie le informazioni per ogni difetto fino alla sua posizione nel sistema di riferimento della camera
    Da come output la lista di tutti i difetti nell'immagine e l'immagine stessa che può essere usata per debug
    '''
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        # ------------------------------------------------
        # ZED DATA ACQUISITION
        # ------------------------------------------------

        # Acquire an image from the cameras
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)  
        # Estimate depth and generate a pointcloud            
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        # Convert the ZED pointcloud format to the Open-CV format 
        bgr_image = zed_mat_to_bgr(image_zed)

        # ------------------------------------------------
        # 2D DEFECT DETECTION
        # ------------------------------------------------
        if generic_detection:
            defect_list = find_all_generic_anomaly_masks_and_centroids(
                bgr_image,
                attention_radius=attention_radius
            )
        else:
            defect_list = find_all_green_masks_and_centroids(
                bgr_image,
                attention_radius=attention_radius
            )

        # ------------------------------------------------
        # 3D POSITION ESTIMATION
        # ------------------------------------------------
       
        for defect in defect_list[:]:  # iterazione su una copia della lista per poter rimuovere elementi
            points_3d = extract_3d_points_from_mask(defect.mask, point_cloud)
            mean_point_3d = compute_mean_3d_point(points_3d)

            if mean_point_3d is not None:
                defect.points3d = points_3d
                defect.pos3d_camera = mean_point_3d
            else:
                # rimuovi il difetto dalla lista se non è stato possibile stimare una posizione 3D valida
                defect_list.remove(defect)
        
        return defect_list, bgr_image
    else:
        raise RuntimeError("Errore durante l'acquisizione dei dati dalla ZED.")


# ============================================================
# MAIN PROGRAM
# 1. acquire image
# 2. acquire point cloud
# 3. detect green area in 2D
# 4. estimate 3D position
# ============================================================

def main():
    # Inizializzazione dei sistemi sensoriali
    zed, runtime, image_zed, point_cloud = init_zed()
    
    print("Sistema inizializzato. Premere [ESC] nella finestra grafica per terminare l'esecuzione.")

    try:
        while True:
            # Esecuzione dell'intero flusso di calcolo
            defect_list, bgr_image = take_defects_local(
                runtime,
                zed,
                image_zed,
                point_cloud,
                attention_radius=200,
                generic_detection=True
            )
            
            # Rendering del layer informativo
            debug_img, mask_bgr = draw_multiple_debug(bgr_image, defect_list)
            cv2.imshow(WINDOW_NAME_MASK, mask_bgr)
            cv2.imshow(WINDOW_NAME_RGB, debug_img)
                
            # Funzione bloccante temporizzata (10 millisecondi) per il rendering hardware di OpenCV
            key = cv2.waitKey(10) & 0xFF
            
            # Condizione di uscita: codice ASCII 27 (tasto ESC)
            if key == 27:
                break
                
    finally:
        # Deallocazione sicura della memoria hardware
        zed.close()
        cv2.destroyAllWindows()
        print("Terminazione corretta dei processi periferici.")

if __name__ == "__main__":
    main()