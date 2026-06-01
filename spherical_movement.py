import kinematics_v2 as kin
from robot_control import RobotController

from Variables import HELMET_CENTER_GLOBAL

import cv2
import numpy as np
import time


Z_LIMIT = 150  # Limite assoluto di altezza sotto il quale non è sicuro muoversi (es. base del casco o tavolo)

def angles_unsafe(alpha, beta):
    """
    Verifica se gli angoli sferici (alpha, beta) si trovano al di fuori dei limiti 
    di sicurezza o all'interno di zone di collisione (dietro, davanti, o laterale eccessivo).
    
    Convenzione corrente:
    - alpha: intervallo [-90, 90] gradi -> 0 = centro, +90 = lato destro, -90 = lato sinistro.
    - beta: intervallo [0, 180] gradi -> (0 = retro, 90 = sommità, 180 = fronte).
    
    :return: True se la configurazione è pericolosa/non ammessa, False altrimenti.
    """
    # 1. Verifica dei limiti geometrici del dominio della convenzione
    if not (0.0 <= beta <= 180.0):
        return True

    # 2. Controllo limite laterale: alpha deve essere compreso entro +/- 120 gradi -> sotituito da z limit
    #if alpha > 105.0 or alpha < -95.0:
    #    return True

    # 3. Controllo per escludere la visiera e la parte davanti
    beta_soglia_visiera = 45 + (120-45) * (alpha / 180.0)**2
    if beta < beta_soglia_visiera:
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

    PERCHé FUNZIONI IL MOVIMENTO CILINDRICO è IMPORTANTE CHE LA IL end_sph_coord[0] SIA IL RAGGIO DEL DIFETTO-PUNTO CHE VOGLIO GUARDARE
    """
    Z_DIST_CYLTOCENTER = 50  # distanza verticale tra il centro del casco e la giunzione cilindro-sfera, da tunare in base alla forma del casco
    Z_CYLINDER = helmet_center[2] + Z_DIST_CYLTOCENTER  # in questo modo evitiamo a prescindere i gimball lock
    SPHERE_RADIUS = radius 

    R_CYLINDER = np.sqrt(SPHERE_RADIUS**2 - Z_DIST_CYLTOCENTER**2)  # Raggio del cilindro alla giunzione con la sfera

    # NB: questi parametri vanno tunati in modo che il cilindro non collida con il casco

    def actually_move(start_a, start_b, end_a, end_b, force_line=False):
        """
        Esegue fisicamente il movimento. Utilizza move_circle calcolando il midpoint,
        oppure ottimizza con PTP per spostamenti molto piccoli o forzati.
        """
        d_alpha = end_a - start_a
        d_beta = end_b - start_b

        p_end, r_end = kin.to_helmet_coordinates([radius, end_a, end_b], helmet_center)
        ee_end = kin.compute_ee_pose_for_tool_target(p_end, r_end, tool_pose_ee=tool_pose_ee)

        if force_line or (d_alpha**2 + d_beta**2 < 5**2):
            if force_line:
                print(f"  [FORCE LINE] Esecuzione forzata verso (alpha={end_a:.1f}°, beta={end_b:.1f}°).")
            else:
                print(f"  [SHORT PATH] Distanza angolare < 5°. Esecuzione ottimizzata PTP verso (alpha={end_a:.1f}°, beta={end_b:.1f}°).")
            controller.move_line(ee_end, speed=speed)
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
    start_alpha, start_beta = start_angles[1], start_angles[2]
    end_alpha, end_beta = end_sph_coord[1], end_sph_coord[2]
    p_end_def, _ = kin.to_helmet_coordinates(end_sph_coord, helmet_center)

    # --- 0. Controllo Sicurezza Destinazione --- 
    if angles_unsafe(end_alpha, end_beta):
        print(f"  [SKIP] Destinazione (alpha={end_alpha:.1f}°, beta={end_beta:.1f}°) fuori limiti sicurezza.")
        return False
    
    # check se la z del difetto è troppo in basso
    if p_end_def[2] < Z_LIMIT:
        print(f"  [ERROR] Destinazione finale a z={p_end_def[2]:.1f} mm, che è sotto il limite assoluto di {Z_LIMIT} mm. Movimento rifiutato.")
        return False

    # --- Verfico se sto partendo dal cilindro e se si mi sposto sulla sfera e aggiorno le coordinate globali del tool ---

    if tool_position_global[2] < Z_CYLINDER:
        print(f"  [CYL] Partenza da cilindro, mi sposto prima sulla sfera a z={Z_CYLINDER}mm")
        tool_position_global[2] = Z_CYLINDER
        cyl_angles = kin.to_helmet_angles(tool_position_global, helmet_center)

        # a questo punto mi sposto sull'intersezione tra cilindro e sfera
        actually_move(0,0, cyl_angles[1], cyl_angles[2], force_line=True)

        time.sleep(1)  # breve pausa per stabilizzare il movimento
        # Ricalcolo la posa finale del tool dopo la deviazione cilindrica nel dubbio
        ee_pose = controller.robot.tcp_coord
        H_ee_to_glob = kin.create_homogeneous_matrix(ee_pose)
        tool_position_global = kin.homogeneous_trasform(H_ee_to_glob, tool_position_ee)

    
    start_angles = kin.to_helmet_angles(tool_position_global, helmet_center)
    start_radius = start_angles[0]

    if abs(start_radius - radius) > 20:
        print(f"  [WARNING] Raggio attuale {start_radius:.1f} mm differisce significativamente dal raggio target {radius:.1f} mm.")
        input("  Premere Invio per continuare comunque, o nope per annullare...")
        if input().lower() == "nope":
            print("  Movimento annullato dall'utente.")
            return False
    
    start_alpha, start_beta = start_angles[1], start_angles[2]
    end_alpha, end_beta = end_sph_coord[1], end_sph_coord[2]

    # --- 2. Controllo Sicurezza Destinazione --- 
    if angles_unsafe(end_alpha, end_beta):
        print(f"  [SKIP] Destinazione (alpha={end_alpha:.1f}°, beta={end_beta:.1f}°) fuori limiti sicurezza.")
        return False
    
    # --- Controllo se la fine è nel ciilindro
    ends_on_cylinder = False
    
    # check se la z del difetto
    if p_end_def[2] < Z_CYLINDER:
        print(f"  [CYL] Destinazione finale prevista a z={p_end_def[2]:.1f} mm, che è sotto la soglia cilindrica di {Z_CYLINDER} mm.")
        ends_on_cylinder = True
        p_end_cyl = np.zeros(3)
        
        #calcolo il punto finale di ispezione sul cilindro alla stessa altezza del difetto
        gamma = np.arctan2(p_end_def[0] - helmet_center[0], p_end_def[1] - helmet_center[1])  # angolo polare del punto di destinazione

        p_end_cyl[0] = helmet_center[0] + R_CYLINDER * np.sin(gamma)  # x del punto dove posizionerò il tool sul cilindro
        p_end_cyl[1] = helmet_center[1] + R_CYLINDER * np.cos(gamma)  # y del punto dove posizionerò il tool sul cilindro
        p_end_cyl[2] = p_end_def[2]  # z del punto dove posizionerò il tool sul cilindro, uguale alla z del difetto

        #trovo anche le rotazioni desiderate dell'ee -> z verso il centro parallela a terra e x verso il basso (CREDO DA VERIFICARE)
        y_axis = np.array([0, 0, -1])  # y verso il basso
        z_axis = np.array([-np.sin(gamma), -np.cos(gamma), 0]) # z verso il centro del casco, parallelo a terra
        x_axis = np.cross(y_axis, z_axis)  # x per completare la base ortonormale

        # FORSE QUA GLI ANGOLI VANNO INVERTITI PERCHé SIAMO QUASI SEMPRE NELLA PARTE IN CUI LA TELECAMERA è A TESTA IN GIù

        cyl_R_mat = np.column_stack((x_axis, y_axis, z_axis))  # matrice di rotazione per l'orientamento cilindrico
        r_end_cyl = kin.rot_matrix_to_angles_zyx(cyl_R_mat) # rotazione obiettivo del tool per il movimento cilindrico

        # alla fine farò un movimento lineare fino a questa pose
        ee_end_cyl = kin.compute_ee_pose_for_tool_target(p_end_cyl, r_end_cyl, tool_pose_ee=tool_pose_ee)

        #sovrascrivo end_alpha e end_beta con quelli del punto di intersezione tra cilindro e sfera
        p_intersection = np.array([p_end_cyl[0], p_end_cyl[1], Z_CYLINDER])
        end_alpha, end_beta = kin.to_helmet_angles(p_intersection, helmet_center)[1:]
        # in questo modo mi muoverò sfericamente al punti di intersezione
    
    # --- 3. Controllo Sicurezza Traiettoria ---
    if angles_unsafe(end_alpha, end_beta):
        print(f"  [SKIP] Destinazione (alpha={end_alpha:.1f}°, beta={end_beta:.1f}°) fuori limiti sicurezza.")
        # nel dubbio richeckiamo se la traiettoria è sicura, se no skippiamo
        return False
    # Se il segmento taglia una zona pericolosa, deviamo passano per l'apice (0, 90) che è sempre sicuro.
    if is_trajectory_unsafe(start_alpha, start_beta, end_alpha, end_beta):
        print(f"  [SAFETY] Traiettoria non sicura. Deviazione tramite l'apice del casco.")
        apex_alpha, apex_beta = 0.0, 90.0
        actually_move(start_alpha, start_beta, apex_alpha, apex_beta)

        start_alpha, start_beta = apex_alpha, apex_beta  # Aggiorno il punto di partenza dopo la deviazione

    # --- 4. Suddivisione Archi Ampi (Prevenzione Errori Controller) ---
    # Dato che alpha è limitato a +/- 90, l'arco massimo teorico è 180°.
    # Un solo split a metà garantisce che il robot debba gestire archi <= 90°.
    if abs(end_alpha - start_alpha) > 90 or abs(end_beta - start_beta) > 90:
        print("  [SPLIT] Traiettoria sferica ampia. Suddivisione in due segmenti.")
        mid_a = np.mean([start_alpha, end_alpha])
        mid_b = np.mean([start_beta, end_beta])
        actually_move(start_alpha, start_beta, mid_a, mid_b)
        actually_move(mid_a, mid_b, end_alpha, end_beta)
    else:
        actually_move(start_alpha, start_beta, end_alpha, end_beta)

    # se il punto finale era sul cilindro end alpha ed end beta sono il punto nella giunzione tra cilindro e sfera,
    # quindi ho bisogno di un ultimo movimento lineare per scendere sul cilindro

    if ends_on_cylinder:
        print(f"  [CYL] Punto finale sul cilindro, esecuzione movimento lineare finale verso {np.round(p_end_cyl, 2)}.")
        controller.move_line(ee_end_cyl, speed=speed)
        
    print(f"  [SUCCESS] Movimento completato verso il difetto in (alpha={end_alpha:.1f}°, beta={end_beta:.1f}°).")
    #riprendiamo le coordinate e plottiamole per debug
    ee_pose_final = controller.robot.tcp_coord
    H_ee_to_glob_final = kin.create_homogeneous_matrix(ee_pose_final)
    tool_position_global_final = kin.homogeneous_trasform(H_ee_to_glob_final, tool_position_ee)
    print(f"  Posizione finale del tool in coordinate globali: {np.round(tool_position_global_final, 2)}")
    print(f"  Posizione del punto da guardare in coordinate globali: {np.round(p_end_def, 2)}")
    return True



if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import Variables as vb
    import kinematics_v2 as kin

    print("Generazione della MESH 3D dello spazio di lavoro...")

    # 1. Definizione della griglia angolare (100x100 basta per una mesh fluida)
    n_points = 50
    alpha_range = np.linspace(-180, 180, n_points)
    beta_range = np.linspace(0, 180, n_points)
    
    # Creiamo le matrici 2D per Alpha e Beta necessari a plot_surface
    Alpha, Beta = np.meshgrid(alpha_range, beta_range)
    
    # Matrici per contenere i punti cartesiani finali della mesh
    X = np.zeros_like(Alpha)
    Y = np.zeros_like(Alpha)
    Z = np.zeros_like(Alpha)
    
    # Matrice RGB per i colori di ogni singolo punto (R, G, B)
    # Inizializziamo tutto a zero
    colors = np.zeros((n_points, n_points, 3))

    # 2. Calcolo dei punti 3D e assegnazione dei colori della mesh
    radius = 200
    
    for i in range(n_points):
        for j in range(n_points):
            a = Alpha[i, j]
            b = Beta[i, j]
            
            # Trasformazione cinematica cartesiana
            sph_coord = [radius, a, b]
            p_global, _ = kin.to_helmet_coordinates(sph_coord, HELMET_CENTER_GLOBAL)
            
            X[i, j] = p_global[0]
            Y[i, j] = p_global[1]
            Z[i, j] = p_global[2]
            
            # Assegnazione colore in base alla sicurezza
            if angles_unsafe(a, b) or p_global[2] < Z_LIMIT:
                colors[i, j] = [0.85, 0.15, 0.15]  # Rosso opaco (Unsafe)
            else:
                colors[i, j] = [0.12, 0.47, 0.71]  # Blu classico (Safe)

    # 3. Configurazione del Plot 3D della Superficie
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # Plottiamo la superficie solida (Mesh)
    # facecolors accetta la nostra matrice RGB customizer per colorare le zone
    mesh = ax.plot_surface(
        X, Y, Z, 
        facecolors=colors, 
        linewidth=0, 
        antialiased=True, 
        shade=True,      # Attiva le ombre per dare profondità 3D tridimensionale
        alpha=0.8        # Leggera trasparenza globale
    )

    # 4. Disegnamo il centro del casco (Stella nera)
    ax.scatter(
        HELMET_CENTER_GLOBAL[0], HELMET_CENTER_GLOBAL[1], HELMET_CENTER_GLOBAL[2], 
        color='black', marker='*', s=200, zorder=10, label='Centro Casco'
    )

    # 5. Estetica, proporzioni e scritte
    ax.set_title("Superficie Mesh 3D dello Spazio di Lavoro\n[ Blu = SAFE  |  Rosso = UNSAFE ]", fontsize=14, weight='bold')
    ax.set_xlabel("Asse X Globale [mm]", fontsize=10)
    ax.set_ylabel("Asse Y Globale [mm]", fontsize=10)
    ax.set_zlabel("Asse Z Globale [mm]", fontsize=10)

    # Forziamo le proporzioni isometriche (fondamentale per non vedere la sfera deformata in un uovo)
    try:
        ax.set_box_aspect([1,1,1])
    except NotImplementedError:
        pass

    # Creiamo una legenda manuale pulita visto che plot_surface non supporta direttamente i label
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#1f77b4', edgecolor='none', alpha=0.8, label='Superficie SAFE'),
        Patch(facecolor='#d62728', edgecolor='none', alpha=0.8, label='Superficie UNSAFE (Limiti)')
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    print("Mesh generata con successo! Muovi il grafico per osservare la calotta.")
    plt.show()