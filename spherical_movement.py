import kinematics_v2 as kin
from robot_control import RobotController

from Variables import HELMET_CENTER_GLOBAL

import cv2
import numpy as np
import time

# Altezza del cilindro in cui switchamo a movimento cilindrico DA REGOLARE
# DEVE ESSERE MAGGIORE DELLA Z DI HELMET CENTER PER EVITARE IL LOCK
Z_CYLINDER = HELMET_CENTER_GLOBAL[2] + 50  
SPHERE_RADIUS = 300 #DEVE ESSERE UGUALE ALL'INSPECTION RADIUS
R_CYLINDER = np.sqrt(SPHERE_RADIUS**2 - Z_CYLINDER**2)  # Raggio del cilindro alla giunzione con la sfera
Z_LIMIT = 100  # Limite assoluto di altezza sotto il quale non è sicuro muoversi (es. base del casco o tavolo)

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

    # 2. Controllo limite laterale: alpha deve essere compreso entro +/- 120 gradi
    if alpha > 105.0 or alpha < -95.0:
        return True

    # 3. Controllo zona posteriore (retro): più larga che alta
    # A alpha = 0 la soglia è 20, a alpha = 90 la soglia sale a 30
    beta_soglia_retro = 15.0 + 26.0 * (alpha / 90.0)**2
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

def move_circle_spherical(controller, end_sph_coord, radius, tool_pose_ee, helmet_center=HELMET_CENTER_GLOBAL, z_cyl=Z_CYLINDER, speed=300):
    """
    Esegue un movimento circolare da una posizione corrente a una posizione finale
    definita da angoli sferici (alpha, beta) attorno al casco.
    La cinematica è sicura dai gimbal lock poiché i poli (beta=0, beta=180) 
    sono esclusi dalle limitazioni di sicurezza.

    PERCHé FUNZIONI IL MOVIMENTO CILINDRICO è IMPORTANTE CHE LA IL end_sph_coord[0] SIA IL RAGGIO DEL DIFETTO\PUNTO CHE VOGLIO GUARDARE
    """

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


    # --- Verfico se sto partendo dal cilindro e se si mi sposto sulla sfera e aggiorno le coordinate globali del tool ---

    if tool_position_global[2] < z_cyl:
        print(f"   [CYL] Partenza da cilidnro, mi sposto prima sulla sfera a z={z_cyl}mm")
        tool_position_global[2] = z_cyl

        # a questo punto mi sposto sulla posizione aggiornata del tool con le stesse rotazioni iniziali
        ee_pose_cyl = kin.compute_ee_pose_for_tool_target(tool_position_global[:3], ee_pose[:3], tool_pose_ee)
        controller.move_line(ee_pose_cyl, speed=speed)

        time.sleep(1)  # breve pausa per stabilizzare il movimento
        # Ricalcolo la posa finale del tool dopo la deviazione cilindrica
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

    # --- 2. Controllo Sicurezza Destinazione --- TENIAMO GLI ANGOLI PER LE ZONE DI SICUREZZA MA VANNO AGGIORNATI
    if angles_unsafe(end_alpha, end_beta):
        print(f"  [SKIP] Destinazione (alpha={end_alpha:.1f}°, beta={end_beta:.1f}°) fuori limiti sicurezza.")
        return False
    
    # --- Controllo se la fine è nel ciilindro
    ends_on_cylinder = False
    # calcolo la z finale del punto target
    p_end, _ = kin.to_helmet_coordinates(end_sph_coord, helmet_center)
    if p_end[2] < z_cyl:
        if p_end[2] < Z_LIMIT:
            print(f"  [ERROR] Destinazione finale a z={p_end[2]:.1f} mm, che è sotto il limite assoluto di {Z_LIMIT} mm. Movimento rifiutato.")
            return False
        
        print(f"  [CYL] Destinazione finale prevista a z={p_end[2]:.1f} mm, che è sotto la soglia cilindrica di {z_cyl} mm.")
        ends_on_cylinder = True
        

        #calcolo il punto finale di ispezione sul cilindro alla stessa altezza del difetto


        #sovrascrivo end_alpha e end_beta con quelli del punto di intersezione tra cilindro e sfera
        p_intersection = np.array([p_end[0], p_end[1], z_cyl])
    
    # --- 3. Controllo Sicurezza Traiettoria ---
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
        return True

    # --- 5. Esecuzione Traiettoria Diretta ---
    actually_move(start_alpha, start_beta, end_alpha, end_beta)

    # se il punto finale era sul cilindro end alpha ed end beta sono il punto nella giunzione tra cilindro e sfera,
    # quindi ho bisogno di un ultimo movimento lineare per scendere sul cilindro

    if ends_on_cylinder:
        print("hi")
        

    return True



if __name__ == "__main__":
    import matplotlib.pyplot as plt

    print("Generazione della mappa di sicurezza (Alpha vs Beta)...")

    # 1. Creazione della griglia di punti (Discretizzazione)
    # Generiamo 500 punti per asse per avere una risoluzione molto definita della mappa
    alpha_range = np.linspace(-120, 120, 500)
    beta_range = np.linspace(-15, 200, 500)
    
    Alpha, Beta = np.meshgrid(alpha_range, beta_range)
    
    # 2. Vettorizzazione della tua funzione angles_unsafe
    # Questo permette di applicare la funzione su tutta la matrice NumPy in un colpo solo
    vec_angles_unsafe = np.vectorize(angles_unsafe)
    unsafe_mask = vec_angles_unsafe(Alpha, Beta)

    # 3. Configurazione del Plot con Matplotlib
    plt.figure(figsize=(10, 8))
    
    # Creiamo una colormap personalizzata: Rosso per True (Unsafe), Blu per False (Safe)
    # Usiamo 'ListedColormap' per avere una distinzione netta senza sfumature
    from matplotlib.colors import ListedColormap
    custom_cmap = ListedColormap(['#1f77b4', '#d62728']) # Blu classico e Rosso acceso

    # Disegnamo la mappa bidimensionale
    mesh = plt.pcolormesh(
        Alpha, Beta, unsafe_mask, 
        cmap=custom_cmap, 
        shading='auto',
        alpha=0.85
    )

    # 4. Estetica del grafico, griglia e limiti
    plt.title("Mappa di Sicurezza Angolare del Casco\n[ Blu = SAFE  |  Rosso = UNSAFE ]", fontsize=14, pad=15, weight='bold')
    plt.xlabel("Angolo Alpha (Sinistra [-] / Destra [+]) [Gradi]", fontsize=11)
    plt.ylabel("Angolo Beta (Retro [0] / Apice [90] / Fronte [180]) [Gradi]", fontsize=11)
    
    # Disegnamo delle linee di riferimento per i limiti principali impostati nel codice
    plt.axvline(x=105, color='black', linestyle='--', alpha=0.7, label='Limiti Alpha (+105° / -95°)')
    plt.axvline(x=-95, color='black', linestyle='--', alpha=0.7)
    
    # Mostriamo dove sono l'apice e i limiti teorici di beta
    plt.axhline(y=90, color='white', linestyle=':', alpha=0.6, label='Apice (Beta = 90°)')
    plt.axhline(y=0, color='black', linestyle='-', alpha=0.5)
    plt.axhline(y=180, color='black', linestyle='-', alpha=0.5)

    # Legenda per le linee di riferimento
    plt.legend(loc='upper left', framealpha=0.9)
    
    # Configurazione griglia e limiti degli assi del grafico
    plt.grid(True, linestyle=':', color='black', alpha=0.3)
    plt.xlim(-120, 120)
    plt.ylim(-15, 195)
    
    # Mostra il grafico a schermo
    plt.show()