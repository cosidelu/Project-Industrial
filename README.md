# Progetto di Robotica Industriale: Workflow Completo di Ispezione e Marcatura

Questo documento descrive il workflow operativo completo per il sistema di ispezione e marcatura automatizzata di difetti su un casco. Il processo è suddiviso in fasi logiche, dall'inizializzazione dell'hardware all'interazione fisica con i difetti rilevati, illustrando l'uso combinato dei vari moduli software.

---

## Architettura e Moduli del Progetto

Il sistema è fortemente modularizzato per garantire una separazione netta tra controllo hardware, visione artificiale e calcolo matematico. I file principali che compongono la pipeline sono:

- **`Variables.py`**: File di configurazione centrale. Contiene le costanti globali del sistema, il centro sferico spaziale del casco e le pose di offset degli utensili (es. `CAMERA_POSE_EE`, `MARKER_POSE_EE`).
- **`robot_control.py`**: Astrazione per il controllo del manipolatore Techman. Gestisce la connessione Modbus TCP, il timeout e i comandi di base di movimento sincrono e bloccante (`move_ptp`, `move_line`, `move_circle`).
- **`camera_scripts_v2.py`**: Modulo di visione artificiale. Comunica con le SDK della ZED, segmenta lo spazio colore delle immagini 2D e interroga la point cloud per estrarre posizioni 3D locali. Qui è definita la struttura dati `defect`.
- **`kinematics_v2.py`**: Motore matematico. Gestisce la cinematica spaziale (creazione matrici omogenee Eye-in-Hand, trasformazioni) e la complessa conversione tra il sistema sferico della calotta del casco e il sistema cartesiano globale.
- **`defects_id_wrapper.py`**: Orchestratore di elaborazione dati. Unisce la visione locale alla cinematica globale calcolando la posa reale dei difetti. Include logiche di pulizia ambientale e filtri per l'eliminazione dei duplicati tra scatti sovrapposti.
- **`spherical_movement.py`**: Modulo per il moto sicuro e l'elusione delle collisioni. Consente al robot di ruotare fluidamente sulla superficie del casco evitando matematicamente l'ingombro della base o del viso, dividendo in autonomia traiettorie ampie o deviandole per l'apice.

---

## Fase 1: Inizializzazione e Posizionamento di Sicurezza

Prima di iniziare qualsiasi operazione, è necessario stabilire la connessione con il robot e la telecamera, e portare il braccio in una posizione di partenza nota e sicura.

```python
import cv2
from robot_control import RobotController
from camera_scripts_v2 import init_zed
import Variables as vb

# 1. Inizializzazione e connessione al robot
# Si passa la configurazione dei giunti per la posa di default.
controller = RobotController(
    ip_address="192.168.19.22", 
    default_position_j=vb.LOOK_DOWN_POSITION_J_INIZIO
)
controller.connect()
print("Robot connesso.")

# 2. Spostamento nella posizione di default
controller.default_positioning()
print("Robot in posizione di default.")

# 3. Inizializzazione della telecamera ZED
zed, runtime, image_zed, point_cloud = init_zed()
print("Telecamera ZED inizializzata.")

# 4. Creazione finestre di debug
cv2.namedWindow("Debug Rilevamenti")
```

---

## Fase 2: Ispezione Globale (Movimento e Scatto)

Il robot esegue una scansione attorno al casco, muovendosi tra una serie di punti di osservazione predefiniti. Da ogni punto, acquisisce un'immagine e rileva i potenziali difetti.

1.  **Definizione Waypoint**: Si definisce una lista di coordinate sferiche (`alpha`, `beta`) che rappresentano i punti da cui scattare le foto.
2.  **Movimento Sferico**: La funzione `move_circle_spherical` gestisce il movimento sicuro tra i punti, evitando collisioni e singolarità.
3.  **Acquisizione 3D**: Una volta in posizione, `take_defects_global` scatta la foto, rileva le macchie verdi e calcola la loro posizione 3D nel sistema di riferimento globale del robot.
4.  **Accumulo**: I difetti di tutti gli scatti vengono raccolti in un'unica lista.

```python
import numpy as np
import kinematics_v2 as kin
from spherical_movement import move_circle_spherical
from defects_id_wrapper import take_defects_global

# Lista dei punti di osservazione sferici [Raggio, Alpha, Beta]
INSPECTION_RADIUS = 400.0
inspection_points = [
    np.array([INSPECTION_RADIUS, 0, 90]),    # Apice
    np.array([INSPECTION_RADIUS, 0, 45]),    # Fronte
    np.array([INSPECTION_RADIUS, 90, 60]),   # Lato destro
    np.array([INSPECTION_RADIUS, -90, 60]),  # Lato sinistro
]

all_defects = []
H_cam_to_ee = kin.create_homogeneous_matrix(vb.CAMERA_POSE_EE)

print("Inizio ispezione globale...")
for point in inspection_points:
    print(f"Movimento verso (alpha={point[1]}°, beta={point[2]}°)")
    
    # 2. Movimento sicuro verso il punto di scatto
    move_circle_spherical(
        controller=controller,
        end_sph_coord=point,
        radius=INSPECTION_RADIUS,
        tool_pose_ee=vb.CAMERA_POSE_EE,
        helmet_center=vb.HELMET_CENTER_GLOBAL,
        speed=300
    )
    
    # 3. Acquisizione e calcolo coordinate globali
    ee_pose_live = controller.robot.tcp_coord
    H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
    H_cam_to_global = H_ee_to_global @ H_cam_to_ee
    
    defect_list, bgr_image = take_defects_global(
        runtime, zed, image_zed, point_cloud, 
        H_cam_to_global=H_cam_to_global,
        cylindrical_filter=True # Filtra rumore 3D locale
    )
    
    # 4. Accumulo dei risultati
    all_defects.extend(defect_list)
    print(f"    Rilevati {len(defect_list)} difetti.")
```

---

## Fase 3: Filtraggio dei Duplicati

Poiché i campi visivi degli scatti si sovrappongono, lo stesso difetto fisico può essere rilevato più volte. La funzione `duplicate_filter` analizza la lista completa e rimuove i cloni basandosi sulla loro vicinanza nello spazio 3D.

```python
from defects_id_wrapper import duplicate_filter

print(f"\nDifetti totali rilevati (pre-filtraggio): {len(all_defects)}")

# Filtra i duplicati con una tolleranza di 25mm
unique_defects = duplicate_filter(all_defects, distance_threshold=25.0)

print(f"Difetti unici trovati: {len(unique_defects)}")
for i, d in enumerate(unique_defects):
    print(f"  - Difetto {i+1}: {np.round(d.pos3d_global, 1)} mm")
```

---

## Fase 4: Ciclo di Lavoro sui Difetti (Raffinamento e Marcatura)

Per ogni difetto unico identificato, il sistema esegue un ciclo di "look and mark": si avvicina per ottenere una misura più precisa e poi esegue la marcatura fisica.

### 4.1. Posizionamento per il Raffinamento

Il robot si sposta in modo che la telecamera sia allineata radialmente con il difetto, ma a una distanza di sicurezza. Questo garantisce una visuale perpendicolare e centrata, ottimale per la stima 3D.

```python
# Esempio per il primo difetto della lista
target_defect = unique_defects[0]
CLOSE_INSPECTION_RADIUS = 350.0 # Distanza di sicurezza per il refining

# 1. Calcola gli angoli sferici del difetto rispetto al centro del casco
r, alpha, beta = kin.to_helmet_angles(
    target_defect.pos3d_global, 
    vb.HELMET_CENTER_GLOBAL
)

# 2. Definisci la posa di osservazione (stessi angoli, raggio di sicurezza)
refining_sph_coord = np.array([CLOSE_INSPECTION_RADIUS, alpha, beta])

# 3. Muovi il robot in posizione
print(f"\nPosizionamento per raffinare il difetto a (alpha={alpha:.1f}°, beta={beta:.1f}°)")
move_circle_spherical(
    controller=controller,
    end_sph_coord=refining_sph_coord,
    radius=CLOSE_INSPECTION_RADIUS,
    tool_pose_ee=vb.CAMERA_POSE_EE,
    helmet_center=vb.HELMET_CENTER_GLOBAL
)
```

### 4.2. Raffinamento della Posizione 3D

Una volta in posizione, il sistema scatta una serie di foto consecutive (senza muoversi) e media le posizioni 3D rilevate per ottenere una stima finale estremamente accurata, che andrà a sovrascrivere quella precedente.

```python
N_CLOSE_SHOTS = 10
refined_positions = []
old_pos = target_defect.pos3d_global

print(f"Avvio raffinamento con {N_CLOSE_SHOTS} scatti...")
for i in range(N_CLOSE_SHOTS):
    # Matrice di trasformazione (costante, il robot è fermo)
    ee_pose_live = controller.robot.tcp_coord
    H_ee_to_global = kin.create_homogeneous_matrix(ee_pose_live)
    H_cam_to_global = H_ee_to_global @ H_cam_to_ee
    
    # Scatto e rilevamento
    defect_list, _ = take_defects_global(runtime, zed, image_zed, point_cloud, H_cam_to_global)
    
    # Associa il difetto rilevato a quello target (il più vicino)
    if defect_list:
        # Trova il difetto più vicino alla stima originale
        best_match = min(defect_list, key=lambda d: np.linalg.norm(d.pos3d_global - old_pos))
        dist = np.linalg.norm(best_match.pos3d_global - old_pos)
        
        # Se è abbastanza vicino, consideralo una misura valida
        if dist < 30.0: # Soglia di associazione in mm
            refined_positions.append(best_match.pos3d_global)
            print(f"  Scatto {i+1}: OK (dist={dist:.1f} mm)")

# Se sono state raccolte misure valide, calcola la media e aggiorna
if refined_positions:
    refined_pos = np.mean(np.array(refined_positions), axis=0)
    target_defect.pos3d_global = refined_pos
    delta = np.linalg.norm(refined_pos - old_pos)
    print(f"Posizione raffinata: {np.round(refined_pos, 1)} mm (Spostamento: {delta:.1f} mm)")
```

### 4.3. Marcatura del Difetto

Con la posizione 3D finale, il sistema calcola la posa necessaria per portare la punta del marker a toccare il difetto. Il movimento è scomposto in un avvicinamento rapido e un tocco finale lento e lineare.

```python
# Posizione finale del difetto (raffinata)
final_defect_pos = target_defect.pos3d_global

# 1. Calcola la posa target per il MARKER (non più per la camera)
#    La posa deve essere perpendicolare alla superficie del casco nel punto del difetto.
r, alpha, beta = kin.to_helmet_angles(final_defect_pos, vb.HELMET_CENTER_GLOBAL)
p_obj, r_obj = kin.to_helmet_coordinates([r, alpha, beta], vb.HELMET_CENTER_GLOBAL)

# 2. Calcola la posa dell'End-Effector compensando l'offset del marker
ee_marking_pose = kin.compute_ee_pose_for_tool_target(
    p_obj, r_obj, tool_pose_ee=vb.MARKER_POSE_EE
)

# 3. Calcola una posa di pre-avvicinamento (50mm indietro lungo la stessa linea)
p_approach, _ = kin.to_helmet_coordinates([r + 50.0, alpha, beta], vb.HELMET_CENTER_GLOBAL)
ee_approach_pose = kin.compute_ee_pose_for_tool_target(
    p_approach, r_obj, tool_pose_ee=vb.MARKER_POSE_EE
)

# 4. Esegui la sequenza di marcatura
print("\nAvvio sequenza di marcatura...")
# Prima ci si sposta al punto di pre-avvicinamento
controller.move_ptp(ee_approach_pose, speed=100)
# Poi si esegue il tocco finale in linea retta
controller.move_line(ee_marking_pose, speed=25)
# Infine, ci si ritrae tornando al punto di pre-avvicinamento
controller.move_line(ee_approach_pose, speed=50)
print("Marcatura completata.")
```

---

## Fase 5: Chiusura

Al termine di tutte le operazioni, è fondamentale rilasciare le risorse hardware in modo pulito.

```python
# Riporta il robot alla posizione di default
controller.default_positioning()

# Disconnetti robot e telecamera
controller.disconnect()
zed.close()
cv2.destroyAllWindows()
print("\nSistema terminato correttamente.")
```
