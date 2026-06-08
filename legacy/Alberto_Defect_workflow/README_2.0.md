# PROGETTO ROBOTICA INDUSTRIALE – Rilevamento e Marcatura Difetti

Sistema automatizzato che usa una telecamera ZED e un manipolatore Techman per rilevare difetti visivi (macchie verdi) su un casco, localizzarli in 3D e marcarli fisicamente.

La struttura dati centrale è l'oggetto `defect` (da `camera_scripts_v2.py`), popolato progressivamente attraverso le fasi del sistema.

---

## FILE PRINCIPALI

-   `Test.py`: Entry point — orchestra tutte le fasi.
-   `camera_scripts_v2.py`: Classe `defect`, acquisizione ZED, rilevamento verde.
-   `kinematics_v2.py`: Trasformazioni omogenee, coordinate sferiche.
-   `robot_control.py`: Wrapper bloccante su Techman (PTP, Line, Circle).
-   `Variables.py`: Costanti di sistema (centro casco, offset camera/marker).

---

## FASE 1 – Inizializzazione

```python
controller = RobotController()
controller.connect()
zed, runtime, image_zed, point_cloud = init_zed()
controller.default_positioning()
```

---

## FASE 2–3 – Acquisizione e stima 3D locale → `take_defects()`

```python
defect_list, bgr_image = take_defects(runtime, zed, image_zed, point_cloud)
```

**Internamente:**

1.  Acquisisce immagine e point cloud dalla ZED.
2.  Segmenta le macchie verdi in HSV → popola `mask`, `centroid`, `area`, `bgr_img`.
3.  Per ogni maschera estrae i punti 3D validi dal point cloud e ne calcola la media → popola `pos3d_camera`.

---

## FASE 4 – Coordinate globali → `compute_global_coordinates()`

La camera è montata sull'EE (eye-in-hand): ad ogni scatto la matrice di trasformazione va aggiornata con la posa reale del robot.

```python
ee_pose_live    = controller.robot.tcp_coord
H_ee_to_global  = kin.create_homogeneous_matrix(ee_pose_live)
H_cam_to_global = H_ee_to_global @ H_cam_to_ee   # offset fisso da Variables
compute_global_coordinates(defect_list, H_cam_to_global)  # → popola defect.pos3d_global
```

---

## FASE 6 – Ispezione globale → `inspection_phase()`

Il robot percorre 5 angolazioni attorno al casco su una sfera di raggio `INSPECTION_RADIUS = 400 mm`, passando da un punto all'altro con `move_circle` senza mai tornare all'hub intermedio.

**Pattern:** `hub (PTP) → sc1 → sc2 → sc3 → sc4 → sc5 → hub (circle)`

I waypoint e i mid-point sono calcolati da `build_inspection_waypoints()` usando `helmet_coordinates()` (coordinate sferiche → posa EE compensata per `CAMERA_POSE_EE`).

**Ad ogni scatto:**

-   `take_defects()` acquisisce i difetti.
-   `compute_global_coordinates()` li trasforma nel frame globale (posa EE letta in real-time da `tcp_coord` dopo il `move_circle` bloccante).
-   I difetti vengono accumulati e infine filtrati dai duplicati tramite `filter_duplicate_defects()` con soglia euclidea di 10 mm.

---

## FASE 6b – Raffinamento → `refinement_phase()`

Per ogni difetto univoco il robot si porta a `CLOSE_INSPECTION_RADIUS = 400 mm` nella direzione radiale esatta del difetto (stesso alpha, stesso beta) e scatta `N_CLOSE_SHOTS = 10` foto ferme. La media delle stime valide aggiorna `pos3d_global`. I difetti non confermati vengono scartati.

```python
# 1. Calcola angoli del difetto
_, alpha, beta = kin.find_defect_angles(defect.pos3d_global, helmet_center)

# 2. Calcola posa di ispezione ravvicinata
p, r = kin.helmet_coordinates([CLOSE_INSPECTION_RADIUS, alpha, beta], helmet_center)

# 3. Muovi il robot
controller.move_ptp(compute_ee_pose_for_camera_target(p, r))

# 4. Scatta 10 foto, calcola la media e aggiorna defect.pos3d_global
```

---

## FASE 7 – Marcatura → `marking_phase()`

Per ogni difetto confermato:

1.  `find_defect_angles()` ricava `(r, alpha, beta)` del difetto.
2.  `helmet_coordinates()` calcola la posa con il marker perpendicolare al casco.
3.  La posa EE viene compensata per `MARKER_POSE_EE`:
    ```python
    t_ee = p_obj - R_ee @ MARKER_POSE_EE[:3]
    ```
4.  `move_ptp` al punto di approccio (`max(r+50, 400) mm`) → `move_line` al difetto (marcatura) → `move_line` di arretramento.

Tra un difetto e l'altro il robot transita sempre per l'hub tramite `safe_transit_via_hub()`.

---

## FASE 8 – Terminazione

```python
controller.default_positioning()
controller.disconnect()
zed.close()
cv2.destroyAllWindows()
```