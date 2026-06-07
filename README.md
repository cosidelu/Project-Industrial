# Branch di generalizzazione apertura robot

Questo README descrive come rendere l'apertura della visiera e il posizionamento del marker compatibili con robot diversi, sfruttando la posizione del casco (`helmet position`) come riferimento comune.

## Obiettivo di questa branch
- Generalizzare l'apertura fasciale per altri robot senza riscrivere l'intera pipeline.
- Usare la posizione del casco come riferimento comune per ri-calcolare le pose dei marker.
- Applicare un cambio di base matematico per mappare le pose originali nel nuovo robot.

## Principio di funzionamento
In `Variables.py` il codice costruisce due riferimenti:
- `BASE_MARKER_POSE`: posa del marker/camera nel sistema di riferimento originale.
- `NEW_MARKER_POSE`: posa del marker/camera nel nuovo robot target.

La trasformazione avviene così:
1. converti la posa originale in coordinate locali del marker (`local_start_marker`)
2. riconverti queste coordinate locali nel nuovo robot usando `NEW_MARKER_POSE` (`global_final_marker`)

Questo approccio evita di dover riscrivere manualmente tutte le pose di apertura e marcatura.

## Cosa modificare in `Variables.py`

### Parametri da aggiornare per un nuovo robot
- `CAMERA_POSE_EE`: offset della telecamera rispetto all'End-Effector del nuovo robot.
- `MARKER_POSE_EE`: offset del marker rispetto all'End-Effector del nuovo robot.
- `NEW_MARKER_POSE`: posa del marker nel nuovo robot target.
- `rot_z`: rotazione di offset attorno all'asse Z da applicare alla posa finale del marker.
- `LOOK_DOWN_POSITION_J_INIZIO`: posizione di partenza dei giunti del robot target.

### Cosa rimane invariato
- `BASE_MARKER_POSE` deve rimanere il riferimento originale del dataset iniziale.
- Le funzioni `local_start_marker()` e `global_final_marker()` rimangono le stesse.
- `HELMET_CENTER_GLOBAL` viene ricostruito automaticamente passando dal sistema originale al nuovo robot.

### Variabili critiche
- `BASE_MARKER_POSE`: dato originale che descrive il marker/camera nello stesso sistema di riferimento usato per le pose base.
- `NEW_MARKER_POSE`: descrive il marker/camera nel sistema del robot nuovo.
- `HELMET_CENTER_GLOBAL`: centro del casco in coordinate globali del robot corrente.

Se uno di questi valori è errato, tutte le pose derivate saranno sbagliate.

## Come usare `helmet position` per estrarre la nuova posizione dei marker

1. Calibra la posizione del casco originale in `Variables_global.py` (es. `vbg.HELMET_CENTER_GLOBAL`).
2. Misura la posa del marker/camera relativa all'EE del nuovo robot e inseriscila in `NEW_MARKER_POSE`.
3. Mantieni `BASE_MARKER_POSE` uguale al riferimento originale usato per la calibrazione del primo robot.
4. Lascia che `HELMET_CENTER_GLOBAL` venga ricalcolato da:
   - `local_start_marker(vbg.HELMET_CENTER_GLOBAL)`
   - `global_final_marker(...)`
5. Tutte le pose di apertura come `VISIERA_0`, `VISIERA_1`, ..., `LUNA_6`, ecc. saranno generate automaticamente nel nuovo sistema di riferimento.

## Cosa cambiare in `Variables_global.py`

In `Variables_global.py` si mantengono le pose raw misurate nel robot di partenza:
- `HELMET_CENTER_GLOBAL`
- `VISIERA_0`, `VISIERA_1`, ..., `LUNA_6`
- `ALLONTANAMENTO`
- `LOOK_DOWN_POSITION`

Non è necessario modificare `Variables_global.py` a meno che non si stia anche cambiando la calibrazione fisica del casco o del marker.

## Aspetti matematici

### Matrici omogenee
Una posa `pose = [x, y, z, rx, ry, rz]` viene trasformata in una matrice omogenea 4x4 `H`.
Questa matrice combina:
- rotazione `R` costruita da `rx, ry, rz`
- traslazione `T` data da `x, y, z`

La formula base è:

```text
p_new = H @ [x, y, z, 1]
```

### Cambio di riferimento
La trasformazione di interesse in `Variables.py` è:

```python
p_local = H_marker_inv @ p_global
p_new_global = H_marker_new @ p_local
```

In termini algebrici:

```text
p_new_global = H_marker_new @ H_marker_inv @ p_global
```

Questo significa che stiamo prendendo una posa definita nel vecchio sistema di riferimento e la stiamo ricostruendo nel nuovo robot attraverso un cambio di base.

### Perché è utile
- `H_marker_inv` annulla il effetto della posa di riferimento originale.
- `H_marker_new` applica la posa del marker nel robot target.
- Il risultato è una posa con la stessa relazione geometrica rispetto al casco, ma espressa nello spazio del robot target.

### Punto chiave: `HELMET_CENTER_GLOBAL`
Questa variabile rappresenta il centro del casco nello spazio del robot.
Tutte le traiettorie sferiche, le rotazioni e i movimenti di `spherical_movement.py` fanno riferimento a questo punto.

Se `HELMET_CENTER_GLOBAL` è corretto, le funzioni sferiche possono lavorare con gli angoli `alpha`, `beta` e `r` senza conoscere il robot specifico.

## Workflow consigliato per un nuovo robot

1. Calibra il marker/camera rispetto all'EE del nuovo robot.
2. Imposta `NEW_MARKER_POSE` in `Variables.py`.
3. Verifica `CAMERA_POSE_EE` e `MARKER_POSE_EE`.
4. Mantieni `BASE_MARKER_POSE` come riferimento originale.
5. Testa le trasformazioni con funzioni di debug prima di muovere il robot.
6. Usa movimenti PTP brevi e poi estendi a movimenti di apertura completo.

## Note finali
Questa branch separa la geometria del casco dalla base robotica. Il cuore del sistema rimane riutilizzabile: basta aggiornare il riferimento del marker per ogni nuovo robot, e il resto del codice viene automaticamente ricostruito dalla pipeline.
